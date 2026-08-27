import gc
import os
import time
import math

# K230 GPIO/PWM 支持库
from machine import Pin, PWM
from media.sensor import *
from media.display import *
from media.media import *

# ================= 硬件与显示配置 =================
SENSOR_ID = 2
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
DETECT_WIDTH = 400
DETECT_HEIGHT = 240

DISPLAY_CHN = CAM_CHN_ID_0
DETECT_CHN = CAM_CHN_ID_1

# ================= 图像检测与 ROI 配置 =================

# Detection ROI (Region of Interest) in the 400x240 grayscale image.
# 这代表“蓝色矩形框”的位置和大小: (y/2, x/2, h/2, w/2)
BALL_ROI = (5, 100, 395, 41)

# Grayscale threshold: 0 is black and 255 is white.
GRAY_THRESHOLD = (0, 135)

# After binary(), pixels inside GRAY_THRESHOLD become white.
BINARY_WHITE_THRESHOLD = (210, 255)

# Blob 过滤参数
MIN_PIXELS = 50             # 钢球最小像素总数
MIN_SIZE = 6                # 最小直径(像素)
MAX_SIZE = 60               # 最大直径(像素)
MIN_ROUND_PERCENT = 10      # 最小圆度百分比
MIN_FILL_PERCENT = 0.5      # 最小填充率

# ================= PID 控制参数配置 (调参区) =================

# 1. 目标中心点 (Target Center)
#    X 轴目标：ROI 的水平中心
ROI_TARGET_X = BALL_ROI[0] + (BALL_ROI[2] // 2)  # 40 + 160 = 200

# 2. PID 增益系数 (针对 X 轴误差)
PID_KP = 2.0

#    Ki: 积分项，消除稳态误差。
PID_KI = 1

#    Kd: 微分项，抑制震荡。
PID_KD = 1.0

# 3. 死区 (Deadband)
#    当误差绝对值小于此值时，PID 偏移量为 0，输出保持默认中位值。
#    单位: 像素
ERROR_DEADBAND = 3

# 4. PWM (舵机) 输出配置
#    舵机标准频率: 50 Hz (周期 20ms = 20000us)
PWM_FREQ = 50

#    脉冲宽度限制 (单位: 微秒 us)
MIN_PULSE_US = 800   # 最小脉宽 (对应舵机一端极限)
MAX_PULSE_US = 1550  # 最大脉宽 (对应舵机另一端极限)

#    默认中位脉宽 (无误差/未激活时的输出)
DEFAULT_PULSE_US = 1100

# ================= 控制方向与输出滤波配置 =================

# 1. 执行器(舵机)方向极性
SERVO_POLARITY = -1

# 2. 输出平滑/滤波（解决舵机转动不平滑、抖动问题）
#    对 PID 输出的脉宽偏移量(offset_us)做平滑，再叠加到默认中位脉宽。
OUTPUT_FILTER_TYPE = "lpf"   # "lpf"=一阶低通 | "ma"=滑动平均 | "none"=不过滤
OUTPUT_LPF_ALPHA = 0.3       # 一阶低通系数: 取值 0~1，越小越平滑(响应越慢)，1=不过滤
OUTPUT_MA_WINDOW = 5         # 滑动平均窗口(帧数)，仅当 FILTER_TYPE="ma" 时生效

# ================= 运行流程控制 =================
DETECT_EVERY_N_FRAMES = 2
PRINT_EVERY_N_FRAMES = 10
GC_INTERVAL_FRAMES = 30

SCALE_X = DISPLAY_WIDTH // DETECT_WIDTH
SCALE_Y = DISPLAY_HEIGHT // DETECT_HEIGHT

# The 400x240 binary image occupies the lower-left quarter of the LCD.
BINARY_PREVIEW_X = 0
BINARY_PREVIEW_Y = DISPLAY_HEIGHT - DETECT_HEIGHT
BINARY_PREVIEW_WIDTH = DETECT_WIDTH
BINARY_PREVIEW_HEIGHT = DETECT_HEIGHT


# ================= PID 控制器类 =================
class XAxisPID:
    def __init__(self, kp, ki, kd, target_x,
                 polarity=1,
                 filter_type="lpf", lpf_alpha=0.3, ma_window=5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_x = target_x

        # 执行器方向极性：+1 标准 / -1 反向（与 Kp 符号解耦，Kp 恒为正）
        self.polarity = polarity

        # 输出滤波参数（平滑舵机转动）
        self.filter_type = filter_type
        self.lpf_alpha = lpf_alpha
        self.ma_window = ma_window
        self._last_filtered = 0.0   # 一阶低通上一拍输出
        self._ma_buf = []           # 滑动平均缓冲

        self.last_error = 0.0
        self.integral = 0.0

        # 积分限幅，防止积分饱和 (Integral Windup)
        # 这里限制积分项产生的最大脉宽偏移量，例如 +/- 200us
        self.integral_limit_us = 200.0

    def compute(self, current_x):
        """
        输入当前小球的 X 坐标，返回 PID 偏移量 (us) 和当前误差
        """
        # 计算误差: 目标 - 当前 (标准定义，保持为正向误差)
        error = self.target_x - current_x

        # --- 比例项 (P) ---
        p_out = self.kp * error

        # --- 积分项 (I) ---
        self.integral += error
        # 积分限幅 (转换为 us 单位进行限制)
        if self.integral > self.integral_limit_us / (self.ki if self.ki != 0 else 1):
             # 简单处理：如果 ki 为 0，不累加
            pass

        # 更稳健的积分限幅方式：限制 integral 变量本身代表的“等效误差累积”
        # 或者直接限制 i_out
        i_out = self.ki * self.integral
        if i_out > self.integral_limit_us:
            i_out = self.integral_limit_us
            self.integral = i_out / self.ki if self.ki != 0 else 0
        elif i_out < -self.integral_limit_us:
            i_out = -self.integral_limit_us
            self.integral = i_out / self.ki if self.ki != 0 else 0

        # --- 微分项 (D) ---
        derivative = error - self.last_error
        d_out = self.kd * derivative

        # 更新上次误差
        self.last_error = error

        # 总偏移量 (us)，并按执行器方向极性修正符号（Kp 始终为正）
        offset_us = self.polarity * (p_out + i_out + d_out)

        # 对输出做平滑滤波，抑制舵机抖动
        offset_us = self.filter_output(offset_us)
        return offset_us, error

    def filter_output(self, raw):
        """
        对 PID 输出偏移量做平滑滤波：
          - lpf: 一阶低通  y = alpha*raw + (1-alpha)*y_prev
          - ma : 滑动平均（最近 ma_window 个样本均值）
          - none: 原样返回
        """
        if self.filter_type == "lpf":
            self._last_filtered = (self.lpf_alpha * raw
                                   + (1.0 - self.lpf_alpha) * self._last_filtered)
            return self._last_filtered
        elif self.filter_type == "ma":
            self._ma_buf.append(raw)
            if len(self._ma_buf) > self.ma_window:
                self._ma_buf.pop(0)
            return sum(self._ma_buf) / len(self._ma_buf)
        else:
            return raw

    def reset(self):
        """重置 PID 状态"""
        self.last_error = 0.0
        self.integral = 0.0
        # 注意：滤波状态(_last_filtered/_ma_buf)有意保留，
        # 使死区/丢球后输出能平滑回到 0，而非瞬间跳变。


# ================= 图像处理函数 =================

def find_steel_ball_in_roi(binary_img):
    """
    仅在指定的 BALL_ROI 区域内寻找钢球
    """
    blobs = binary_img.find_blobs(
        [BINARY_WHITE_THRESHOLD],
        False,
        BALL_ROI,
        x_stride=1,
        y_stride=1,
        pixels_threshold=MIN_PIXELS,
        merge=False,
        margin=4
    )

    best = None
    best_score = 0

    for blob in blobs:
        w = blob.w()
        h = blob.h()
        pixels = blob.pixels()

        if w < MIN_SIZE or h < MIN_SIZE:
            continue

        if w > MAX_SIZE or h > MAX_SIZE:
            continue

        long_side = max(w, h)
        short_side = min(w, h)

        round_percent = short_side * 100 // long_side
        fill_percent = pixels * 100 // (w * h)

        if round_percent < MIN_ROUND_PERCENT:
            continue

        if fill_percent < MIN_FILL_PERCENT:
            continue

        score = pixels * round_percent

        if score > best_score:
            best_score = score
            best = (
                blob.x(),
                blob.y(),
                w,
                h,
                blob.cx(),
                blob.cy(),
                pixels
            )

    return best


def draw_result(img, ball, fps, pwm_us, err_x):
    # 缩放 ROI 到显示分辨率
    roi_x = BALL_ROI[0] * SCALE_X
    roi_y = BALL_ROI[1] * SCALE_Y
    roi_w = BALL_ROI[2] * SCALE_X
    roi_h = BALL_ROI[3] * SCALE_Y

    # 1. 绘制蓝色矩形框 (ROI)
    img.draw_rectangle(
        roi_x,
        roi_y,
        roi_w,
        roi_h,
        color=(0, 0, 255),  # 蓝色
        thickness=2
    )

    # 2. 绘制目标中心线 (X轴目标)
    target_draw_x = ROI_TARGET_X * SCALE_X
    img.draw_line(target_draw_x, roi_y, target_draw_x, roi_y + roi_h, color=(0, 255, 255), thickness=1)

    if ball is None:
        text = "NO BALL  FPS:%d" % int(fps)
        img.draw_string_advanced(10, 10, 28, text, color=(255, 0, 0))
        return

    # 缩放球的数据到显示分辨率
    x = ball[0] * SCALE_X
    y = ball[1] * SCALE_Y
    w = ball[2] * SCALE_X
    h = ball[3] * SCALE_X # Typo fix in original logic? No, w is width.
    h_disp = ball[3] * SCALE_Y
    cx = ball[4] * SCALE_X
    cy = ball[5] * SCALE_Y
    radius = (w + h_disp) // 4

    # 3. 绘制球
    img.draw_rectangle(x, y, w, h_disp, color=(0, 255, 0), thickness=3)
    img.draw_circle(cx, cy, radius, color=(255, 255, 0), thickness=3)
    img.draw_cross(cx, cy, color=(255, 0, 0), size=12, thickness=2)

    # 4. 绘制误差连线 (水平方向)
    img.draw_line((cx, cy, target_draw_x, cy), color=(255, 255, 255), thickness=2)

    # 5. 显示调试信息
    text = "X_ERR:%.1f PWM:%dus FPS:%d" % (err_x, pwm_us, int(fps))
    img.draw_string_advanced(10, 10, 28, text, color=(255, 0, 0))


def draw_binary_preview(display_img, binary_img, ball):
    if binary_img is None:
        return

    display_img.draw_image(binary_img, BINARY_PREVIEW_X, BINARY_PREVIEW_Y)

    # 绘制 ROI 边框
    display_img.draw_rectangle(
        BINARY_PREVIEW_X + BALL_ROI[0],
        BINARY_PREVIEW_Y + BALL_ROI[1],
        BALL_ROI[2],
        BALL_ROI[3],
        color=(0, 0, 255),
        thickness=2
    )

    # 绘制目标 X 线
    t_x = BINARY_PREVIEW_X + ROI_TARGET_X
    display_img.draw_line(
        t_x, BINARY_PREVIEW_Y + BALL_ROI[1],
        t_x, BINARY_PREVIEW_Y + BALL_ROI[1] + BALL_ROI[3],
        color=(0, 255, 255), thickness=1
    )

    if ball is not None:
        cx = BINARY_PREVIEW_X + ball[4]
        cy = BINARY_PREVIEW_Y + ball[5]
        display_img.draw_cross(
            cx,
            cy,
            color=(255, 0, 0),
            size=8,
            thickness=2
        )

        # 画水平误差线
        display_img.draw_line((cx, cy, t_x, cy), color=(255, 255, 255), thickness=1)

    display_img.draw_string_advanced(
        BINARY_PREVIEW_X + 8,
        BINARY_PREVIEW_Y + 6,
        20,
        "ROI X-TRACK",
        color=(255, 0, 0)
    )


# ================= 主程序 =================

sensor = None
last_ball = None
last_binary_img = None
miss_count = 0

# 初始化 PID 控制器 (仅 X 轴)
# Kp 取正数；方向由 SERVO_POLARITY 决定；输出经低通滤波平滑。
pid_controller = XAxisPID(
    kp=PID_KP, ki=PID_KI, kd=PID_KD, target_x=ROI_TARGET_X,
    polarity=SERVO_POLARITY,
    filter_type=OUTPUT_FILTER_TYPE,
    lpf_alpha=OUTPUT_LPF_ALPHA,
    ma_window=OUTPUT_MA_WINDOW
)


# ================= P13 引脚 PWM 舵机控制 (物理 P13 = FPIOA GPIO 42 -> PWM0) =================


from machine import FPIOA

# 初始化前先置空，避免 set_servo_pulse 在异常路径下引用未定义变量
pwm_motor = None


def set_servo_pulse(pulse_us):
    """
    K230 MicroPython PWM 实时更新（兼容 machine.PWM API）。
    不使用 enable()，每次直接写 freq()+duty()。
    返回实际写入硬件的 pulse_us，供调试打印。
    """
    if pwm_motor is None:
        return None

    pulse_us = int(max(MIN_PULSE_US, min(MAX_PULSE_US, pulse_us)))

    # 重新确认 P13 -> PWM0
    fpioa.set_function(42, FPIOA.PWM0)

    duty = int(round(pulse_us * 100 / 20000))

    pwm_motor.freq(PWM_FREQ)
    pwm_motor.duty(duty)

    return pulse_us


try:
    sensor = Sensor(id=SENSOR_ID)
    sensor.reset()

    sensor.set_framesize(
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        chn=DISPLAY_CHN
    )
    sensor.set_pixformat(Sensor.RGB565, chn=DISPLAY_CHN)

    sensor.set_framesize(
        width=DETECT_WIDTH,
        height=DETECT_HEIGHT,
        chn=DETECT_CHN
    )
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=DETECT_CHN)

    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True
    )

    MediaManager.init()
    sensor.run()

    # ============ PWM 舵机初始化（放在显示/媒体初始化之后）============
    # 让本程序最后配置 PWM0，避免被显示/媒体初始化时的引脚/时钟配置覆盖。
    # 运行期每帧由 set_servo_pulse() 重新设定 freq+duty+enable 以对抗外设占用。
    try:
        fpioa = FPIOA()
        fpioa.set_function(42, FPIOA.PWM0)   # 物理 P13 = IO42 -> PWM0
        pwm_motor = PWM(0)
        pwm_motor.freq(PWM_FREQ)             # 50Hz 舵机标准频率
        set_servo_pulse(DEFAULT_PULSE_US)    # 输出初始中位脉宽
        print("PWM0 initialized on PIN13, Pulse:", DEFAULT_PULSE_US, "us")
    except Exception as e:
        print("Warning: Could not init PWM on P13:", e)
        pwm_motor = None

    clock = time.clock()
    frame_id = 0

    # 保持当前控制状态，避免在非检测帧被重置为 0 / 默认值
    current_pwm_us = DEFAULT_PULSE_US
    current_err_x = 0.0

    print("X-Axis Servo Tracker Started")
    print("Target X: %d, Default PWM: %d us" % (ROI_TARGET_X, DEFAULT_PULSE_US))

    while True:
        os.exitpoint()
        clock.tick()

        display_img = sensor.snapshot(chn=DISPLAY_CHN)

        # 每隔 N 帧进行一次检测
        if frame_id % DETECT_EVERY_N_FRAMES == 0:
            detect_img = sensor.snapshot(chn=DETECT_CHN)

            # 复制并二值化
            binary_img = detect_img.copy()
            binary_img = binary_img.binary([GRAY_THRESHOLD])

            # 仅在 ROI 内寻找球
            ball = find_steel_ball_in_roi(binary_img)
            last_binary_img = binary_img

            if ball is not None:
                last_ball = ball
                miss_count = 0

                # --- PID 计算核心逻辑 (仅 X 轴) ---
                ball_cx_detect = ball[4]

                # 计算 X 轴误差
                err_x = ROI_TARGET_X - ball_cx_detect

                # 检查死区
                if abs(err_x) < ERROR_DEADBAND:
                    # 死区内也经过滤波，使输出平滑归零而非瞬间跳变
                    pid_offset = pid_controller.filter_output(0.0)
                    pid_controller.reset()
                else:
                    pid_offset, _ = pid_controller.compute(ball_cx_detect)

                # 计算最终 PWM 脉宽: 默认值 + PID 偏移
                final_pulse = DEFAULT_PULSE_US + pid_offset

                # 限幅
                if final_pulse > MAX_PULSE_US:
                    final_pulse = MAX_PULSE_US
                elif final_pulse < MIN_PULSE_US:
                    final_pulse = MIN_PULSE_US

                current_pwm_us = final_pulse
                current_err_x = err_x

            else:
                miss_count += 1
                if miss_count >= 2:
                    last_ball = None
                    # 丢失目标时，回归默认中位
                    current_pwm_us = DEFAULT_PULSE_US
                    current_err_x = 0.0
                    pid_controller.reset()

        # 绘制结果
        draw_result(display_img, last_ball, clock.fps(), int(current_pwm_us), current_err_x)
        draw_binary_preview(display_img, last_binary_img, last_ball)
        Display.show_image(display_img)

        # 更新 PWM 输出：放在 show_image 之后，确保本帧舵机占空比是最后写入的值，
        # 避免被显示子系统(背光 PWM)覆盖，从而保证运行期实时响应、无需 reset
        if pwm_motor:
            hw_pulse_us = set_servo_pulse(current_pwm_us)
        else:
            hw_pulse_us = None

        # 打印调试信息（软件值 + 实际写入值）
        if frame_id % PRINT_EVERY_N_FRAMES == 0:
            if last_ball is not None:
                cx = last_ball[4] * SCALE_X
                print("BALL,X:%d,ERR:%.2f,PWM_CMD:%dus,PWM_HW:%dus" %
                      (cx, current_err_x,
                       int(current_pwm_us),
                       int(hw_pulse_us) if hw_pulse_us is not None else -1))
            else:
                print("NO_BALL,PWM_CMD:%dus,PWM_HW:%dus" %
                      (int(current_pwm_us),
                       int(hw_pulse_us) if hw_pulse_us is not None else -1))

        if frame_id % 50 == 0 and pwm_motor:
            try:
                print("PWM_DIAG freq=%d duty=%d" %
                      (pwm_motor.freq(), pwm_motor.duty()))
            except Exception as e:
                print("PWM_DIAG failed:", e)

        frame_id += 1

        # 定期垃圾回收
        if frame_id % GC_INTERVAL_FRAMES == 0:
            gc.collect()

except KeyboardInterrupt:
    print("user stopped")

except Exception as e:
    print("error:", e)

finally:
    if pwm_motor:
        pwm_motor.deinit()

    if isinstance(sensor, Sensor):
        sensor.stop()

    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    gc.collect()
