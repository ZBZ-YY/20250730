import os
import math
import time
from machine import UART, FPIOA
from media.sensor import *
from media.display import *
from media.media import *
import gc

# 距离测算参数
KNOWN_WIDTH = 25.0       # 已知物体实际宽度(cm)
FOCAL_LENGTH = 550.0    # 相机焦距(像素)
MIN_RECT_AREA = 500     # 最小矩形面积(过滤小目标)

# 舵机控制参数
DEADZONE_MOTION = 1.5   # 运动死区（度）
LASER_Y_OFFSET = -0.6   # 激光偏置角度

# 初始化UART通信
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
uart = UART(UART.UART1, baudrate=115200)

# PID参数
Kp = 0.8   # 比例增益
Ki = 0.001 # 积分增益
Kd = 0.6   # 微分增益

# 通信数据包格式
HEADER = b'\xFF'
FOOTER = b'\xFE'
Servo_data = [0, 0, 0]

# 颜色定义（RGB565格式整数，适配平台要求）
COLOR_GREEN = 0x07E0     # 绿色（矩形框）
COLOR_BLUE = 0x001F      # 蓝色（中心点）


class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_limit=50, output_limit=80, deadzone=8.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.deadzone = deadzone

    def update(self, error):
        if abs(error) < self.deadzone:
            error = 0
        proportional = self.Kp * error
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        integral_term = self.Ki * self.integral
        derivative = self.Kd * (error - self.last_error)
        output = proportional + integral_term + derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        self.last_error = error
        return output


def send_packet(data):
    # 确保数据都是有效的整数
    cleaned_data = [int(d) for d in data]
    data_bytes = bytes(cleaned_data)
    packet = HEADER + data_bytes + FOOTER
    uart.write(packet)


def output_to_servo(output_x, output_y):
    servo_dx = math.atan2(output_x * 1.8/66, 15) * 180 / math.pi
    servo_dy = math.atan2(output_y * 1.8/66, 15) * 180 / math.pi
    servo_dy += LASER_Y_OFFSET
    servo_dx = max(-127.0, min(127.0, servo_dx))
    servo_dy = max(-127.0, min(127.0, servo_dy))
    return servo_dx + 128.0, servo_dy + 128.0


def calculate_distance(known_width, focal_length, pixel_width):
    if pixel_width <= 0:
        return 0
    return int((known_width * focal_length) / pixel_width)


def draw_detections(img, rects):
    """仅绘制检测到的矩形框和中心点"""
    for rect in rects:
        x1, y1, w, h = rect.rect()

        # 确保矩形尺寸有效
        if w <= 0 or h <= 0:
            continue

        # 绘制矩形边框
        img.draw_rectangle(x1, y1, w, h, color=COLOR_GREEN, thickness=2)

        # 计算中心坐标
        center_x = x1 + w // 2
        center_y = y1 + h // 2

        # 绘制中心标记（实心圆）
        img.draw_circle(center_x, center_y, 5, color=COLOR_BLUE, fill=True)


def detection():
    print("矩形检测开始")

    # 初始化传感器
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.QVGA)  # 320x240
    sensor.set_pixformat(Sensor.RGB565)  # RGB565格式

    # 初始化显示器
    Display.init(Display.ST7701, width=640, height=480, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 图像中心坐标
    img_center_x = sensor.width() // 2
    img_center_y = sensor.height() // 2

    # 初始化PID控制器
    pid_x = PIDController(Kp-0.2, Ki, Kd+0.2, deadzone=2.0)
    pid_y = PIDController(Kp-0.45, Ki, Kd-0.2, deadzone=1.0)

    try:
        while True:
            os.exitpoint()
            clock.tick()
            img = sensor.snapshot()

            # 检测矩形
            rects = img.find_rects(threshold=8000)

            # 过滤小矩形
            valid_rects = []
            for r in rects:
                x1, y1, w, h = r.rect()
                # 确保矩形尺寸有效
                if w > 0 and h > 0 and w * h >= MIN_RECT_AREA:
                    valid_rects.append(r)

            # 处理检测结果
            target_found = False
            if valid_rects:
                # 选最大矩形
                def get_rect_area(r):
                    x1, y1, w, h = r.rect()
                    return w * h
                target_rect = max(valid_rects, key=get_rect_area)
                x1, y1, w, h = target_rect.rect()

                # 计算误差
                target_center_x = x1 + w // 2
                target_center_y = y1 + h // 2
                error_x = img_center_x - target_center_x
                error_y = img_center_y - target_center_y

                # PID控制
                output_x = pid_x.update(error_x)
                output_y = pid_y.update(error_y)

                # 计算距离
                distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, w)
                distance = min(255, max(0, distance))

                # 发送舵机控制
                servo_dx, servo_dy = output_to_servo(output_x, output_y)
                Servo_data = [int(round(servo_dx)), int(round(servo_dy)), distance]
                send_packet(Servo_data)

                target_found = True

            # 目标丢失处理
            if not target_found:
                pid_x.integral = 0
                pid_y.integral = 0
                Servo_data = [128, 128, 0]
                send_packet(Servo_data)

            # 仅绘制检测到的矩形和中心点
            draw_detections(img, valid_rects)

            # 显示图像
            Display.show_image(img,
                              x=(640 - sensor.width())//2,
                              y=(480 - sensor.height())//2)

            gc.collect()

    except KeyboardInterrupt:
        print("用户中断")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        print("矩形检测结束")


if __name__ == "__main__":
    detection()
