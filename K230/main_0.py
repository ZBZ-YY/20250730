import os
import ujson
import aicube
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.sensor import *
from media.display import *
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import gc
import math
from machine import UART
from machine import FPIOA
from machine import PWM

# 距离测算参数
KNOWN_WIDTH = 25.0       # 已知物体的实际宽度(cm)，需根据实际物体修改
FOCAL_LENGTH = 550.0    # 相机焦距(像素)，需通过标定获得
MIN_DETECTION_AREA = 500 # 最小矩形面积，过滤小噪点

DEADZONE_MOTION = 1.5  # 运动死区（度）
DEADZONE_ERROR = 8.0   # 误差死区（像素）

LASER_Y_OFFSET = -0.6  # 激光偏置角度

# 初始化UART通信
fpioa = FPIOA()
fpioa.set_function(9, fpioa.UART1_TXD, ie=0, oe=1)
fpioa.set_function(10, fpioa.UART1_RXD, ie=1, oe=0)
uart = UART(UART.UART1, baudrate=115200)

# PID parameters
Kp = 0.8  # Proportional gain
Ki = 0.001 # Integral gain
Kd = 0.6  # Derivative gain

Servo_data = [0,0,0];

HEADER = b'\xFF'  # 包头
FOOTER = b'\xFE'  # 包尾

def send_packet(data):
    # 将数据转换为字节数组
    data_bytes = bytes(data)
    # 拼接数据包：包头 + 数据 + 包尾
    packet = HEADER + data_bytes + FOOTER
    # 发送数据包
    uart.write(packet)

def output_to_servo(output_x, output_y):
    # 计算舵机角度
    Servo_dx = math.atan2(output_x * 1.8/66, 15) * 180 / math.pi
    Servo_dy = math.atan2(output_y * 1.8/66, 15) * 180 / math.pi

    # 应用固定补偿
    Servo_dy += LASER_Y_OFFSET

    # 限幅处理
    Servo_dx = max(-127.0, min(127.0, Servo_dx))
    Servo_dy = max(-127.0, min(127.0, Servo_dy))

    # 编码为0-255范围
    return Servo_dx + 128.0, Servo_dy + 128.0

class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_limit=50, output_limit=80,deadzone=8.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.deadzone = deadzone

    def update(self, error):
        # 应用死区处理
        if abs(error) < self.deadzone:
            error = 0
        # Proportional term
        proportional = self.Kp * error

        # Integral term
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        integral_term = self.Ki * self.integral

        # Derivative term
        derivative = self.Kd * (error - self.last_error)

        # PID output
        output = proportional + integral_term + derivative
        output = max(-self.output_limit, min(self.output_limit, output))

        # Update last error
        self.last_error = error

        return output

# 距离计算函数
def calculate_distance(known_width, focal_length, pixel_width):
    """计算物体到相机的距离"""
    if pixel_width <= 0:
        return 0
    return int((known_width * focal_length) / pixel_width)

def draw_fps(img, fps, distance=None):
    """在图像上绘制FPS和距离"""
    img.draw_string(10, 10, f"FPS: {fps:.1f}", color=(255, 0, 0), scale=2)
    if distance:
        img.draw_string(10, 40, f"Distance: {distance:.1f}cm", color=(0, 255, 0), scale=2)


display_mode="lcd"
if display_mode=="lcd":
    DISPLAY_WIDTH = ALIGN_UP(640, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1080, 16)
OUT_RGB888P_HEIGH = 720

# 颜色盘
color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
        (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
        (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0)]

# 矩形检测参数
rect_params = {
    'blur_ksize': 5,          # 高斯模糊核大小
    'canny_low': 50,          # Canny低阈值
    'canny_high': 150,        # Canny高阈值
    'min_area': 500,          # 最小矩形面积
    'angle_tolerance': 15,    # 角度容差（与90°的偏差）
    'length_tolerance': 0.2,  # 长度容差（相对误差）
    'epsilon_ratio': 0.01,    # 多边形逼近精度
}

def distance(p1, p2):
    """计算两点间距离"""
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def angle_between(v1, v2):
    """计算两个向量的夹角（度）"""
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    norm_v1 = math.hypot(v1[0], v1[1])
    norm_v2 = math.hypot(v2[0], v2[1])
    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0
    cos_theta = dot / (norm_v1 * norm_v2)
    cos_theta = max(-1.0, min(1.0, cos_theta))  # 数值稳定处理
    return math.degrees(math.acos(cos_theta))

def is_rectangle(contour, img_width, img_height):
    """判断轮廓是否为矩形"""
    # 1. 面积过滤
    area = contour.area()
    min_area = max(rect_params['min_area'], img_width * img_height * 0.001)
    if area < min_area:
        return False, None

    # 2. 多边形逼近（获取顶点）
    perimeter = contour.perimeter()
    epsilon = max(2, rect_params['epsilon_ratio'] * perimeter)
    approx = contour.approx_poly(dp=epsilon, closed=True)
    if len(approx) != 4:  # 必须有4个顶点
        return False, None

    # 3. 提取并排序顶点
    corners = [(p[0], p[1]) for p in approx]

    # 4. 计算四条边长度
    (tl, tr, br, bl) = corners  # 假设已排序：左上、右上、右下、左下
    top_len = distance(tl, tr)
    right_len = distance(tr, br)
    bottom_len = distance(br, bl)
    left_len = distance(bl, tl)

    # 5. 对边长度验证
    if (abs(top_len - bottom_len) / max(top_len, bottom_len) > rect_params['length_tolerance'] or
        abs(right_len - left_len) / max(right_len, left_len) > rect_params['length_tolerance']):
        return False, None

    # 6. 角度验证（接近90度）
    angles = [
        angle_between((tr[0]-tl[0], tr[1]-tl[1]), (bl[0]-tl[0], bl[1]-tl[1])),  # 左上角
        angle_between((tl[0]-tr[0], tl[1]-tr[1]), (br[0]-tr[0], br[1]-tr[1])),  # 右上角
        angle_between((bl[0]-br[0], bl[1]-br[1]), (tr[0]-br[0], tr[1]-br[1])),  # 右下角
        angle_between((br[0]-bl[0], br[1]-bl[1]), (tl[0]-bl[0], tl[1]-bl[1]))   # 左下角
    ]
    if not all(abs(angle - 90) < rect_params['angle_tolerance'] for angle in angles):
        return False, None

    # 7. 对角线验证
    diag1 = distance(tl, br)
    diag2 = distance(tr, bl)
    if abs(diag1 - diag2) / max(diag1, diag2) > rect_params['length_tolerance']:
        return False, None

    return True, corners

def detect_rectangles(img):
    """检测图像中的矩形"""
    width, height = img.width(), img.height()

    # 1. 预处理：转为灰度图
    gray = img.to_grayscale()

    # 2. 高斯模糊去噪
    blurred = gray.gaussian_blur(rect_params['blur_ksize'])

    # 3. Canny边缘检测
    edges = blurred.canny(rect_params['canny_low'], rect_params['canny_high'])

    # 4. 提取轮廓
    contours = edges.find_contours()
    if not contours:
        return [], edges

    # 5. 筛选矩形轮廓
    rectangles = []
    for contour in contours:
        is_rect, corners = is_rectangle(contour, width, height)
        if is_rect and corners:
            # 计算矩形中心和宽度
            center_x = sum(p[0] for p in corners) / 4
            center_y = sum(p[1] for p in corners) / 4
            rect_width = (distance(corners[0], corners[1]) + distance(corners[2], corners[3])) / 2  # 平均宽度
            rectangles.append({
                'center': (center_x, center_y),
                'width': rect_width,
                'corners': corners
            })

    return rectangles, edges

class ScopedTiming:
    def __init__(self, info="", enable_profile=True):
        self.info = info
        self.enable_profile = enable_profile

    def __enter__(self):
        if self.enable_profile:
            self.start_time = time.time_ns()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enable_profile:
            elapsed_time = time.time_ns() - self.start_time
            print(f"{self.info} took {elapsed_time / 1000000:.2f} ms")

def detection():
    last_distance = 0
    print("rectangle detection start")

    # 初始化显示和传感器
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    algo_center_x = OUT_RGB888P_WIDTH // 2  # 图像中心X
    algo_center_y = OUT_RGB888P_HEIGH // 2  # 图像中心Y

    # 初始化传感器
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    # 通道0用于显示
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # 通道2用于检测
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定显示
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(** sensor_bind_info, layer=Display.LAYER_VIDEO1)
    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=True)
    else:
        Display.init(Display.LT9611, to_ide=True)

    # 创建OSD图像
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    # 初始化PID控制器
    pid_x = PIDController(Kp-0.2, Ki, Kd+0.2, integral_limit=50, output_limit=80, deadzone=2.0)
    pid_y = PIDController(Kp-0.45, Ki, Kd-0.2, integral_limit=50, output_limit=80, deadzone=1.0)

    try:
        MediaManager.init()
        sensor.run()
        rgb888p_img = None
        fps_counter = 0
        fps_timer = time.time_ns()

        while True:
            distance = 0
            with ScopedTiming("total", 1):
                # 获取图像
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                if not rgb888p_img:
                    continue

                # 检测矩形
                rectangles, edges = detect_rectangles(rgb888p_img)

                # 计算FPS
                fps_counter += 1
                current_time = time.time_ns()
                if current_time - fps_timer >= 1e9:  # 每秒计算一次
                    fps = fps_counter / ((current_time - fps_timer) / 1e9)
                    fps_timer = current_time
                    fps_counter = 0
                else:
                    fps = 0

                osd_img.clear()
                target_found = False

                # 处理检测到的矩形
                if rectangles:
                    # 取最大的矩形作为目标
                    largest_rect = max(rectangles, key=lambda r: r['width'])
                    center_x, center_y = largest_rect['center']
                    rect_width = largest_rect['width']
                    corners = largest_rect['corners']

                    # 计算误差
                    error_x = algo_center_x - center_x
                    error_y = algo_center_y - center_y

                    # PID控制
                    output_x = pid_x.update(error_x)
                    output_y = pid_y.update(error_y)

                    # 计算距离
                    distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, rect_width)
                    distance = min(255, max(0, int(distance)))
                    last_distance = distance

                    # 发送舵机数据
                    servo_dx, servo_dy = output_to_servo(output_x, output_y)
                    Servo_data = [
                        int(round(servo_dx)),
                        int(round(servo_dy)),
                        distance
                    ]
                    send_packet(Servo_data)

                    # 绘制矩形
                    for i in range(4):
                        x1, y1 = corners[i]
                        x2, y2 = corners[(i+1)%4]
                        # 坐标转换到显示分辨率
                        disp_x1 = int(x1 * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                        disp_y1 = int(y1 * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)
                        disp_x2 = int(x2 * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                        disp_y2 = int(y2 * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)
                        osd_img.draw_line(disp_x1, disp_y1, disp_x2, disp_y2, color=(0, 255, 0), thickness=2)

                    # 绘制中心和距离
                    disp_cx = int(center_x * DISPLAY_WIDTH / OUT_RGB888P_WIDTH)
                    disp_cy = int(center_y * DISPLAY_HEIGHT / OUT_RGB888P_HEIGH)
                    osd_img.draw_circle(disp_cx, disp_cy, 5, color=(0, 0, 255), fill=True)
                    osd_img.draw_string(disp_cx - 30, disp_cy - 20,
                                       f"D: {distance}cm", color=(255, 255, 0), scale=2)

                    target_found = True

                # 目标丢失处理
                if not target_found:
                    pid_x.integral = 0
                    pid_x.last_error = 0
                    pid_y.integral = 0
                    pid_y.last_error = 0
                    # 发送零输出
                    Servo_data = [128, 128, 0]  # 中间位置
                    send_packet(Servo_data)

                # 绘制FPS
                draw_fps(osd_img, fps, last_distance if distance == 0 else distance)

                # 显示OSD
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()

            rgb888p_img = None

    except Exception as e:
        print(f"Error: {e}")
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()

    print("detection end")
    return 0


if __name__ == "__main__":
    detection()
