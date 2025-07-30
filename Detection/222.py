import cv2
import numpy as np

# 参数设置（可根据实际场景调整）
params = {
    'blur_ksize': 5,          # 高斯模糊核大小
    'canny_low': 50,          # Canny低阈值
    'canny_high': 150,        # Canny高阈值
    'min_area': 500,          # 最小矩形面积（过滤小目标）
    'angle_tolerance': 15,    # 角度容差（与90°的偏差，单位：度）
    'length_tolerance': 0.2,  # 长度容差（相对误差，如0.2表示20%）
    'epsilon_ratio': 0.01,    # 多边形逼近精度（周长比例）
    'show_edges': True,       # 是否显示边缘图
    'show_contours': False,   # 是否显示轮廓图
}

def is_rectangle(contour, img_width, img_height):
    """四元检测法验证轮廓是否为矩形"""
    # 特征1：轮廓面积过滤（自适应最小面积）
    min_area = max(params['min_area'], img_width * img_height * 0.001)  # 至少占图像0.1%
    area = cv2.contourArea(contour)
    if area < min_area:
        return False, None
    
    # 特征2：多边形逼近，获取顶点（边数为4）
    perimeter = cv2.arcLength(contour, True)
    epsilon = max(2, params['epsilon_ratio'] * perimeter)  # 确保epsilon≥2像素
    approx = cv2.approxPolyDP(contour, epsilon, True)
    if len(approx) != 4:  # 必须有4个顶点
        return False, None
    
    # 提取四个顶点
    corners = approx.reshape(4, 2).astype(np.float32)
    
    # 使用凸包排序顶点（更鲁棒的排序方法）
    hull = cv2.convexHull(corners, returnPoints=True)
    if len(hull) != 4:
        return False, None
    hull = hull.reshape(4, 2).astype(np.float32)
    
    # 按顺时针排序（基于质心）
    center = np.mean(hull, axis=0)
    ordered_corners = sorted(hull, key=lambda p: np.arctan2(p[1]-center[1], p[0]-center[0]))
    ordered_corners = np.array(ordered_corners, dtype=np.float32)
    (tl, tr, br, bl) = ordered_corners  # 左上、右上、右下、左下

    # 计算四条边的长度
    def distance(p1, p2):
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])** 2)
    
    top_len = distance(tl, tr)
    right_len = distance(tr, br)
    bottom_len = distance(br, bl)
    left_len = distance(bl, tl)
    
    # 特征3：对边长度近似相等
    if not (abs(top_len - bottom_len) / max(top_len, bottom_len) < params['length_tolerance'] and
            abs(right_len - left_len) / max(right_len, left_len) < params['length_tolerance']):
        return False, None
    
    # 计算四个角的角度（通过向量点积）
    def angle_between(v1, v2):
        """计算两个向量的夹角（度）"""
        dot = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0
        cos_theta = dot / (norm_v1 * norm_v2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 避免数值误差
        return np.degrees(np.arccos(cos_theta))
    
    # 计算四个角的角度
    angle_tl = angle_between(tr - tl, bl - tl)  # 左上角
    angle_tr = angle_between(tl - tr, br - tr)  # 右上角
    angle_br = angle_between(bl - br, tr - br)  # 右下角
    angle_bl = angle_between(br - bl, tl - bl)  # 左下角
    
    # 特征4：四个角均接近90度
    angles = [angle_tl, angle_tr, angle_br, angle_bl]
    if not all(abs(angle - 90) < params['angle_tolerance'] for angle in angles):
        return False, None
    
    # 特征5：两条对角线长度近似相等
    diag1 = distance(tl, br)  # 主对角线
    diag2 = distance(tr, bl)  # 副对角线
    if abs(diag1 - diag2) / max(diag1, diag2) > params['length_tolerance']:
        return False, None
    
    return True, ordered_corners

def detect_rectangles(frame):
    """检测图像中的矩形并绘制边框"""
    height, width = frame.shape[:2]
    
    # 预处理：去噪+边缘检测
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (params['blur_ksize'], params['blur_ksize']), 0)
    edges = cv2.Canny(blurred, params['canny_low'], params['canny_high'])
    
    # 提取轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    result = frame.copy()
    contours_img = frame.copy()
    
    # 绘制所有轮廓（用于调试）
    cv2.drawContours(contours_img, contours, -1, (0, 255, 0), 1)
    
    for contour in contours:
        # 四元检测法验证矩形
        is_rect, corners = is_rectangle(contour, width, height)
        if is_rect and corners is not None:
            # 绘制矩形边框
            corners = corners.astype(np.int32)
            for i in range(4):
                cv2.line(result, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
            
            # 计算并标注矩形中心
            center = np.mean(corners, axis=0).astype(np.int32)
            cv2.circle(result, tuple(center), 5, (0, 0, 255), -1)
            cv2.putText(result, f"Rect: ({center[0]},{center[1]})", 
                       (center[0]-50, center[1]-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # 标注四个角点
            labels = ['TL', 'TR', 'BR', 'BL']
            for i, (x, y) in enumerate(corners):
                cv2.putText(result, labels[i], (x+5, y+5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    # 创建显示窗口
    if params['show_contours']:
        cv2.imshow("Contours", contours_img)
    if params['show_edges']:
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        return edges_bgr, result
    else:
        return result, result

def main():
    # 打开摄像头（0为默认摄像头）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("按 'q' 键退出")
    print("按 'e' 切换边缘图显示")
    print("按 'c' 切换轮廓图显示")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取图像帧")
            break
        
        # 检测矩形并绘制
        edges, result = detect_rectangles(frame)
        
        # 显示结果
        if params['show_edges']:
            combined = np.hstack((edges, result))
            cv2.putText(combined, "Edges", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined, "Detection Result", (frame.shape[1]+10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Rectangle Detection", combined)
        else:
            cv2.imshow("Rectangle Detection", result)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 退出
            break
        elif key == ord('e'):  # 切换边缘图显示
            params['show_edges'] = not params['show_edges']
        elif key == ord('c'):  # 切换轮廓图显示
            params['show_contours'] = not params['show_contours']
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()