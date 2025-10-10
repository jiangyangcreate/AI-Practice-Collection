import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

def extract_roi(image ,roi_start_ratio=0.5):
    """提取感兴趣区域（ROI）
    
    Args:
        image: 输入的BGR图像
        roi_start_ratio: 感兴趣区域。（0表示整张都感兴趣、0.5表示下半部分感兴趣）
        
    Returns:
        roi_image: ROI区域图像
    """
    # 获取图像尺寸
    height, width = image.shape[:2]
    
    # 计算ROI起始行（从图像中部开始，关注前方地面）
    roi_start = int(height * roi_start_ratio)
    
    # 提取ROI区域（图像下半部分）
    roi_image = image[roi_start:height, 0:width]
    
    return roi_image

def preprocess_and_binarize(bgr_image, method ,invert):
    """完整的预处理和二值化流程

    otsu方法会自动计算最佳阈值，适合双峰分布的图像（如黑色车道+灰色地面）

    adaptive自适应均值法适应局部光照变化，适合有阴影和光照不均的场景

    invert: 是否反向二值化（True=低于阈值变白，False=高于阈值变白）
    
    Args:
        bgr_image: 输入的BGR彩色图像
        method: 二值化方法，可选 'mean', 'adaptive'
        **kwargs: 各方法的额外参数
            - invert: 是否反向二值化
            - block_size: 自适应方法的邻域大小
            - C: 自适应方法的常数偏移
        
    Returns:
        binary_image: 二值化结果
    """
    # 灰度化
    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

    
    if method == 'adaptive':
        # 选择二值化类型
        thresh_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
        
        # 自适应均值法：每个像素的阈值由其邻域的均值决定
        binary_image = cv2.adaptiveThreshold(
            gray_image,
            255,                            # 最大值
            cv2.ADAPTIVE_THRESH_MEAN_C,     # 使用邻域均值
            thresh_type,                    # 二值化类型（正向或反向）
            11,                             # 邻域大小（必须是奇数），越大对光照变化越不敏感
            2                               # 常数偏移从均值中减去的常数，用于微调阈值
        )
    elif method == 'mean':
        thresh_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
        _, binary_image = cv2.threshold(gray_image, gray_image.mean(), 255, thresh_type)
    else:
        raise ValueError(f"未知的二值化方法: {method}")
    
    return binary_image

def denoise_binary_image(binary_image, kernel_size=(5, 5)):
    """使用形态学开运算去除二值图像中的噪声
    
    Args:
        binary_image: 输入的二值化图像（0或255）
        kernel_size: 结构元素大小，默认为(5, 5)
        
    Returns:
        denoised_image: 去噪后的二值图像
    """
    # 创建结构元素（矩形核）
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
    
    # 开运算 = 先腐蚀后膨胀
    # 作用：去除小的白色噪点（孤立的亮点）
    denoised_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    
    return denoised_image

def visualize_preprocessing(roi_image, binary_image):
    """可视化预处理的各个阶段
    
    Args:
        roi_image: 感兴趣区域的原图（ROI）
        denoised_image: 去噪后的二值化图像
    """
    cv2.imshow('ROI', roi_image)
    cv2.imshow('ROI-II', binary_image)
    cv2.waitKey(1)


def find_lane_base_positions(binary_image, n_windows=9, edge_detection=True):
    """通过直方图找到左右车道线的起始位置
    
    Args:
        binary_image: 二值化图像（白色为车道线）
        n_windows: 滑动窗口数量
        edge_detection: 是否检测边缘模式（适合宽道路）
        
    Returns:
        left_base: 左车道线起始x坐标（None表示未检测到）
        right_base: 右车道线起始x坐标（None表示未检测到）
    """
    histogram = np.sum(binary_image[binary_image.shape[0]//2:, :], axis=0)
    midpoint = histogram.shape[0] // 2
    
    if edge_detection:
        threshold = np.max(histogram) * 0.3
        white_pixels = np.where(histogram > threshold)[0]
        
        if len(white_pixels) == 0:
            return None, None
        
        left_candidates = white_pixels[white_pixels < midpoint]
        right_candidates = white_pixels[white_pixels >= midpoint]
        
        left_base = np.min(left_candidates) if len(left_candidates) > 0 else None
        right_base = np.max(right_candidates) if len(right_candidates) > 0 else None
    else:
        left_half = histogram[:midpoint]
        right_half = histogram[midpoint:]
        
        left_base = np.argmax(left_half) if np.max(left_half) > 0 else None
        right_base = np.argmax(right_half) + midpoint if np.max(right_half) > 0 else None
    
    return left_base, right_base


def detect_road_edges(binary_image):
    """检测宽道路的左右边缘
    
    Args:
        binary_image: 二值化图像（白色为道路）
        
    Returns:
        left_edge_x: 左边缘x坐标数组
        left_edge_y: 左边缘y坐标数组
        right_edge_x: 右边缘x坐标数组
        right_edge_y: 右边缘y坐标数组
    """
    height, width = binary_image.shape
    left_edge_x, left_edge_y = [], []
    right_edge_x, right_edge_y = [], []
    
    for y in range(height):
        row = binary_image[y, :]
        white_pixels = np.where(row > 127)[0]
        
        if len(white_pixels) > 10:
            left_edge_x.append(white_pixels[0])
            left_edge_y.append(y)
            right_edge_x.append(white_pixels[-1])
            right_edge_y.append(y)
    
    return (np.array(left_edge_x), np.array(left_edge_y)), (np.array(right_edge_x), np.array(right_edge_y))


def sliding_window_detection(binary_image, n_windows=9, margin=80, min_pixels=50):
    """使用滑动窗口法检测车道线像素
    
    Args:
        binary_image: 二值化图像（白色为车道线，0或255）
        n_windows: 滑动窗口数量
        margin: 窗口宽度的一半（像素）
        min_pixels: 重新定位窗口所需的最小像素数
        
    Returns:
        left_lane_pixels: 左车道线像素坐标 (x_coords, y_coords)，未检测到则为 ([], [])
        right_lane_pixels: 右车道线像素坐标 (x_coords, y_coords)，未检测到则为 ([], [])
        window_info: 窗口信息列表，用于可视化 [(x_center, y_center, width, height), ...]
    """
    height, width = binary_image.shape
    window_height = height // n_windows
    
    nonzero = binary_image.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])
    
    left_base, right_base = find_lane_base_positions(binary_image, n_windows, edge_detection=True)
    
    left_current = left_base
    right_current = right_base
    
    left_lane_indices = []
    right_lane_indices = []
    window_info = []
    
    for window in range(n_windows):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        
        if left_current is not None:
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            window_info.append(('left', win_xleft_low, win_y_low, win_xleft_high, win_y_high))
            
            good_left_indices = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                                 (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            left_lane_indices.append(good_left_indices)
            
            if len(good_left_indices) > min_pixels:
                left_current = int(np.mean(nonzero_x[good_left_indices]))
        
        if right_current is not None:
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin
            window_info.append(('right', win_xright_low, win_y_low, win_xright_high, win_y_high))
            
            good_right_indices = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & 
                                  (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            right_lane_indices.append(good_right_indices)
            
            if len(good_right_indices) > min_pixels:
                right_current = int(np.mean(nonzero_x[good_right_indices]))
    
    left_lane_indices = np.concatenate(left_lane_indices) if left_lane_indices else np.array([])
    right_lane_indices = np.concatenate(right_lane_indices) if right_lane_indices else np.array([])
    
    left_x = nonzero_x[left_lane_indices] if len(left_lane_indices) > 0 else np.array([])
    left_y = nonzero_y[left_lane_indices] if len(left_lane_indices) > 0 else np.array([])
    right_x = nonzero_x[right_lane_indices] if len(right_lane_indices) > 0 else np.array([])
    right_y = nonzero_y[right_lane_indices] if len(right_lane_indices) > 0 else np.array([])
    
    return (left_x, left_y), (right_x, right_y), window_info


def fit_polynomial(lane_pixels, degree=2):
    """对车道线像素进行多项式拟合
    
    Args:
        lane_pixels: 车道线像素坐标 (x_coords, y_coords)
        degree: 多项式阶数，默认为2（二次多项式）
        
    Returns:
        poly_coeffs: 多项式系数 [a, b, c, ...]，使得 x = a*y^2 + b*y + c
                    如果拟合失败则返回 None
    """
    x_coords, y_coords = lane_pixels
    
    if len(x_coords) < degree + 1:
        return None
    
    poly_coeffs = np.polyfit(y_coords, x_coords, degree)
    return poly_coeffs


def calculate_lane_center_line(left_coeffs, right_coeffs, image_height):
    """根据左右车道线的多项式系数计算中心线
    
    Args:
        left_coeffs: 左车道线多项式系数 [a, b, c]
        right_coeffs: 右车道线多项式系数 [a, b, c]
        image_height: 图像高度
        
    Returns:
        center_coeffs: 中心线多项式系数 [a, b, c]
        center_points: 中心线采样点 [(x, y), ...]
    """
    if left_coeffs is None or right_coeffs is None:
        return None, []
    
    y_vals = np.linspace(0, image_height - 1, image_height)
    
    left_x = np.polyval(left_coeffs, y_vals)
    right_x = np.polyval(right_coeffs, y_vals)
    center_x = (left_x + right_x) / 2
    
    center_coeffs = np.polyfit(y_vals, center_x, 2)
    
    sample_points = 50
    y_samples = np.linspace(0, image_height - 1, sample_points, dtype=int)
    x_samples = np.polyval(center_coeffs, y_samples)
    center_points = list(zip(x_samples.astype(int), y_samples))
    
    return center_coeffs, center_points


def calculate_control_command(center_coeffs, image_width, image_height, 
                             lookahead_ratio=0.7, k_p=2.0,
                             v_straight=1.0, v_curve=0.5, curve_threshold=0.0005):
    """基于中心线多项式系数计算运动控制指令
    
    Args:
        center_coeffs: 中心线多项式系数 [a, b, c]，x = a*y^2 + b*y + c
        image_width: 图像宽度
        image_height: 图像高度
        lookahead_ratio: 前视点位置比例（0-1），越大越远
        k_p: 比例增益
        v_straight: 直道速度 (m/s)
        v_curve: 弯道速度 (m/s)
        curve_threshold: 曲率阈值，超过此值认为是弯道
        
    Returns:
        linear_velocity: 线速度 (m/s)
        angular_velocity: 角速度 (rad/s)
        lateral_error: 横向偏差（像素）
        curvature: 曲率
    """
    if center_coeffs is None:
        return 0.0, 0.0, 0.0, 0.0
    
    lookahead_y = int(image_height * lookahead_ratio)
    lookahead_x = np.polyval(center_coeffs, lookahead_y)
    
    image_center_x = image_width / 2
    lateral_error = lookahead_x - image_center_x
    normalized_error = lateral_error / (image_width / 2)
    
    a, b, c = center_coeffs
    curvature = abs(2 * a)
    
    if curvature > curve_threshold or abs(normalized_error) > 0.2:
        linear_velocity = v_curve
    else:
        linear_velocity = v_straight
    
    angular_velocity = -k_p * normalized_error
    angular_velocity = np.clip(angular_velocity, -2.0, 2.0)
    
    return linear_velocity, angular_velocity, lateral_error, curvature


def visualize_lane_detection(original_image, binary_image, left_pixels, right_pixels, 
                             left_coeffs, right_coeffs, center_coeffs, window_info,
                             lookahead_point=None, lateral_error=0.0, 
                             linear_vel=0.0, angular_vel=0.0):
    """可视化车道线检测结果
    
    Args:
        original_image: 原始彩色图像
        binary_image: 二值化图像
        left_pixels: 左车道线像素 (x_coords, y_coords)
        right_pixels: 右车道线像素 (x_coords, y_coords)
        left_coeffs: 左车道线多项式系数
        right_coeffs: 右车道线多项式系数
        center_coeffs: 中心线多项式系数
        window_info: 滑动窗口信息
        lookahead_point: 前视点坐标 (x, y)
        lateral_error: 横向偏差
        linear_vel: 线速度
        angular_vel: 角速度
        
    Returns:
        vis_image: 可视化图像
    """
    vis_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
    height = vis_image.shape[0]
    width = vis_image.shape[1]
    
    for window in window_info:
        lane_type, x_low, y_low, x_high, y_high = window
        color = (0, 255, 0) if lane_type == 'left' else (255, 0, 0)
        cv2.rectangle(vis_image, (x_low, y_low), (x_high, y_high), color, 2)
    
    left_x, left_y = left_pixels
    right_x, right_y = right_pixels
    
    if len(left_x) > 0:
        for x, y in zip(left_x, left_y):
            cv2.circle(vis_image, (int(x), int(y)), 2, (0, 255, 255), -1)
    
    if len(right_x) > 0:
        for x, y in zip(right_x, right_y):
            cv2.circle(vis_image, (int(x), int(y)), 2, (255, 255, 0), -1)
    
    y_vals = np.linspace(0, height - 1, height).astype(int)
    
    if left_coeffs is not None:
        left_fit_x = np.polyval(left_coeffs, y_vals).astype(int)
        for i in range(len(y_vals) - 1):
            cv2.line(vis_image, (left_fit_x[i], y_vals[i]), 
                    (left_fit_x[i+1], y_vals[i+1]), (0, 255, 0), 3)
    
    if right_coeffs is not None:
        right_fit_x = np.polyval(right_coeffs, y_vals).astype(int)
        for i in range(len(y_vals) - 1):
            cv2.line(vis_image, (right_fit_x[i], y_vals[i]), 
                    (right_fit_x[i+1], y_vals[i+1]), (255, 0, 0), 3)
    
    if center_coeffs is not None:
        center_fit_x = np.polyval(center_coeffs, y_vals).astype(int)
        for i in range(len(y_vals) - 1):
            cv2.line(vis_image, (center_fit_x[i], y_vals[i]), 
                    (center_fit_x[i+1], y_vals[i+1]), (0, 0, 255), 4)
    
    image_center_x = width // 2
    cv2.line(vis_image, (image_center_x, 0), (image_center_x, height), (255, 255, 255), 2)
    
    if lookahead_point is not None:
        lx, ly = lookahead_point
        cv2.circle(vis_image, (int(lx), int(ly)), 10, (255, 0, 255), -1)
        cv2.arrowedLine(vis_image, (image_center_x, int(ly)), (int(lx), int(ly)), 
                       (255, 0, 255), 3)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(vis_image, f'Error: {lateral_error:.1f}px', (10, 30), 
                font, 0.6, (255, 255, 255), 2)
    
    turn_info = "LEFT" if angular_vel > 0 else "RIGHT" if angular_vel < 0 else "STRAIGHT"
    cv2.putText(vis_image, f'Angular: {angular_vel:.3f} ({turn_info})', (10, 60), 
                font, 0.6, (255, 255, 255), 2)
    
    cv2.putText(vis_image, f'Speed: {linear_vel:.2f} m/s', (10, 90), 
                font, 0.6, (255, 255, 255), 2)
    
    return vis_image


class VisualMotionController(Node):
    """基于视觉的车道线跟随控制节点
    
    功能：
        - 接收摄像头图像
        - 滑动窗口法检测车道线
        - 多项式拟合车道线
        - 计算运动控制指令
        - 发布速度控制命令
    """
    
    def __init__(self):
        super().__init__('visual_motion_controller')
        
        # 订阅摄像头话题
        self.image_subscription = self.create_subscription(
            Image,
            '/camera',
            self.process_and_control,
            10
        )
        
        # 发布控制话题
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.bridge = CvBridge()
        
        # ========== 图像处理参数 ==========
        self.roi_start_ratio = 0.5
        
        # ========== 运动控制参数 ==========
        self.lookahead_ratio = 0.7
        self.k_p = 2.0
        self.v_straight = 1.0
        self.v_curve = 0.5
        self.curve_threshold = 0.0005
        
        # ========== 调试模式 ==========
        self.debug_mode = False
        
        
    def process_and_control(self, msg):
        """处理图像并计算运动控制指令（主回调函数）
        
        Args:
            msg: ROS2图像消息
        """
        try:
            # Step 1: 图像格式转换
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Step 2: 提取ROI
            roi_image = extract_roi(cv_image, self.roi_start_ratio)
            
            # Step 3: 图像预处理和二值化
            binary_image = preprocess_and_binarize(roi_image, method='mean', invert=True)
            
            # Step 4: 去噪
            denoised = denoise_binary_image(binary_image)

            # 可视化
            visualize_preprocessing(roi_image, denoised)
            
            # Step 5: 滑动窗口检测车道线
            left_pixels, right_pixels, windows = sliding_window_detection(denoised)
            
            # Step 6: 多项式拟合
            left_fit = fit_polynomial(left_pixels)
            right_fit = fit_polynomial(right_pixels)
            
            # Step 7: 计算中心线
            center_fit, center_pts = calculate_lane_center_line(
                left_fit, right_fit, denoised.shape[0]
            )
            
            # Step 8: 计算运动控制指令
            linear_vel, angular_vel, lateral_error, curvature = calculate_control_command(
                center_fit,
                denoised.shape[1],
                denoised.shape[0],
                lookahead_ratio=self.lookahead_ratio,
                k_p=self.k_p,
                v_straight=self.v_straight,
                v_curve=self.v_curve,
                curve_threshold=self.curve_threshold
            )
            
            # Step 9: 计算前视点用于可视化
            lookahead_point = None
            if center_fit is not None:
                lookahead_y = int(denoised.shape[0] * self.lookahead_ratio)
                lookahead_x = np.polyval(center_fit, lookahead_y)
                lookahead_point = (lookahead_x, lookahead_y)
            
            # Step 10: 可视化
            vis = visualize_lane_detection(
                roi_image, denoised,
                left_pixels, right_pixels,
                left_fit, right_fit, center_fit,
                windows,
                lookahead_point=lookahead_point,
                lateral_error=lateral_error,
                linear_vel=linear_vel,
                angular_vel=angular_vel
            )
            cv2.imshow('Lane Detection', vis)
            cv2.waitKey(1)
            
            # Step 11: 发布控制命令
            cmd = Twist()
            if self.debug_mode:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('调试模式：运动已暂停', throttle_duration_sec=2.0)
            else:
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel
            self.velocity_publisher.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建视觉运动控制节点
    controller = VisualMotionController()
    
    try:
        # 启动节点
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('节点被用户中断')
    finally:
        # 发送停止指令
        stop_cmd = Twist()
        controller.velocity_publisher.publish(stop_cmd)
        # 清理资源
        cv2.destroyAllWindows()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()