import time, os, sys
import math
import my_text

from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *

sensor_id = 2
sensor = None

# 颜色阈值设置
#WHITE_THRESHOLD = ((50, 100, -80, -30, -50, 50)) #亮绿色
#WHITE_THRESHOLD = ((75, 100, -53, -8, -21, 9))
WHITE_THRESHOLD = ((40, 100, -60, -20, -20, 40))

image_center_x = 320
image_center_y = 240

cx = 0
cy = 0

# 色块有效性验证函数
def is_valid_blob(blob, min_area=300):
    """验证色块是否有效（过滤小噪点和低密度色块）"""
    # 面积校验
    if blob.pixels() < min_area:
        return False
    # 密度校验：排除分散的、非实心的色块
    rect_area = blob.w() * blob.h()
    if rect_area == 0:
        return False
    density = blob.pixels() / rect_area
    if density < 0.2:  # 密度阈值可根据实际场景调整
        return False
    return True

# 中值滤波函数
def median_filter(values, new_value):
    """对一组值应用中值滤波，保持列表长度并返回中值"""
    values.append(new_value)
    if len(values) > 5:  # 保持5个最近的值用于中值计算
        values.pop(0)
    return sorted(values)[len(values)//2]  # 返回中值

try:
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id)
    # 重置摄像头sensor
    sensor.reset()

    # 无需进行镜像翻转
    sensor.set_vflip(False)

    # 设置通道1的输出尺寸（VGA）和像素格式
    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    # 初始化显示（ST7701屏幕，640*480分辨率）
    Display.init(Display.VIRT, width = 640, height = 480)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动传感器
    sensor.run()

    # ---------------------- 跟踪维持机制核心变量 ----------------------
    last_white_center = None         # 上一次有效白色色块中心坐标 (cx, cy)
    white_skip_count = 0             # 白色色块丢失计数器
    MAX_WHITE_SKIP = 9               # 最大允许丢失帧数
    ALPHA = 0.98                     # 增大滤波系数，使历史数据权重更高
    MIN_WHITE_AREA = 10             # 白色色块最小有效面积
    MIN_CHANGE_THRESHOLD = 1.5         # 最小变化阈值，忽略微小波动

    # 中值滤波历史数据
    x_history = []
    y_history = []

    # ---------------------- FPS计算相关变量 ----------------------
    last_time = time.ticks_ms()  # 上一帧的时间（毫秒）
    fps_history = []  # 存储最近的帧率，用于平滑
    smooth_fps = 0    # 平滑后的帧率

    while True:
        os.exitpoint()

        # 捕获图像
        img = sensor.snapshot(chn=CAM_CHN_ID_1)

        # ---------------------- 计算FPS ----------------------
        current_time = time.ticks_ms()  # 当前帧时间（毫秒）
        # 计算与上一帧的时间差（处理时间溢出问题）
        delta_ms = time.ticks_diff(current_time, last_time)
        last_time = current_time  # 更新上一帧时间

        # 计算瞬时帧率（避免除零）
        if delta_ms > 0:
            current_fps = 1000.0 / delta_ms  # 转换为每秒帧数
        else:
            current_fps = 0

        # 平滑处理：保留最近5帧的帧率，取平均值
        fps_history.append(current_fps)
        if len(fps_history) > 5:
            fps_history.pop(0)
        # 计算平均帧率（平滑后）
        smooth_fps = sum(fps_history) / len(fps_history) if fps_history else 0

        # 1. 查找并筛选有效白色色块
        white_blobs = img.find_blobs(
            [WHITE_THRESHOLD],
            pixels_threshold=150,
            area_threshold=150,
            merge=True  # 合并相邻色块，减少误检
        )
        # 选择最大的有效白色色块
        largest_white = None
        for blob in white_blobs:
            if is_valid_blob(blob, min_area=MIN_WHITE_AREA):
                if largest_white is None or blob.pixels() > largest_white.pixels():
                    largest_white = blob

        # ---------------------- 2. 白色色块跟踪维持逻辑 ----------------------
        current_white_center = None  # 当前帧有效中心坐标
        current_blob = None  # 保存当前有效色块用于绘制

        if largest_white:
            # 保存当前有效色块
            current_blob = largest_white

            # 2.1 检测到有效白色色块：重置丢失计数器，更新滤波中心
            white_skip_count = 0  # 检测到目标，丢失计数清零
            raw_center = (largest_white.cx(), largest_white.cy())  # 当前色块原始中心

            # 应用中值滤波减少突变噪声
            filtered_x = median_filter(x_history, raw_center[0])
            filtered_y = median_filter(y_history, raw_center[1])
            filtered_raw = (filtered_x, filtered_y)

            # 应用移动平均滤波（平滑中心坐标，减少抖动）
            if last_white_center:
                # 检查变化是否超过阈值，只有明显变化才更新
                delta_x = abs(filtered_raw[0] - last_white_center[0])
                delta_y = abs(filtered_raw[1] - last_white_center[1])

                if delta_x > MIN_CHANGE_THRESHOLD or delta_y > MIN_CHANGE_THRESHOLD:
                    # 对x、y坐标分别滤波，避免突变
                    filtered_center = (
                        ALPHA * filtered_raw[0] + (1 - ALPHA) * last_white_center[0],
                        ALPHA * filtered_raw[1] + (1 - ALPHA) * last_white_center[1]
                    )
                    last_white_center = filtered_center
                # 如果变化太小则保持上次值
            else:
                # 首次检测到目标，直接用滤波后的中心初始化
                last_white_center = filtered_raw

            current_white_center = last_white_center  # 标记当前有效中心
            cx = last_white_center[0]
            cy = last_white_center[1]

        else:
            # 2.2 未检测到有效白色色块：判断是否维持跟踪
            white_skip_count += 1  # 丢失计数+1
            if white_skip_count <= MAX_WHITE_SKIP and last_white_center:
                # 未超过最大丢失帧数，沿用历史中心坐标（维持跟踪）
                current_white_center = last_white_center
            else:
                # 超过最大丢失帧数，重置跟踪状态（放弃跟踪）
                last_white_center = None
                current_white_center = None
                # 重置历史数据
                x_history = []
                y_history = []
                cx = 0
                cy = 0
         # ---------------------- 3. 绘制目标框和相关信息 ----------------------
         # 绘制经过滤波和跟踪的中心（蓝色十字，代表实际跟踪值）
        if current_white_center:  # 增加current_blob的判断，避免空指针错误
            # 转换为整数坐标用于绘制
            int_tx = int(round(current_white_center[0]))
            int_ty = int(round(current_white_center[1]))
            if current_blob:
                # 获取原始色块的宽高
                tw = current_blob.w()
                th = current_blob.h()
                 # 计算矩形左上角坐标
                x = int_tx - tw // 2
                y = int_ty - th // 2

                 # 关键修正：使用(x, y, 宽, 高)格式绘制矩形
                 # 而不是(x1, y1, x2, y2)格式
                img.draw_rectangle((x, y, tw, th), color=(0, 255, 0), thickness=2)
            img.draw_cross(filtered_x, filtered_y, color=(0, 255, 0), size=10)

        Display.show_image(img)
        # 打印调试信息（包含FPS）
        print(f"x={cx:.1f}, y={cy:.1f}, FPS={smooth_fps:.1f}")


except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"用户手动异常: {e}")
finally:
    # 资源释放
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
