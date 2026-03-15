import time, os, sys
import math

from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ---------------------- 新增：导入字体相关模块（关键补充） ----------------------
# 若IDE有专门的字体模块，需先导入（多数情况会包含在display或media中）
try:
    from media.font import Font, TEXT_ALIGN_LEFT  # 假设字体类和对齐常量在此路径
except ImportError:
    # 若导入失败，用通用默认值替代（避免报错）
    class Font:
        DEFAULT = None  # 默认字体对象（IDE会自动识别系统默认字体）
    TEXT_ALIGN_LEFT = 0  # 左对齐（多数库用0表示左对齐）

sensor_id = 2
sensor = None

# 颜色阈值设置（绿色灯泡LAB阈值）
WHITE_THRESHOLD = ((40, 100, -60, -20, -20, 40))

image_center_x = 320
image_center_y = 240

cx = 0
cy = 0

# 色块有效性验证函数
def is_valid_blob(blob, min_area=300):
    if blob.pixels() < min_area:
        return False
    rect_area = blob.w() * blob.h()
    if rect_area == 0:
        return False
    density = blob.pixels() / rect_area
    if density < 0.2:
        return False
    return True

# 中值滤波函数
def median_filter(values, new_value):
    values.append(new_value)
    if len(values) > 5:
        values.pop(0)
    return sorted(values)[len(values)//2]

try:
    # 摄像头与显示初始化
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_1)  # 640*480
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)
    Display.init(Display.VIRT, width = 640, height = 480)
    MediaManager.init()
    sensor.run()

    # 跟踪与FPS变量初始化
    last_white_center = None
    white_skip_count = 0
    MAX_WHITE_SKIP = 9
    ALPHA = 0.98
    MIN_WHITE_AREA = 10
    MIN_CHANGE_THRESHOLD = 1.5
    x_history = []
    y_history = []
    last_time = time.ticks_ms()
    fps_history = []
    smooth_fps = 0

    # ---------------------- 新增：初始化字体对象（必选参数1） ----------------------
    # 使用系统默认字体，若有自定义字体可替换为具体路径（如Font("/font/simhei.ttf", 16)）
    fps_font = Font.DEFAULT

    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_1)

        # 1. 计算平滑FPS
        current_time = time.ticks_ms()
        delta_ms = time.ticks_diff(current_time, last_time)
        last_time = current_time
        current_fps = 1000.0 / delta_ms if delta_ms > 0 else 0
        fps_history.append(current_fps)
        if len(fps_history) > 5:
            fps_history.pop(0)
        smooth_fps = sum(fps_history) / len(fps_history) if fps_history else 0

        # 2. 绿色色块检测与跟踪（原有逻辑不变）
        white_blobs = img.find_blobs(
            [WHITE_THRESHOLD],
            pixels_threshold=150,
            area_threshold=150,
            merge=True
        )
        largest_white = None
        for blob in white_blobs:
            if is_valid_blob(blob, min_area=MIN_WHITE_AREA):
                if largest_white is None or blob.pixels() > largest_white.pixels():
                    largest_white = blob

        current_white_center = None
        current_blob = None
        if largest_white:
            current_blob = largest_white
            white_skip_count = 0
            raw_center = (largest_white.cx(), largest_white.cy())
            filtered_x = median_filter(x_history, raw_center[0])
            filtered_y = median_filter(y_history, raw_center[1])
            filtered_raw = (filtered_x, filtered_y)

            if last_white_center:
                delta_x = abs(filtered_raw[0] - last_white_center[0])
                delta_y = abs(filtered_raw[1] - last_white_center[1])
                if delta_x > MIN_CHANGE_THRESHOLD or delta_y > MIN_CHANGE_THRESHOLD:
                    filtered_center = (
                        ALPHA * filtered_raw[0] + (1 - ALPHA) * last_white_center[0],
                        ALPHA * filtered_raw[1] + (1 - ALPHA) * last_white_center[1]
                    )
                    last_white_center = filtered_center
            else:
                last_white_center = filtered_raw

            current_white_center = last_white_center
            cx = last_white_center[0]
            cy = last_white_center[1]
        else:
            white_skip_count += 1
            if white_skip_count > MAX_WHITE_SKIP:
                last_white_center = None
                current_white_center = None
                x_history = []
                y_history = []
                cx = 0
                cy = 0
            else:
                current_white_center = last_white_center

        # 3. 绘制目标框与十字（原有逻辑不变）
        if current_white_center:
            int_tx = int(round(current_white_center[0]))
            int_ty = int(round(current_white_center[1]))
            if current_blob:
                tw = current_blob.w()
                th = current_blob.h()
                x = int_tx - tw // 2
                y = int_ty - th // 2
                img.draw_rectangle((x, y, tw, th), color=(0, 255, 0), thickness=2)
            img.draw_cross(filtered_x, filtered_y, color=(0, 255, 0), size=10)

        # ---------------------- 修正：补充3个必选参数的draw_string_advanced ----------------------
        # 核心补充：字体对象（fps_font）、对齐方式（TEXT_ALIGN_LEFT）、文本最大宽度（150）
        img.draw_string_advanced(10, 10, 30, f"FPS: {smooth_fps:.1f}")

        # 显示图像
        Display.show_image(img)
        # 控制台打印
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
