import time, os, gc, sys, math

from media.sensor import *
from media.display import *
from media.media import *
from machine import PWM, FPIOA, UART
import socket
import network
import struct
from machine import SPI
import struct  # 导入struct模块进行字节流转换

DETECT_WIDTH = 640
DETECT_HEIGHT = 480

SERVO_ZERO = 0.025*100
SERVO_PI = 0.125*100
SERVO_MID = 0.075*100

MAX_PACKET_SIZE = 1400

# 希望过滤的最小色块
FLITER_MINI_PART = 0

last_calculate_target_x = 0
last_calculate_target_y = 0

# 绿色阈值 (L Min, L Max, A Min, A Max, B Min, B Max)，特别是亮度较高的绿色
thresholds = [(50, 100, -80, -30, -50, 50)]  # 高亮度的绿色阈值
thresholds_1 = [(40, 100, -70, -20, -60, 60)]
# 高亮度绿色阈值 (L Min, L Max, A Min, A Max, B Min, B Max)
green_thresholds = [(40, 100, -80, -20, 10, 80)]  # 绿色偏暖区间
#green_thresholds = [(0, 100, -20, 13, -28, 10)]  # 减光镜

sensor = None

imu_rec_data = b''
imu_raw_data= None

FRAME_LENGTH = 11

TARGET_IP = "192.168.43.180"
TARGET_PORT = 8080

target_addr = (TARGET_IP,TARGET_PORT)


def compress_image_pillow(image, quality=50):
    output = io.BytesIO()  # 用于保存压缩后的图像数据
    image.save(output, format="JPEG", quality=quality)
    return output.getvalue()


# 陀螺仪解包
def unpack_imu_data(data):
    """解包IMU数据"""
#    roll = ((data[2] << 8) | data[3]) / 32768.0 * 180.0
#    pitch = ((data[4] << 8) | data[5]) / 32768.0 * 180.0
#    yaw = ((data[6] << 8) | data[7]) / 32768.0 * 180.0
#    version = (data[8] << 8) | data[9]
#    checksum = data[10]

    # 解包数据
    roll = struct.unpack('<h', data[2:4])[0]  # 小端 16 位有符号整数
    pitch = struct.unpack('<h', data[4:6])[0]
    yaw = struct.unpack('<h', data[6:8])[0]

    # 转换为角度
    roll_angle = roll / 32768.0 * 180.0
    pitch_angle = pitch / 32768.0 * 180.0
    yaw_angle = yaw / 32768.0 * 180.0

    # 校验和计算
    checksum = data[10]
    calculated_sum = sum(data[:10]) & 0xFF
    if calculated_sum == checksum:
        return {
            "roll": roll_angle,
            "pitch": pitch_angle,
            "yaw": yaw_angle,
        }
    else:
        print("校验和错误")
        return None

def process_data(rec_data):
    """处理接收到的数据，找到并解析有效数据帧"""
    while len(rec_data) >= FRAME_LENGTH:  # 确保至少有一帧长度的数据
        # 查找帧头0x55
        header_index = rec_data.find(b'\x55')
        if header_index == -1:  # 没有找到帧头，清空缓冲区
            rec_data = b''
            break

        # 检查第二个字节是否是0x53
        if rec_data[header_index + 1] != 0x53:
            # 如果不是，丢弃帧头本身，继续查找下一个帧头
            rec_data = rec_data[header_index + 1:]
            continue

        # 如果帧头后面剩余的字节不足一帧长度，等待更多数据
        if len(rec_data) - header_index < FRAME_LENGTH:
            rec_data = rec_data[header_index:]  # 保留帧头及其后续数据
            break

        # 截取一个完整数据帧
        frame = rec_data[header_index:header_index + FRAME_LENGTH]

        # 校验和
        imu_data = unpack_imu_data(frame)
        if imu_data:
            #print(f"解包数据: {imu_data}")
            imu_raw_data = imu_data

        # 清除已经处理的数据帧
        rec_data = rec_data[header_index + FRAME_LENGTH:]

    return rec_data

# 初始化wlan
def network_use_wlan(is_wlan=True):
    if is_wlan:
        sta=network.WLAN(0)
        sta.connect("honor","8888888866")
        print(sta.status())
        while sta.ifconfig()[0] == '0.0.0.0':
            os.exitpoint()
        print(sta.ifconfig())
        ip = sta.ifconfig()[0]
        return ip
    else:
        a=network.LAN()
        if not a.active():
            raise RuntimeError("LAN interface is not active.")
        a.ifconfig("dhcp")
        print(a.ifconfig())
        ip = a.ifconfig()[0]
        return ip

try:
#    # udp 初始化
#    #获取lan接口
#    ip = network_use_wlan(True)
#    #获取地址和端口号 对应地址
#    ai = socket.getaddrinfo(ip, 8080)
#    #ai = socket.getaddrinfo("10.10.1.94", 60000)
#    print("Address infos:", ai)
#    addr = ai[0][-1]

#    print("Connect address:", addr)
#    #建立socket
#    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 创建FPIOA对象，用于初始化引脚功能配置
    fpioa = FPIOA()
    fpioa.set_function(42, FPIOA.PWM0)
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)

    # 初始化 SPI，时钟速率 5 MHz，极性 0，相位 0，数据位宽 8 位
    spi=SPI(1,baudrate=5000000, polarity=0, phase=0, bits=8) # spi init clock 5MHz, polarity 0, phase 0, data bitwide 8bits

    # 初始化UART2，波特率9600，8位数据位，无校验，1位停止位
    uart = UART(UART.UART2, baudrate=9600, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    # 初始化PWM参数
    # channel: PWM 通道号，取值范围为 [0, 5]
    # freq: PWM 通道输出频率。单位为Hz。
    # duty: PWM 通道输出占空比，表示高电平在整个周期中的百分比，取值范围为 [0, 100]，支持小数点。可选参数，默认值为 50。
    # enable: PWM 通道输出是否立即使能，可选参数，默认值为 False。
    pwm = PWM(0, 50, 50, enable=True)  # 默认频率2kHz,占空比50%

    # 构造传感器对象
    sensor = Sensor(width=DETECT_WIDTH, height=DETECT_HEIGHT,fps=90)
    sensor.reset()
    sensor.set_framesize(width=DETECT_WIDTH, height=DETECT_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)

    # 使用虚拟显示器进行输出
    Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT, fps=100)

    # 初始化媒体管理器
    MediaManager.init()
    sensor.run()

    fps = time.clock()

    # 坐标
    target_position_x = 0
    target_position_y = 0
    duty = 50

    while True:
        fps.tick()

        # 捕捉图像
        img = sensor.snapshot()

        max_area = 0
        max_blob = None

        # 查找符合绿色阈值的颜色块
        for blob in img.find_blobs(green_thresholds, pixels_threshold=10, area_threshold=10, merge=True):
            if blob.code() == 1:  # 只处理绿色块
                # 计算当前色块的面积
                area = blob.area()
                if area < FLITER_MINI_PART:
                    continue  # 忽略过小的色块
                # 如果当前色块是最大且满足圆形或椭圆形要求
                if area > max_area:
#                    elongation = blob.elongation()
#                    print(elongation)
#                    # 检查是否接近圆形（elongation 值接近 1）
#                    if 0.2 < elongation :  # 可以调整这个值来过滤不规则形状
                        max_area = area
                        max_blob = blob

        # 如果找到了最大且符合条件的绿色色块
        if max_blob is not None:
            # 绘制矩形框
            img.draw_rectangle([v for v in max_blob.rect()])
            # 绘制十字标记
            img.draw_cross(max_blob.cx(), max_blob.cy())
            # 绘制关键点（旋转角度）
            img.draw_keypoints([(max_blob.cx(), max_blob.cy(), int(math.degrees(max_blob.rotation())))], size=20)

            # 得到目标位置
            target_position_x = max_blob.cx()
            target_position_y = max_blob.cy()

            # 设置尾翼角度
#            duty = (target_position_x - DETECT_WIDTH/2)/DETECT_WIDTH*(SERVO_PI-SERVO_ZERO)/2 + SERVO_MID;
#            pwm.duty(duty)

            # 将浮点数转换为字节流（使用小端字节序，可根据接收端需求调整）
            byte_data = struct.pack('<f', target_position_x)
            spi.write(bytes([0x66]) + byte_data + bytes([0x99]))
            print(target_position_x,",",target_position_y)

        # 显示处理后的图像
        Display.show_image(img)
        gc.collect()
#        # udp 发送视频流
#        # 将图像压缩为 JPEG 格式
#        img_jpeg = img.compress(quality=50)  # 压缩质量（可调整 0-100）
#        # 获取压缩后的 JPEG 字节流数据
#        jpeg_data = bytes(img_jpeg)
#        # 分片发送数据
#        HEADER_SIZE = 2  # 标头占用2字节
#        for i in range(0, len(jpeg_data), MAX_PACKET_SIZE - HEADER_SIZE):
#            chunk = jpeg_data[i:i + (MAX_PACKET_SIZE - HEADER_SIZE)]
#            # 标头包含：分片序号（1字节）和是否为最后一个分片（1字节）
#            header = bytes([i // (MAX_PACKET_SIZE - HEADER_SIZE), int(i + len(chunk) >= len(jpeg_data))])
#            sock.sendto(header + chunk, target_addr)
#        print(fps.fps())  # 打印帧率
        #data = None

        #如果接收不到数据就一直尝试读取
#        while data == None:
            # 读取数据
        temp_data = uart.read()  # 尝试读取数据
        if temp_data:
            imu_rec_data += temp_data  # 将新接收的数据添加到缓冲区
            # 处理接收缓冲区的数据
            imu_rec_data = process_data(imu_rec_data)
            #通过CanMV IDE K230中的串行终端控制台打印出来
#            print("Received:", temp_data,"fps:",fps.fps())

except KeyboardInterrupt as e:
    print(f"user stop")
except BaseException as e:
    print(f"Exception '{e}'")
finally:
    # 停止传感器
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 关闭显示器
    Display.deinit()

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

    # 释放媒体管理器
    MediaManager.deinit()

    # 释放资源
    #sock.close()
    spi.deinit()
