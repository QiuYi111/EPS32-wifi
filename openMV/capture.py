import sensor
import image
import ustruct
import pyb
import time
from pyb import UART, LED

# ================= 用户配置区域 =================
TARGET_FPS = 10           # 目标帧率 (10 FPS = 100ms 间隔)
BAUD_RATE = 921600        # 串口波特率
# ===============================================

# 定义 LED (OpenMV板载定义: 1=红, 2=绿, 3=蓝)
red_led = LED(1)
green_led = LED(2)

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000) # 给摄像头2秒时间自动调整光线

# 初始状态：确保灯都灭掉
red_led.off()
green_led.off()

# 串口初始化
uart = UART(3, BAUD_RATE, timeout_char=1000)

# 计算最小帧间隔 (100ms)
min_interval_ms = 1000 // TARGET_FPS

def capture_and_send_image():
    # 记录本轮开始时刻
    start_time = time.ticks_ms()

    # ------------------------------------------------
    # 1. 准备阶段：拍照与压缩
    # ------------------------------------------------
    img = sensor.snapshot()
    # 压缩图片 (quality=50 在 921600 波特率下通常能保持流畅)
    img_compressed = img.compress(quality=50)

    # ------------------------------------------------
    # 2. 发送前：闪烁红灯
    # ------------------------------------------------
    red_led.on()
    time.sleep_ms(5)   # 稍微亮一下
    red_led.off()

    # ------------------------------------------------
    # 3. 发送阶段：直接发送，不等待
    # ------------------------------------------------
    # 发送大小 (4字节)
    uart.write(ustruct.pack("<L", len(img_compressed)))
    # 发送图片数据
    uart.write(img_compressed)

    # ------------------------------------------------
    # 4. 发送后：闪烁绿灯
    # ------------------------------------------------
    green_led.on()
    time.sleep_ms(5)   # 稍微亮一下
    green_led.off()

    # ------------------------------------------------
    # 5. 时间控制：补足 100ms
    # ------------------------------------------------
    # 计算刚才拍照、压缩、闪灯、发送总共花了多久
    elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
    
    # 如果处理时间小于 100ms，则休息剩下的时间
    if elapsed_time < min_interval_ms:
        time.sleep_ms(min_interval_ms - elapsed_time)

while True:
    capture_and_send_image()