'''
测试LED灯
GPIO 输出操作示例：以下示例展示如何控制 GPIO 输出信号以点亮 LED。
物理连接：将 RGB 蓝色 LED 的正极连接至 GPIO-70，负极通过限流电阻连接至 GND
'''
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from gpiozero import LED
import time

pin_number = 70
led = LED(pin_number)

try:
    while True:
        led.on()
        print(f"GPIO {pin_number} ON")
        time.sleep(1)

        led.off()
        print(f"GPIO {pin_number} OFF")
        time.sleep(1)

except KeyboardInterrupt:
    print("\n程序终止")
finally:
    led.close()