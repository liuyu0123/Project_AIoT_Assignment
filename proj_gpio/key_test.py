'''
功能：按键测试
香蕉派接线：将按钮阳极连接至 GPIO-77，引脚阴极连接至 GND
'''
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)

from gpiozero import Button
from signal import pause

pin_number = 77
print(f"Monitoring button on GPIO {pin_number}")

def button_pressed():
    print("Button pressed")

def button_released():
    print("Button released")

button = Button(pin_number)
button.when_pressed = button_pressed
button.when_released = button_released

pause()  # 保持程序运行