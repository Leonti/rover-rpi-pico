from machine import Pin, I2C, SoftI2C
from ssd1306 import SSD1306_I2C
import framebuf
import math
import utime
WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height

# Explicit Method
sda=Pin(12)
scl=Pin(13)
i2c=I2C(0,sda=sda, scl=scl, freq=400000)
#i2c=SoftI2C(sda=sda, scl=scl, freq=400000)
#  print(i2c.scan())
from ssd1306 import SSD1306_I2C
oled = SSD1306_I2C(128, 64, i2c)

def reset():
  oled.fill(0)
  oled.show()

def battery_status(voltage, current):
  voltage_str = '{0:.2f}'.format(voltage)
  current_str = '{0:.2f}'.format(current)
  oled.fill_rect(5, 5, 123, 8, 0)
  oled.text(voltage_str + 'V', 5, 5)
  oled.text(current_str + 'mA', 60, 5)
  oled.show()

def connected_status(connected):
  message = 'CONNECTED' if connected else 'DISCONNECTED'
  oled.fill_rect(5, 15, 123, 8, 0)
  oled.text(message, 5, 15)
  oled.show()  

def off_in_seconds(remaining_seconds):
  off_message = 'OFF in: {0:d}s'.format(remaining_seconds)
  oled.fill_rect(5, 25, 123, 8, 0)
  oled.text(off_message, 5, 25)
  oled.show()  

def custom_message(message):
  oled.fill_rect(5, 55, 120, 8, 0)
  oled.text(message, 5, 55)
  oled.show()    