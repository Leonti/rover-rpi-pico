import screen
from machine import Pin, UART, Timer, I2C
import utime
from stepper import Stepper
from ina219 import INA219

# OFF button
# GP6 = blue - control
# GP7 = green - signal

i2c = I2C(1, scl=Pin(15), sda=Pin(14))
ina = INA219(i2c)
ina.set_calibration_32V_2A()

screen.reset()

#uart = UART(id = 1, baudrate = 9600, tx = Pin(4), rx = Pin(5))
uart = UART(id = 0, baudrate = 115200, tx = Pin(0), rx = Pin(1))
m1 = Stepper(0, 18, 19)
m2 = Stepper(1, 20, 21)
motors_en = Pin(22, Pin.OUT)
motors_en.high()

buffer=b''

def send(message):
  uart.write(bytes(message + '\n', 'utf-8'))

def parse_speed(s):
  try:
    return int(s)
  except ValueError:
    return None

last_keepalive = 0
last_speed = 0
motors_enabled = False

def set_speed(enable, m1_steps_per_second, m2_steps_per_second):
  global motors_en, m1, m2, motors_enabled
  if enable:
    motors_en.low()
    motors_enabled = True
  else:
    motors_en.high()
    motors_enabled = False
  m1.set_steps_per_second(m1_steps_per_second)
  m2.set_steps_per_second(m2_steps_per_second)

def on_command(command):
  global last_keepalive, last_speed
  print(command)
  send(command)
  split_command = command.split(':', 1)
  command_type = split_command[0]

  # C0:Some message
  if command_type == 'C0' and len(split_command) == 2:
    screen.custom_message(split_command[1])

  # C1:1,1000,1000
  if command_type == 'C1' and len(split_command) == 2:  
    speeds = split_command[1].split(',')
    print(split_command[1])
    if len(speeds) == 3:
      enable = parse_speed(speeds[0])
      m1_steps_per_second = parse_speed(speeds[1])
      m2_steps_per_second = parse_speed(speeds[2])
      if enable is not None and m1_steps_per_second is not None and m2_steps_per_second is not None:
        set_speed(enable == 1, m1_steps_per_second, m2_steps_per_second)
        last_speed = utime.ticks_ms()  

  # C3
  if command_type == 'C3':
    last_keepalive = utime.ticks_ms()

def update_power():
  global screen, uart
  voltage = ina.bus_voltage
  current = ina.current
  screen.battery_status(voltage, current)
  send('P:' + str(voltage) + ',' + str(current))

off_received = -1
OFF_TIMEOUT_SECONDS = 30
def turn_off():
  global off_received
  send('OFF')
  off_received = utime.ticks_ms()
  screen.off_in_seconds(OFF_TIMEOUT_SECONDS)
  print('turning off')

power_button = Pin(7, Pin.IN, Pin.PULL_DOWN)
power_button.irq(turn_off, Pin.IRQ_RISING)

report_deadline = utime.ticks_add(utime.ticks_ms(), 200)
while True:
  if uart.any() > 0:
    c = uart.read(1)
    if c == b'\n' and len(buffer) > 0:
      on_command(buffer.decode('utf-8'))
      buffer = b''
    elif len(buffer) > 0 or (len(buffer) == 0 and c == b'C'):
      buffer += c

  if utime.ticks_diff(report_deadline, utime.ticks_ms()) < 0:
    report_deadline = utime.ticks_add(utime.ticks_ms(), 1000)
    update_power()
    is_connected = utime.ticks_diff(utime.ticks_ms(), last_keepalive) < 2000
    screen.connected_status(is_connected)
    if motors_enabled and (utime.ticks_diff(utime.ticks_ms(), last_speed) > 2000 or not is_connected):
      set_speed(False, 0, 0)
      print('Disabled motors due to inactivity')

  if off_received != -1:
    off_elapsed = utime.ticks_diff(utime.ticks_ms(), off_received) / 1000
    screen.off_in_seconds(OFF_TIMEOUT_SECONDS - off_elapsed)
    if off_elapsed >= OFF_TIMEOUT_SECONDS:
      off_pin = Pin(6, Pin.OUT)
      off_pin.high()

  utime.sleep_us(100)