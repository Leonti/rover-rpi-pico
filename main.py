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

def send(message):
  uart.write(bytes(message + '\n', 'utf-8'))

def parse_speed(s):
  try:
    return int(s)
  except ValueError:
    return None

last_keepalive = -1
last_speed = 0
motors_enabled = False
last_bumper_trigger = utime.ticks_ms()

def set_speed(enable, m1_steps_per_second, m2_steps_per_second):
  global motors_en, m1, m2, motors_enabled
  if enable:
    motors_en.low()
    motors_enabled = True
  else:
    motors_en.high()
    motors_enabled = False

  if (m1_steps_per_second != 0 or m2_steps_per_second != 0) and utime.ticks_diff(utime.ticks_ms(), last_bumper_trigger) < 200:
    print("Ignoring set_speed while processing bumper")
  else: 
    m1.set_steps_per_second(m1_steps_per_second)
    m2.set_steps_per_second(m2_steps_per_second)

def on_command(command):
  global last_keepalive, last_speed
  print(command)
  print('-----')
#  send(command)
  split_command = command.split(':', 1)
  command_type = split_command[0]

  # C0:Some message
  if command_type == 'C0' and len(split_command) == 2:
    screen.custom_message(split_command[1])

  # C1:1,1000,1000
  if command_type == 'C1' and len(split_command) == 2:  
    speeds = split_command[1].split(',')
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
  global screen
  voltage = ina.bus_voltage
  current = ina.current
  screen.battery_status(voltage, current)
  send('P:' + str(voltage) + ',' + str(current))

def send_bumpers(left, right):
  send('B:' + str(1 if left else 0) + ',' + str(1 if right else 0))  

startup_time = utime.ticks_ms()
off_received = -1
OFF_TIMEOUT_SECONDS = 30
def turn_off():
  global off_received, startup_time
  print('off_button')
  if off_received == -1 and utime.ticks_diff(utime.ticks_ms(), startup_time) > 3000:
    send('OFF')
    off_received = utime.ticks_ms()
    screen.off_in_seconds(OFF_TIMEOUT_SECONDS)
    print('turning off')

power_button = Pin(7, Pin.IN)
#power_button.irq(handler=turn_off, trigger=Pin.IRQ_RISING)
off_pin = Pin(6, Pin.OUT, Pin.PULL_DOWN)
off_pin.low()

class Bumper():
  def __init__(self, pin, on_start, on_end):
    pin.irq(handler=self.cb, trigger=Pin.IRQ_RISING)
    self.pin = pin
    self.triggered = None
    self.on_start = on_start
    self.on_end = on_end

  def cb(self, pin):
    if self.triggered is None:
      self.triggered = utime.ticks_ms()
      self.on_start()

  def update(self):
    if self.triggered is not None and utime.ticks_diff(utime.ticks_ms(), self.triggered) > 30 and self.pin.value() == 0:
      self.triggered = None
      self.on_end()

to_report_bumper_start_left = False
to_report_bumper_start_right = False
to_report_bumper_end_left = False
to_report_bumper_end_right = False

def on_start_left():
  global to_report_bumper_start_left, last_bumper_trigger
  set_speed(True, 0, 0)
  to_report_bumper_start_left = True
  last_bumper_trigger = utime.ticks_ms()

def on_start_right():
  global to_report_bumper_start_right, last_bumper_trigger
  set_speed(True, 0, 0)
  to_report_bumper_start_right = True
  last_bumper_trigger = utime.ticks_ms()

def on_bumper_end_left():
  global to_report_bumper_end_left
  print("On bumper end left")
  to_report_bumper_end_left = True

def on_bumper_end_right():
  global to_report_bumper_end_right
  print("On bumper end left")
  to_report_bumper_end_right = True

bumper_left = Bumper(Pin(2, Pin.IN, Pin.PULL_DOWN), on_start_left, on_bumper_end_left)
bumper_right = Bumper(Pin(3, Pin.IN, Pin.PULL_DOWN), on_start_right, on_bumper_end_right)

report_deadline = utime.ticks_add(utime.ticks_ms(), 200)
buffer=b''

def find_commands(current_buffer):
  commands = current_buffer.split(b'\n')
  return (commands[0:-1], commands[-1])  

while True:
  if uart.any() > 0:
    recv = uart.read()
    if recv is not None:
      (commands, remaining) = find_commands(recv)
      buffer = remaining
      for command in commands:
        on_command(command.decode('utf-8'))

  bumper_left.update()
  bumper_right.update()

  if utime.ticks_diff(report_deadline, utime.ticks_ms()) < 0:
    report_deadline = utime.ticks_add(utime.ticks_ms(), 1000)
    update_power()
    is_connected = last_keepalive != -1 and utime.ticks_diff(utime.ticks_ms(), last_keepalive) < 2000
    screen.connected_status(is_connected)
    if motors_enabled and (utime.ticks_diff(utime.ticks_ms(), last_speed) > 2000 or not is_connected):
      set_speed(False, 0, 0)
      print('Disabled motors due to inactivity')

  if power_button.value() == 1:
    turn_off()

  if off_received != -1:
    off_elapsed = round(utime.ticks_diff(utime.ticks_ms(), off_received) / 1000)
    screen.off_in_seconds(OFF_TIMEOUT_SECONDS - off_elapsed)
    if off_elapsed >= OFF_TIMEOUT_SECONDS:  
      off_pin.high()

  if to_report_bumper_start_left or to_report_bumper_start_right:
    send_bumpers(bumper_left.triggered is not None, bumper_right.triggered is not None)
    to_report_bumper_start_left = False
    to_report_bumper_start_right = False
    screen.bumper_status(bumper_left.triggered is not None, bumper_right.triggered is not None)

  if to_report_bumper_end_left or to_report_bumper_end_right:
    send_bumpers(bumper_left.triggered is not None, bumper_right.triggered is not None)
    to_report_bumper_end_left = False
    to_report_bumper_end_right = False
    screen.bumper_status(bumper_left.triggered is not None, bumper_right.triggered is not None)

  utime.sleep_ms(2)