from machine import Pin
import rp2
from time import sleep
import utime

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
  pull(noblock)
  mov(x, osr)
  mov(y, x)

  jmp(not_x, "end")

  set(pins, 1) [1]
  set(pins, 0)
  irq(0)

  label("delay") 
  jmp(x_dec, "delay")
  
  label("end")
  mov(x, y)


steps = 0
start = 0
end = 0

def turn(sm):
  global steps
  global start
  global end

  if steps == 0:
    start = utime.ticks_us()
  steps = steps + 1

#  if steps == 6400:
#    end = utime.ticks_us()
#    sm.put(0)

# 6400 steps per revolution
# 10us + delay, delay 500 means 510us per step
class Stepper:
    
  def __init__(self, id, dir_pin, step_pin, step_callback = None):
    self.dir_pin = Pin(dir_pin, Pin.OUT)
    # instruction is 1us
    self._sm = rp2.StateMachine(id, stepper, freq=1_000_000, set_base=Pin(step_pin, Pin.OUT))

    if step_callback is not None:
      self._sm.irq(step_callback)
    
    self._sm.put(0)
    self._sm.active(1)

  def set_steps_per_second(self, steps_per_second):
    if steps_per_second == 0:
      self._sm.put(0)
      return

    if steps_per_second > 0:
      self.dir_pin.high()
    elif steps_per_second < 0:
      self.dir_pin.low()

    delay = abs(round(1_000_000 / steps_per_second)) - 10
    self._sm.put(delay)

motors_en = Pin(22, Pin.OUT)
motors_en.low()

motor = Stepper(0, 18, 19, turn)
print("delay")
sleep(2)
print("one direction")
#motor.set_speed(500) # 508.91 us per step 3.263431s for 6400 steps (1 revolution)
#motor.set_speed(400) # 408.92 us per step
#motor.set_speed(450) # 458.89 us per step
motor.set_steps_per_second(2000)
sleep(5)
print(end - start)

print("stop for a bit")
motor.set_steps_per_second(0)
sleep(2)

print("another direction")
motor.set_steps_per_second(-2000)
sleep(5)
print("stop")
motor.set_steps_per_second(0)
sleep(2)
print("done")