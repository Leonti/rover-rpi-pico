import time
import rp2
from machine import Pin

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
  pull(noblock)
  mov(x, osr)
  mov(y, x)

#  jmp(not_x, "end")
  irq(0)

  set(pins, 1)
  set(pins, 0)

  label("delay")
  jmp(x_dec, "delay")
  
  mov(x, y)

#  label("end")

  
def turn(sm):
  print("irq")

sm = rp2.StateMachine(0, stepper, freq=10000, set_base=Pin(25))
sm.irq(turn)

sm.put(10000)
sm.active(1)
print("moving")
time.sleep(5)
print("faster")
sm.put(5000)
time.sleep(5)
print("even faster")
sm.put(1000)
time.sleep(5)

sm.active(0)