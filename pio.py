import time
import rp2
from machine import Pin

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper():
  pull()
  mov(y, osr) # steps

  pull()  # get delay, but keep it in osr
 
  label("step")
  mov(x, osr)  # get delay from OSR
  set(pins, 1)
  label("delay")
  set(pins, 0)
  jmp(x_dec, "delay")
  irq(rel(0))
  jmp(y_dec, "step")

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper2():
  pull(noblock)
  mov(x, osr) # Keep most recent pull data stashed in X, for recycling by noblock
  mov(y, isr) # ISR must be preloaded with PWM count max
  
  jmp(not_x, "end")

  set(pins, 1)
  set(pins, 0)
  irq(0)

  label("pwmloop")
  jmp(x_not_y, "skip")
  jmp("end")
  label("skip")
  jmp(y_dec, "pwmloop")

  label("end")

def turn(sm):
  print("irq")

# at 10_000Hz, 1 tick is 100us
#sm = rp2.StateMachine(0, stepper, freq=10000, set_base=Pin(25))

#sm.irq(turn)
#sm.active(1)
#sm.put(3)  # steps
#sm.put(2)  # delay

sm = rp2.StateMachine(0, stepper2, freq=10000, set_base=Pin(25))
sm.irq(turn)

sm.put(10000) #y
sm.exec("pull()")
sm.exec("mov(isr, osr)")
sm.put(0) #x

sm.active(1)
print("not moving")
time.sleep(5)
#print("moving")
#sm.put(10000 - 1000) #x
#time.sleep(5)
#print("slower")
#sm.put(1000) #x
#time.sleep(5)
sm.active(0)