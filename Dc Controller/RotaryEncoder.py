import pigpio

class Encoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, pi, gpioA, gpioB):

      """
      Instantiate the class with the pi and gpios connected to
      rotary encoder contacts A and B.  The common contact
      should be connected to ground.  The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for clockwise and -1 for
      counterclockwise.
      """

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB

      self.levA = 0
      self.levB = 0
      
      self.newState = 0
      self.lastState = 0
      
      self.ticks = 0

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if(gpio == self.gpioA):
         self.levA = level
      else:
         self.levB = level;
      self.newState = (self.levA ^ self.levB) | self.levB << 1
      delta = (self.newState - self.lastState) % 4
      if(delta == 1):
         self.ticks += 1
      elif(delta == 3):
         self.ticks -= 1
      '''
      elif delta == 2:
         self.ticks -= 1
      '''

      self.lastState = self.newState

   def cancel(self):
      """
      Cancel the rotary encoder decoder.
      """
      self.cbA.cancel()
      self.cbB.cancel()

   def getTicks(self):

      return self.ticks
