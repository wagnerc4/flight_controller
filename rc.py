from pyb import Timer, Pin

rc_timers = [12, 4]
rc_pins_timers = [0, 0, 1, 1]
rc_pins = ['Y8', 'Y7', 'Y4', 'Y3']
rc_pins_channels = [2, 1, 4, 3]

timers = [Timer(k, prescaler=83, period=0x0fffffff) for k in rc_timers]
#timer = Timer(4, prescaler=83, period=0x0fffffff)

class RC:
  start = width = last_width = 0
  def __init__(self, index):
    timer = timers[rc_pins_timers[index]]
    self.pin = Pin(rc_pins[index])
    self.channel = timer.channel(rc_pins_channels[index],
                                 Timer.IC,
                                 pin=self.pin,
                                 polarity=Timer.BOTH)
    self.channel.callback(self.callback)
  def callback(self, timer):
    if self.pin.value(): self.start = self.channel.capture()
    else: self.width = self.channel.capture() - self.start & 0x0fffffff
  def get_width(self):
    w = self.width
    self.last_width = w if w > 950 and w < 1950 else self.last_width
    return self.last_width
  def __del__(self):
    self.timer.deinit()


def _map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def wrap_180(x):
  return x+360 if x < -180 else (x-360 if x > 180 else x)
