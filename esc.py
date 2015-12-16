from pyb import Pin, Timer

esc_pins = ['X1', 'X2', 'X3', 'X6', 'Y9', 'Y10']
esc_pins_timers = [5, 5, 5, 2, 2, 2]
esc_pins_channels = [1, 2, 3, 1, 3, 4]
esc_trim = [0, 0, 0, 0, 0, 0]

class ESC:
  freq_min = 950
  freq_max = 1950
  def __init__(self, index):
    self.timer = Timer(esc_pins_timers[index], prescaler=83, period=19999)
    self.channel = self.timer.channel(esc_pins_channels[index],
                                      Timer.PWM,
                                      pin=Pin(esc_pins[index]))
    self.trim = esc_trim[index]
  def move(self, freq):
    freq = min(self.freq_max, max(self.freq_min, freq + self.trim))
    self.channel.pulse_width(int(freq))
  def __del__(self):
    self.timer.deinit()
