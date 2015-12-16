# main.py -- put your code here!
from mpu6050 import MPU6050
from pid import PID
from rc import RC, _map, wrap_180
from esc import ESC

# MPU
mpu = MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)
packetSize = mpu.dmpGetFIFOPacketSize() 

# PID
rr_pid = PID(p=0.7, i=1, imax=50)
pr_pid = PID(p=0.7, i=1, imax=50)
yr_pid = PID(p=2.7, i=1, imax=50)
rs_pid = PID(p=4.5)
ps_pid = PID(p=4.5)
ys_pid = PID(p=10)

# RC
rc_thr = RC(0)  # throttle
rc_rol = RC(1)  # roll
rc_pit = RC(2)  # pitch
rc_yaw = RC(3)  # yaw

# ESC
esc_0 = ESC(0)
esc_1 = ESC(1)
esc_2 = ESC(2)
esc_3 = ESC(3)
esc_4 = ESC(4)
esc_5 = ESC(5)

yaw_target = 0
while True:
  # MPU
  mpuIntStatus = mpu.getIntStatus()
  fifoCount = mpu.getFIFOCount()
  if mpuIntStatus < 2 or fifoCount == 1024:
    mpu.resetFIFO()
    print('FIFO overflow!')
    continue
  while fifoCount < packetSize:
    fifoCount = mpu.getFIFOCount()
  fifoCount -= packetSize
  fifoBuffer = mpu.getFIFOBytes(packetSize)
  yaw, rol, pit = mpu.dmpGetEuler(*mpu.dmpGetQuaternion(fifoBuffer))
  g_pit, g_rol, g_yaw = mpu.dmpGetGyro(fifoBuffer)
  yaw -= 6
  #print(rol, pit, yaw, g_rol, g_pit, g_yaw)
  # RC
  #print('%s  %s  %s  %s' % (rc_thr.get_width(), rc_rol.get_width(), rc_pit.get_width(), rc_yaw.get_width()))
  rc_thr_width = rc_thr.get_width()
  rc_rol_width = _map(rc_rol.get_width(), 995, 1945, -45, 45)
  rc_pit_width = _map(rc_pit.get_width(), 995, 1945, -45, 45)
  rc_yaw_width = _map(rc_yaw.get_width(), 995, 1945, -180, 180)
  #print('%s  %s  %s' % (rc_rol_width, rc_pit_width, rc_yaw_width))
  if rc_thr_width > 1200:
    # Stablise PIDS
    rol_stab_out = max(min(rs_pid.get_pid(rc_rol_width + rol, 1), 250), -250)
    pit_stab_out = max(min(ps_pid.get_pid(rc_pit_width + pit, 1), 250), -250)
    yaw_stab_out = max(min(ys_pid.get_pid(wrap_180(yaw_target + yaw), 1), 360), -360)
    #print('%s  %s  %s' % (rol_stab_out, pit_stab_out, yaw_stab_out))
    if abs(rc_yaw_width) > 5:
      yaw_stab_out = rc_yaw_width
      yaw_target = yaw
    # rate PIDS
    rol_out = max(min(rr_pid.get_pid(rol_stab_out + g_rol, 1), 500), -500)
    pit_out = max(min(pr_pid.get_pid(pit_stab_out + g_pit, 1), 500), -500)
    yaw_out = max(min(yr_pid.get_pid(yaw_stab_out + g_yaw, 1), 500), -500)
    #print('%s  %s  %s' % (rol_out, pit_out, yaw_out))
    # ESC
    yaw_out = rc_yaw_width
    esc_0.move(rc_thr_width - (0.866 * pit_out) + (0.5 * rol_out) - yaw_out)
    esc_1.move(rc_thr_width - (0.866 * pit_out) - (0.5 * rol_out) + yaw_out)
    esc_2.move(rc_thr_width - rol_out - yaw_out)
    esc_3.move(rc_thr_width + (0.866 * pit_out) - (0.5 * rol_out) + yaw_out)
    esc_4.move(rc_thr_width + (0.866 * pit_out) + (0.5 * rol_out) - yaw_out)
    esc_5.move(rc_thr_width + rol_out + yaw_out)
  else:
    esc_0.move(rc_thr_width)
    esc_1.move(rc_thr_width)
    esc_2.move(rc_thr_width)
    esc_3.move(rc_thr_width)
    esc_4.move(rc_thr_width)
    esc_5.move(rc_thr_width)
    yaw_target = yaw
    rr_pid.reset_I()
    pr_pid.reset_I()
    yr_pid.reset_I()
    ps_pid.reset_I()
    rs_pid.reset_I()
    ys_pid.reset_I()
