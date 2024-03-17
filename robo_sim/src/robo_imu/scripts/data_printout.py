from mpu6050.MPU6050 import *
from hmc5883l.HMC5883L import *
import time

time.sleep(1) # delay necessary to allow imu to settle

print('recording data')
while 1:
    try:
        sensor_mpu = MPU6050()
        sensor_hmc = HMC5883L()
        ax,ay,az,gx,gy,gz = sensor_mpu.convert() # read and convert mpu6050 data
        mx,my,mz = sensor_hmc.read_data() # read and convert hmc5883l magnetometer data
    except:
        continue
    
    print('{}'.format('-'*30))
    print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax,ay,az))
    print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(gx,gy,gz))
    print('mag [uT]:   x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(mx,my,mz))
    print('{}'.format('-'*30))
    time.sleep(1)