#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)


from Kalman import KalmanAngle
import smbus			#import SMBus module of I2C
import time
import math
import rospy
from sensor_msgs.msg import Imu
import sys

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

RestrictPitch = True	#Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


#Read the gyro and acceleromater values from MPU6050
def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(DeviceAddress, SMPLRT_DIV, 7)

	#Write to power management register
	bus.write_byte_data(DeviceAddress, PWR_MGMT_1, 1)

	#Write to Configuration register
	bus.write_byte_data(DeviceAddress, CONFIG, 0)

	#Write to Gyro configuration register
	bus.write_byte_data(DeviceAddress, GYRO_CONFIG, 24)

	#Write to interrupt enable register
	bus.write_byte_data(DeviceAddress, INT_ENABLE, 1)


def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(DeviceAddress, addr)
        low = bus.read_byte_data(DeviceAddress, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
DeviceAddress = 0x68   # MPU6050 device address

ori_cov = float(rospy.get_param('~ori_cov', '0.0025') ) # Orientation covariance
vel_cov = float(rospy.get_param('~vel_cov', '0.02') ) # Angular velocity covariance
acc_cov = float(rospy.get_param('~acc_cov', '0.04') ) # Linear acceleration covariance
imu_i2c = rospy.get_param('~imu_i2c', '0x68') # I2C device No
imu_link = rospy.get_param('~imu_link', 'imu_link') # imu_link name
pub_freq = float( rospy.get_param('~pub_freq', '10') ) # hz of imu pub
pub = rospy.Publisher('imu_data', Imu, queue_size=1)
imuMsg = Imu()
rospy.init_node('mpu6050_node', anonymous=True)

MPU_Init()

time.sleep(1)
#Read Accelerometer raw value
accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

#print(accX,accY,accZ)
#print(math.sqrt((accY**2)+(accZ**2)))
if (RestrictPitch):
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
else:
    roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
    pitch = math.atan2(-accX,accZ) * radToDeg
print(roll)
kalmanX.setAngle(roll)
kalmanY.setAngle(pitch)
gyroXAngle = roll
gyroYAngle = pitch
compAngleX = roll
compAngleY = pitch
gyroZAngle = 0
gyroZRate = 0

timer = time.time()
flag = 0
while True:
	if(flag >100): #Problem with the connection
		print("There is a problem with the connection")
		flag=0
		continue
	try:
	    #Read Accelerometer raw value
	    accX = read_raw_data(ACCEL_XOUT_H)
	    accY = read_raw_data(ACCEL_YOUT_H)
	    accZ = read_raw_data(ACCEL_ZOUT_H)

	    #Read Gyroscope raw value
	    gyroX = read_raw_data(GYRO_XOUT_H)
	    gyroY = read_raw_data(GYRO_YOUT_H)
	    gyroZ = read_raw_data(GYRO_ZOUT_H)

	    dt = time.time() - timer
	    timer = time.time()

	    if (RestrictPitch):
	        roll = math.atan2(accY,accZ) * radToDeg
	        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
	    else:
	        roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
	        pitch = math.atan2(-accX,accZ) * radToDeg

	    gyroXRate = gyroX/131
	    gyroYRate = gyroY/131
	    gyroZRate = gyroZ/16.4

	    if (RestrictPitch):

	        if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
	            kalmanX.setAngle(roll)
	            complAngleX = roll
	            kalAngleX   = roll
	            gyroXAngle  = roll
	        else:
	            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

	        if(abs(kalAngleX)>90):
	            gyroYRate  = -gyroYRate
	            kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
	    else:

	        if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
	            kalmanY.setAngle(pitch)
	            complAngleY = pitch
	            kalAngleY   = pitch
	            gyroYAngle  = pitch
	        else:
	            kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

	        if(abs(kalAngleY)>90):
	            gyroXRate  = -gyroXRate
	            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

		#angle = (rate of change of angle) * change in time
	    gyroXAngle = gyroXRate * dt
	    gyroYAngle = gyroYAngle * dt
	    gyroZAngle += gyroZRate * dt
		gyroZAngle = gyroXAngle - 360 * math.floor(gyroZAngle / 360)
		if (gyroZAngle > 180):
			gyroZAngle = gyroZAngle - 360

		#compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
	    compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
	    compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch

	    if ((gyroXAngle < -180) or (gyroXAngle > 180)):
	        gyroXAngle = kalAngleX
	    if ((gyroYAngle < -180) or (gyroYAngle > 180)):
	        gyroYAngle = kalAngleY

		accx = accX / 16384
		accy = accY / 16384
		accz = accZ / 16384

		gyrox = gyroX / 16.4
		gyroy = gyroY / 16.4
		gyroz = gyroZ / 16.4	    

	    #print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
	    #print(str(roll)+"  "+str(gyroXAngle)+"  "+str(compAngleX)+"  "+str(kalAngleX)+"  "+str(pitch)+"  "+str(gyroYAngle)+"  "+str(compAngleY)+"  "+str(kalAngleY))
	    time.sleep(0.005)
	    imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = imu_link
        imuMsg.orientation_covariance[0] = ori_cov
        imuMsg.orientation_covariance[4] = ori_cov
        imuMsg.orientation_covariance[8] = ori_cov
        imuMsg.angular_velocity_covariance[0] = vel_cov
        imuMsg.angular_velocity_covariance[4] = vel_cov
        imuMsg.angular_velocity_covariance[8] = vel_cov
        imuMsg.linear_acceleration_covariance[0] = acc_cov
        imuMsg.linear_acceleration_covariance[4] = acc_cov
        imuMsg.linear_acceleration_covariance[8] = acc_cov
        imuMsg.orientation.x = float(kalAngleX)
        imuMsg.orientation.y = float(kalAngleY)
        imuMsg.orientation.z = float(gyroZAngle)
        imuMsg.angular_velocity.x = float(gyrox)
        imuMsg.angular_velocity.y = float(gyroy)
        imuMsg.angular_velocity.z = float(gyroz)
        imuMsg.linear_acceleration.x = float(accx)
        imuMsg.linear_acceleration.y = float(accy)
        imuMsg.linear_acceleration.z = float(accz)
        pub.publish(imuMsg)
	    rospy.spin()

	except Exception as exc:
		flag += 1
