import serial, rospy, re, math
import datetime
import time as tim
import globalv 
import numpy as np
from sensor_msgs.msg import Imu


cmd_yaw = "yaw di.\r\n"
cmd_roll = "roll di.\r\n"
cmd_pitch = "pitch di.\r\n"
cmd_accelero = "accelp di.\r\n"
cmd_gyro = "gyrop di.\r\n"
cmd_temp = "temperature di.\r\n"
cmd_magno = "magp di.\r\n"

port = '/dev/ttyS5'
#port = '/dev/ttyUSB0' # Intel Nuc
baud = 115200

serimu = serial.Serial(port, baud)
serimu.flush()

def time():
    t =  datetime.datetime.now()
    t = str(t)
    t = t[17:]
    t = float(t)
    return t

def yaw():
    flag = 0
    serimu.flush()
    while flag == 0:
        serimu.write(cmd_yaw)
        t = time()
        while abs(t-time() < 0.01) and flag == 0 : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("yaw ="):
                    yaw = line.split("=")
                    yaw = float(yaw[1])
                    flag = 1
    return yaw

def roll():
    flag = 0
    serimu.flush()
    while flag == 0:
        serimu.write(cmd_roll)
        t = time()
        while abs(t-time() < 0.01) and flag == 0: 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("roll ="):
                    roll = line.split("=")
                    roll = float(roll[1])
                    flag = 1
    return roll

def pitch():
    flag = 0
    serimu.flush()
    while flag == 0:
        serimu.write(cmd_pitch)
        t = time()
        while abs(t-time() < 0.01) and flag == 0 : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("pitch ="):
                    pitch = line.split("=")
                    pitch = float(pitch[1])
                    flag = 1
    return pitch

	
def gyro():
    flagx = 0
    flagy = 0
    flagz = 0
    serimu.flush()
    while flagx == 0 or flagy == 0 or flagz == 0:
        serimu.write(cmd_gyro)
        t = time()
        while abs(t-time() < 0.01) and (flagx == 0 or flagy == 0 or flagz == 0) : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("gyrop = 00--"):
                    gx = line.split("--")
                    gx = float(gx[1])
                    flagx = 1
                if line.startswith ("01--"):
                    gy = line.split("--")
                    gy = float(gy[1])
                    flagy = 1
                if line.startswith ("02--"):
                    gz = line.split("--")
                    gz = float(gz[1])
                    flagz = 1
    g = [gx,gy,gz]
    return g

def accelerometer():
    flagx = 0
    flagy = 0
    flagz = 0
    serimu.flush()
    while flagx == 0 or flagy == 0 or flagz == 0:
        serimu.write(cmd_accelero)
        t = time()
        while abs(t-time() < 0.01) and (flagx == 0 or flagy == 0 or flagz == 0) : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("accelp = 00--"):
                    ax = line.split("--")
                    ax = float(ax[1])
                    flagx = 1
                if line.startswith ("01--"):
                    ay = line.split("--")
                    ay = float(ay[1])
                    flagy = 1
                if line.startswith ("02--"):
                    az = line.split("--")
                    az = float(az[1])
                    flagz = 1
        a = [ax,ay,az]
        return a

def magnometer():
    flagx = 0
    flagy = 0
    flagz = 0
    serimu.flush()
    while flagx == 0 or flagy == 0 or flagz == 0:
        serimu.write(cmd_magno)
        t = time()
        while abs(t-time() < 0.01) and (flagx == 0 or flagy == 0 or flagz == 0) : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("magp = 00--"):
                    mx = line.split("--")
                    mx = float(mx[1])
                    flagx = 1
                if line.startswith ("01--"):
                    my = line.split("--")
                    my = float(my[1])
                    flagy = 1
                if line.startswith ("02--"):
                    mz = line.split("--")
                    mz = float(mz[1])
                    flagz = 1
        m = [mx,my,mz]
        return m
		
		
def temperature():
    flag = 0
    serimu.flush()
    while flag == 0:
        serimu.write(cmd_temp)
        t = time()
        while abs(t-time() < 0.01) and flag == 0 : 
            read = serimu.readline()
            for line in read.split('\r') :
                if line.startswith ("temperature ="):
                    temp = line.split("=")
                    temp = float(temp[1])
                    flag = 1
    return temp

		
def imu():
	global ser 
	publisher = rospy.Publisher('sensors/imu', Imu, queue_size=10)
	ser = None 
	
	rospy.init_node('imu')
	rate = rospy.Rate(100)
	
	imu_msg = Imu()
	imu_msg.header.seq = 0;
	imu_msg.header.frame_id = "imu0"
	
	imu_msg.orientation_covariance = [0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001]
    imu_msg.angular_velocity_covariance = [0.000001, 0.0, 0.0, 0.0, 0.000001, 0.0, 0.0, 0.0, 0.000001]
    imu_msg.linear_acceleration_covariance = [0.00117, 0.0, 0.0, 0.0, 0.00277, 0.0, 0.0, 0.0, 0.00034]
	
	try:
		ser = serial.Serial(port,baud)
		ser.flush()
	except serial.SerialException:
		rospy.logerr("Error Connecting to IMU") 
		
	while not rospy.is_shutdown():
		if not ser:
			continue
		
		imu_msg.header.seq += 1
		imu_msg.header.stamp = rospy.get_rostime() 
		
		data = get_imu_data(magnometer())
		imu_msg.orientation.x = data[0]
		imu_msg.orientation.y = data[1]
		imu_msg.orientation.z = data[2]
		
		data = get_imu_data(gyro())
		imu_msg.angular_velocity.x = data[0] * math.pi / 180 / 1000
		imu_msg.angular_velocity.y = data[1] * math.pi / 180 / 1000
		imu_msg.angular_velocity.z = data[2] * math.pi / 180 / 1000
		
		data = get_imu_data(accelerometer())
		imu_msg.linear_acceleration.x = data[0] * 9.80665 / 1000
		imu_msg.linear_acceleration.y = data[1] * 9.80665 / 1000
		imu_msg.linear_acceleration.z = data[2] * 9.80665 / 1000
		
		publisher.publish(imu_msg)
		rate.sleep()
		
	def get_imu_data(self,command):
		self.sr.write(command)
		data = ser.readline()
		values = np.array(re.findall('([-\d.]+)' , data)).astype(np.float)
		return values


if __name__ == "__main__":
	try:
		imu()
	except rospy.ROSInterruptException:
		pass
	
	
	
# if __name__ == "__main__": 
	# Iteration = 0
	# while 1:
		# print ("Roll " + str(roll()))
		# print ("Pitch " + str(pitch()))
		# print ("Yaw " + str(yaw()))
		# print ("Temperature " + str(temperature()))
		# print ("Acceleration " + str(accelerometer()))	
		# print ("Gyro " + str(gyro()))
		# print ("Time " + str(time()))
		# print ("Magno " + str(magnometer()))
		# Iteration += 1
		# print("Iteration: " + str(Iteration))
		# print(" ")
		# tim.sleep(5)	