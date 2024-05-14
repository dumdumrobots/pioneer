from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Magnetometer import *
from Phidget22.Devices.Spatial import *
import time

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onAngularRateUpdate(self, angularRate, timestamp):
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onMagneticFieldChange(self, magneticField, timestamp):
	print("MagneticField: \t"+ str(magneticField[0])+ "  |  "+ str(magneticField[1])+ "  |  "+ str(magneticField[2]))
	print("Timestamp: " + str(timestamp))
	print("----------")

def onAlgorithmData(self, quaternion, timestamp):
	print("Timestamp: " + str(timestamp))

	eulerAngles = self.getEulerAngles()
	print("EulerAngles: ")
	print("\tpitch: " + str(eulerAngles.pitch))
	print("\troll: " + str(eulerAngles.roll))
	print("\theading: " + str(eulerAngles.heading))

	quaternion = self.getQuaternion()
	print("Quaternion: ")
	print("\tx: " + str(quaternion.x))
	print("\ty: " + str(quaternion.y))
	print("\tz: " + str(quaternion.z))
	print("\tw: " + str(quaternion.w))
	print("----------")

def main():
	accelerometer0 = Accelerometer()
	gyroscope0 = Gyroscope()
	magnetometer0 = Magnetometer()
	spatial0 = Spatial()

	accelerometer0.setDeviceSerialNumber(373407)
	gyroscope0.setDeviceSerialNumber(373407)
	magnetometer0.setDeviceSerialNumber(373407)
	spatial0.setDeviceSerialNumber(373407)

	accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
	gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)
	magnetometer0.setOnMagneticFieldChangeHandler(onMagneticFieldChange)
	spatial0.setOnAlgorithmDataHandler(onAlgorithmData)

	accelerometer0.openWaitForAttachment(5000)
	gyroscope0.openWaitForAttachment(5000)
	magnetometer0.openWaitForAttachment(5000)
	spatial0.openWaitForAttachment(5000)

	spatial0.setHeatingEnabled(True)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	accelerometer0.close()
	gyroscope0.close()
	magnetometer0.close()
	spatial0.close()

main()