from ev3dev import ev3

gyro1 = ev3.GyroSensor("in1")
gyro2 = ev3.GyroSensor("in2")


def reset():
	global gyro1
	global gyro2

	gyro1.mode = "GYRO-CAL"
	gyro2.mode = "GYRO-CAL"

	gyro1.mode = "GYRO-ANG"
	gyro2.mode = "GYRO-ANG"

def angle():
	return (gyro1.value() - gyro2.value()) / 2

if __name__ == "__main__":
	reset()
	while True:
		print(angle())
