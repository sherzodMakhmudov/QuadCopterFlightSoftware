all:
	g++ main.cpp Ublox.cpp gpio.cpp I2Cdev.cpp PCA9685.cpp MPU9250.cpp MS5611.cpp PWM.cpp Util.cpp  RCOutput_Navio2.cpp  -o run
	./run
