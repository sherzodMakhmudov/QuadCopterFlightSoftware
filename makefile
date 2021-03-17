all:
	g++ main.cpp Ublox.cpp I2Cdev.cpp PCA9685.cpp  -o run
	./run
