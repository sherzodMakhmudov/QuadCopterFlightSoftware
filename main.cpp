//git clone https://github.com/LeNgineer-Lennovation/FlightSoftware.git

#define NAVIO_RCOUTPUT_1 11
#define NAVIO_RCOUTPUT_2 12
#define NAVIO_RCOUTPUT_3 13
#define NAVIO_RCOUTPUT_4 14
#define SERVO_MIN 1 // ms
#define SERVO_MAX 1.75 //ms

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <cstdio>
#include <ctime>



//Navio libraries
#include "gpio.h" //general purpose input/output
#include "Ublox.h" // GPS NEO-M8N
#include "MPU9250.h" // IMU that contains gyro, accel and mag
#include "PCA9685.h" // motor
#include "MS5611.h" // barometer lib 
#include "RCOutput_Navio2.h"
#include "PWM.h"
#include "RCOutput.h"
#include "Util.h"


using namespace std;
using namespace Navio;


//function declaration
void takeoff(); 
void land();
void rotateCCW();
void forwardMotion();
void hover(double ax, double ay, double az);
long double millis();

int main(){
	
	float latitude_user;
	float longitude_user;
	float latitudeFeet_gps;
	float longitudeFeet_gps;
	float latitudeFeet_user;
	float longitudeFeet_user;
	float latitudeFeet_diff;
	float longitudeFeet_diff;
	int destination;
	double ground_level;
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	int counter = 0;
	vector<double> pos_data;
	
	
	
    cout<<"Enter your desired destination. "<<endl;
    cout<<"Enter a valid latitude value between -90 and 90: ";
    cin>>latitude_user;
    
    
    while(latitude_user < -90 || latitude_user > 90){
		latitude_user = 0;
		cout<<"Please enter a valid latitude value between -90 and 90: ";
		cin>>latitude_user;
	}
	
	cout<<"You have entered a GPS coordinate of latitude: "<<latitude_user<<endl;
    cout<<"Enter a valid longitude value between -180 and 180: ";
    cin>>longitude_user;
    
    while(longitude_user < -180 || longitude_user > 180){
		longitude_user = 0;
		cout<<"Please enter a valid longitude value between -180 and 180: ";
		cin>> longitude_user;
		
	}
	
	cout<<"You have entered a GPS coordinate of longitude: "<<longitude_user<<endl;
    
    // Accuracy down to roughly 4 inches per .000001 change
    // .000139 = 50 feet
    latitudeFeet_user = (latitude_user * (10000/90)) * 3280.4;
    longitudeFeet_user = (longitude_user * (10000/90)) * 3280.4;
    printf("User latitude in feet: %+7.3f", latitudeFeet_user);
    cout<<endl;
    printf("User longitude in feet: %+7.3f", longitudeFeet_user);
    cout << endl;
    
    MS5611 barometer;
    barometer.initialize();
    Ublox gps;
    MPU9250 imu;
    imu.initialize();
    
    while(1){
		
		imu.update();

		printf("Acc: %+7.3f %+7.3f %+7.3f ", ax, ay, az);
		printf("Gyr: %+8.3f %+8.3f %+8.3f ", gx, gy, gz);
		printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
		
		
		if(gps.testConnection()){
			
		cout<<"GPS is working"<<endl;
				
		if(gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1){
						printf("Longitude: %lf\n", pos_data[1]);
						printf("Latitude: %lf\n", pos_data[2]);
						printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
              
				
						latitudeFeet_gps = ((pos_data[2]) * (10000/90)) * 3280.4;
						longitudeFeet_gps = ((pos_data[1]) * (10000/90)) * 3280.4;
						printf("Latitude in feet: %.3lf m\n ", latitudeFeet_gps);
						printf("GPS longitude in feet: %.3lf m\n ", longitudeFeet_gps);
				
				
						latitudeFeet_diff = latitudeFeet_gps - latitudeFeet_user;
						longitudeFeet_diff = longitudeFeet_gps - longitudeFeet_user;
						printf("Latitude difference in feet: %.3lf m\n ", latitudeFeet_diff);
						printf("Longitude difference in feet: %.3lf m\n", longitudeFeet_diff);
			
//The points of the compass:
// Setting of dest_dir variable
// N = 1, NE = 2, E = 3, SE = 4
// S = 5, SW = 6, W = 7, NW = 8
				 
/*
    1. Agar lat farqi 2 dan kotta bosa va long 2 dan kotta bosa South eastga yur
    2. Agar lat farqi 2 dan kotta bosa va long -2 dan kicik bosa South westga yur
    3. Agar lat farqi 2 dan kotta bosa Southga yur.

    4. Agar lat farqi -2 dan kicik bosa va long 2 dan kotta bosa North eastga yur
    5. Agar lat farqi -2 dan kicik bosa va long -2 dan kicik bosa North westga yur
    6. Agar lat farqi -2 dan kicik bosa Northga yur
    
    7. Agar long farqi 2 dan kotta bosa Westga yur
    8. Agar long farqi -2 dan kicik bosa Eastga yur
 */
 
 //Let's find out the quad directions
 
		if(latitudeFeet_diff>2){
			if(longitudeFeet_diff>2){
				destination = 4;
			}else if(longitudeFeet_diff<-2){
				destination = 6;
			}else destination = 5;
		}else if(latitudeFeet_diff<-2){
			if(longitudeFeet_diff>2){
				destination = 2;
			}else if(longitudeFeet_diff<-2){
				destination = 8;
			}else destination = 1;
		}else if(longitudeFeet_diff >2){
			destination = 7;
		}else if(longitudeFeet_diff<-2){
			destination = 3;
		}		 
	
	
	cout<<"Destination: "<<destination<<endl;
	
				
				
			
	
	//Navigation Status
	if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
	{
		printf("Current GPS status:\n");
		printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));
		printf("gps Fix status: ");
		switch((int)pos_data[0]){
			case 0x00: printf("no fix\n");
			break;
			
			case 0x01: printf("dead reckoning only\n");
			break;
 
			case 0x02: printf("2D-fix\n");
			break;
 
			case 0x03: printf("3D-fix\n");
			break;
			
			case 0x04: printf("GPS + dead reckoning combined\n");
			break;
 
			case 0x05: printf("Time only fix\n");
			break;
 
			default: printf("Reserved value. Current state unknown\n");
			break;
		}
	}else{
		printf("Status Message not captured\n");
	}
	
	
	static const uint8_t outputEnablePin = RPI_GPIO_27;
	Pin pin(outputEnablePin);
	
	if (pin.init()) {
		pin.setMode(Pin::GpioModeOutput);
		pin.write(0); // drive Output Enable low
	}else{
		fprintf(stderr, "Output Enable not set. Are you root?\n");
	}
		
	}else{
	cout<<"Cannot start gps"<<endl;	
}
	 
}	
	
	
	PCA9685 pwm;
	pwm.initialize();
	pwm.setFrequency(50);
	
	cout<<"Millis: "<<millis()<<endl;
	cout<<"Destination:"<<destination<<endl;
	
	// Set ground level to altitude before lift off
	ground_level = pos_data[4]/1000;
	
	if(millis()<300000){
		takeoff();
	}else if(az< .93){
		hover(ax, ay, az);
	}else if((latitudeFeet_diff < 2 && latitudeFeet_diff > -2) && (longitudeFeet_diff < 2 && longitudeFeet_diff > -2) &&(pos_data[4]/1000 > (ground_level + 1))){
		land();
	}else if((latitudeFeet_diff < 2 && latitudeFeet_diff > -2) && (longitudeFeet_diff < 2 && longitudeFeet_diff > -2) &&(pos_data[4]/1000 < (ground_level + 1))){
		pwm.setPWMmS(NAVIO_RCOUTPUT_1, SERVO_MIN);
		pwm.setPWMmS(NAVIO_RCOUTPUT_2, SERVO_MIN);
		pwm.setPWMmS(NAVIO_RCOUTPUT_3, SERVO_MIN);
		pwm.setPWMmS(NAVIO_RCOUTPUT_4, SERVO_MIN);
		cout << "Done" << endl;
	}
	
	
	if(destination == 2 || destination == 8 || destination == 1){
		//turn the drone north
		if((mx < 23 || mx > 28) && (my < 51 || my > 57) && az > .93){
			rotateCCW();
		}
		if(my >= 51 && my <= 57 && mx > 0){
			forwardMotion();
		}
	}else if(destination == 4 || destination == 5 || destination == 6){
		//turn the drone south
		if((mx < -23 || mx > -17) && (my < 52 || my > 59) && az > .93){
			rotateCCW();
		}
		if(my >= 52 && my <= 59 && mx < 0){
			forwardMotion();
		}
		
	}else if(destination == 3){
		//turn the drone east
		if((mx < -2 || mx > 8) && (my < 27 || my > 32) && az > .93){
			rotateCCW();
		}
		if(my >= 27 && my <= 32){
			forwardMotion();
		}
		
	}else if(destination == 7){
		// turn drone west
		if((mx < 2 || mx > 7) && (my < 74 || my > 80) && az > .93){
			rotateCCW();
		}
		if(my >= 74 && my <= 80){
			forwardMotion();
		}
 }
	
	
}
}


void takeoff(){
	PCA9685 pwm;
	cout<<"Taking off..."<<endl;
	sleep(10);
	pwm.setPWMmS(NAVIO_RCOUTPUT_1, SERVO_MIN);
	pwm.setPWMmS(NAVIO_RCOUTPUT_2, SERVO_MIN);
	pwm.setPWMmS(NAVIO_RCOUTPUT_3, SERVO_MIN);
	pwm.setPWMmS(NAVIO_RCOUTPUT_4, SERVO_MIN);
	sleep(5);
	pwm.setPWMmS(NAVIO_RCOUTPUT_1, SERVO_MAX);
	pwm.setPWMmS(NAVIO_RCOUTPUT_2, SERVO_MAX);
	pwm.setPWMmS(NAVIO_RCOUTPUT_3, SERVO_MAX);
	pwm.setPWMmS(NAVIO_RCOUTPUT_4, SERVO_MAX);
	
}

void land(){
	PCA9685 pwm;
	for(float i = 1.4; i > 1.0; i = i - .0001)
	{
		pwm.setPWMmS(NAVIO_RCOUTPUT_4, i);
		pwm.setPWMmS(NAVIO_RCOUTPUT_1, i);
		pwm.setPWMmS(NAVIO_RCOUTPUT_3, i);
		pwm.setPWMmS(NAVIO_RCOUTPUT_2, i);
		usleep(1000);
		cout << "Landing" << endl;
}

}

void forwardMotion(){
	PCA9685 pwm;
	//Set forward facing motor to minimum throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.1);
	//Set side motors to mid throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
	pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
	//Set rear motor to max throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.75);
	cout << "------------------------Moving------------------------" << endl;	
	
}

void rotateCCW(){
	
	PCA9685 pwm;
	// Set forward facing motor to minimum throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.1);
	// Set side motors to max throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.75);
	pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.75);
	// Set rear motor to max throttle
	pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.1);
	cout << "-------------------------------Rotating-----------------------------------" << endl;
	
}

void hover(double ax, double ay, double az){
	//working on it
 
 
}



long double millis()
{
 long double millisecond;
 millisecond = std::clock();
 return millisecond;
}
