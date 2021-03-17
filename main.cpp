//git clone https://github.com/LeNgineer-Lennovation/FlightSoftware.git

#define NAVIO_RCOUTPUT_1 12
#define NAVIO_RCOUTPUT_2 13
#define NAVIO_RCOUTPUT_3 14
#define NAVIO_RCOUTPUT_4 15
#define SERVO_MIN 1 // ms
#define SERVO_MAX 1.75 //ms

#include <iostream>
#include <iomanip>

//Navio libraries
#include "gpio.h" //general purpose input/output
#include "Ublox.h" // GPS NEO-M8N
#include "MPU9250.h" // IMU that contains gyro, accel and mag
#include "PCA9685.h" // unknown
#include "MS5611.h" // barometer lib 


using namespace std;
using namespace Navio;


//function declaration
void takeoff(); 
void land();
void rotateCCW();
void forwardMotion();
void hover();


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
	bool isEnd = false;
	vector<double> position_data;
	
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
    cout << "User latitude in feet: " << latitudeFeet_user << endl;
    cout << "User longitude in feet: " << longitudeFeet_user << endl;
    
    while(isEnd == false){
		//lets try to setup gps
		Ublox gps;
		
		if(gps.testConnection()){
			
			cout<<"GPS is working"<<endl;
			if(gps.decodeSingleMessage(Ublox::NAV_POSLLH, position_data) == 1){
				cout<<"GPS Millisecond time of the week: " << position_data[0]<<endl;
				cout<<"Longitude: "<< position_data[1]<<endl;
				cout<<"Latitude: "<< position_data[2]<<endl;
				cout<< "Height above Ellipsoid: "<< position_data[3]<<endl;
				cout<< "Height above mean sea level: "<< position_data[4]<<endl;
				cout<< "Horizontal Accuracy Estateimate: "<< position_data[5]<<endl;
				cout<< "Vertical Accuracy Estateimate: "<< position_data[6]<<endl;
				
				latitudeFeet_gps = ((position_data[2]/10000000) * (10000/90)) * 3280.4;
				longitudeFeet_gps = ((position_data[1]/10000000) * (10000/90)) * 3280.4;
				cout << "GPS latitude in feet: " << latitudeFeet_gps << endl;
				cout << "GPS longitude in feet: " << longitudeFeet_gps << endl;
				
				
				latitudeFeet_diff = latitudeFeet_gps - latitudeFeet_user;
				longitudeFeet_diff = longitudeFeet_gps - longitudeFeet_user;
				cout << "Latitude difference in feet: " << latitudeFeet_diff << endl;
				cout << "Longitude difference in feet: " << longitudeFeet_diff << endl;
			
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
	}
		
	}else{
		cout<<"Cannot start gps"<<endl;	
	}
	
	takeoff();
	
	
	isEnd = true;
	//end while
}
    return 0;
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
