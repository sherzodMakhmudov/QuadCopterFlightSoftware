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
int main(){
	
	float latitudeUser;
	float longitudeUser;
    cout<<"Enter your desired destination. "<<endl;
    cout<<"Enter a valid latitude value between -90 and 90: ";
    cin>>latitudeUser;
    
    
    while(latitudeUser < -90 || latitudeUser > 90){
		latitudeUser = 0;
		cout<<"Please enter a valid latitude value between -90 and 90: ";
		cin>>latitudeUser;
	}
	
	cout<<"You have entered a GPS coordinate of latitude: "<<latitudeUser<<endl;
    cout<<"Enter a valid longitude value between -180 and 180: ";
    cin>>longitudeUser;
    
    while(longitudeUser < -180 || longitudeUser > 180){
		longitudeUser = 0;
		cout<<"Please enter a valid longitude value between -180 and 180: ";
		cin>> longitudeUser;
		
	}
	
	cout<<"You have entered a GPS coordinate of longitude: "<<longitudeUser<<endl;
    
    
    
    //lets try to setup gps
    Ublox gps;
    if(gps.testConnection()){
		cout<<"GPS is working"<<endl;
		
	}else{
		cout<<"Cannot start the gps"<<endl;
		
	}
    
    
    return 0;
}
