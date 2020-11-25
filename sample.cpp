// USB front
#define NAVIO_RCOUTPUT_1 12 // Brown (Left)
#define NAVIO_RCOUTPUT_2 13 // Red (Back)
#define NAVIO_RCOUTPUT_3 14 // Orange (Right)
#define NAVIO_RCOUTPUT_4 15 // White (Front)
#define SERVO_MIN 1 /*mS*/
#define SERVO_MAX 1.75 /*mS*/
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
// PID controller
#include "PID_v1.h"
#include <iostream>
#include <cstdio>
#include <ctime>


//Navio lib:
#include "Navio/Ublox.h"//C++/Common
#include "Navio/MPU9250.h"//C++/Common
#include "Navio/PCA9685.h" //C++/Navio+
#include "Navio/MS5611.h"//C++/Common
#include <Navio/gpio.h> //C++/Common

using namespace std;
using namespace Navio;
long double millis();
void takeoff();
void forward_motion();
void hover(double ax, double ay, double az);
void rotate_CCW();
void land();


int main(int argc, char *argv[])
{
    float latitude_user = 0;
    float longitude_user = 0;
    float latfeet_user;
    float longfeet_user;
    float latfeet_gps;
    float longfeet_gps;
    float lat_diff;
    float long_diff;
    cout << setprecision(8);
    int dest_dir;
    double ground_level;

    printf("To begin please enter in a set of GPS coordinates with the form
    XX.XXXXXX\n");
    printf("Placing a negative sign in front of latitude value causes the computer to treat it as
    a West coordinate\n");
    printf("Placing a negative sign in front of the longitude value causes the computer to treat
    it as a South coordinate\n");
    cout << "Enter a valid Latitude value between -90 and 90: ";
    cin >> latitude_user;

    while(latitude_user > 90 || latitude_user < -90)
    {
        latitude_user = 0;
        cout << "Invalid Latitude. Please enter a longitude value in the correct range: ";
        cin >> latitude_user;
    }
    cout << "Enter a valid Longitude value between -180 and 180: ";
    cin >> longitude_user;
    while(longitude_user > 180 || longitude_user < -180)
    {
        longitude_user = 0;
        cout << "Invalid Longitude. Please enter a longitude value in the correct range: ";
        cin >> longitude_user;
    }

    cout << "You have entered a GPS coordinate of Latitude: " << latitude_user << endl;
    cout << "With a Longitude of: " << longitude_user << endl;
    // Accuracy down to roughly 4 inches per .000001 change
    // .000139 = 50 feet
    latfeet_user = (latitude_user * (10000/90)) * 3280.4;
    longfeet_user = (longitude_user * (10000/90)) * 3280.4;
    cout << "Latitude in feet: " << latfeet_user << endl;
    cout << "Longitude in feet: " << longfeet_user << endl;

    // This vector is used to store location data, decoded from ubx messages.
    // After you decode at least one message successfully, the information is stored in vector
    // in a way described in function decodeMessage(vector<double>& data) of class
    //UBXParser(see ublox.h)

    std::vector<double> pos_data;
    MS5611 barometer;
    barometer.initialize();
 // create ublox class instance
    Ublox gps;
    MPU9250 imu;
    imu.initialize();
    float ax, ay, az, gx, gy, gz, mx, my, mz;

 //-------------------------------------------------------------------------
    while(1) {
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    printf("Acc: %+7.3f %+7.3f %+7.3f ", ax, ay, az);
    printf("Gyr: %+8.3f %+8.3f %+8.3f ", gx, gy, gz);
    printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
/*
barometer.refreshPressure();
 usleep(10000); // Waiting for pressure data ready
 barometer.readPressure();
 barometer.refreshTemperature();
 usleep(10000); // Waiting for temperature data ready
 barometer.readTemperature();
 barometer.calculatePressureAndTemperature();
 printf("Temperature(C): %f Pressure(millibar): %f\n",
 barometer.getTemperature(), barometer.getPressure());
 sleep(1);
 */
if(gps.testConnection())
 {
 printf("Ublox test OK\n");
 // gps.decodeMessages();
 if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
 {
 //printf("GPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
 printf("Longitude: %lf\n", pos_data[1]/10000000);
 printf("Latitude: %lf\n", pos_data[2]/10000000);
 //printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
 printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
 //printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
 //printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
 latfeet_gps = ((pos_data[2]/10000000) * (10000/90)) * 3280.4;
longfeet_gps = ((pos_data[1]/10000000) * (10000/90)) * 3280.4;
cout << "Latitude in feet: " << latfeet_gps << endl;
cout << "Longitude in feet: " << longfeet_gps << endl;
lat_diff = latfeet_gps - latfeet_user;
long_diff = longfeet_gps - longfeet_user;
cout << "Latitude difference in feet: " << lat_diff << endl;
cout << "Longitude difference in feet: " << long_diff << endl;
 // Setting of dest_dir variable
 // N = 1, NE = 2, E = 3, SE = 4 ....
 if(lat_diff > 2) // South
 {
 if(long_diff > 2) // South East
 dest_dir = 4;
 else if(long_diff < -2) // South West
 dest_dir = 6;
 else
 dest_dir = 5;
 }
 else if(lat_diff < -2) //North
 {
 if(long_diff > 2) // North East
 dest_dir = 2;
 else if(long_diff < -2) // North West
 dest_dir = 8;
 else
 dest_dir = 1;
 }
 else if(long_diff > 2) // West
 dest_dir = 7;
 else if(long_diff < -2) // East
 dest_dir = 3;
 } else {
 printf("Message not captured\n");
 }
 if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
 {
 printf("Current GPS status:\n");
 printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));
 printf("gps Fix status: ");
 switch((int)pos_data[0]){
 case 0x00:
 printf("no fix\n");
 break;
 case 0x01:
 printf("dead reckoning only\n");
 break;
 case 0x02:
 printf("2D-fix\n");
 break;
 case 0x03:
 printf("3D-fix\n");
 break;
 case 0x04:
 printf("GPS + dead reckoning combined\n");
 break;
 case 0x05:
 printf("Time only fix\n");
 break;
 default:
 printf("Reserved value. Current state unknown\n");
 break;
 }
 printf("\n");
 } else {
 // printf("Status Message not captured\n");
 } static const uint8_t outputEnablePin = RPI_GPIO_27;
 Pin pin(outputEnablePin);
 if (pin.init()) {
 pin.setMode(Pin::GpioModeOutput);
 pin.write(0); /* drive Output Enable low */
 } else {
 fprintf(stderr, "Output Enable not set. Are you root?");
 }
 }

 // Sets up motors for use
 PCA9685 pwm;
 pwm.initialize();
 pwm.setFrequency(50);

 cout << millis() << endl;
 cout << dest_dir << endl;

 // Set ground level to altitude before lift off
 ground_level = pos_data[4]/1000;

// Set of if statements to check for current drone status
 if(millis() < 300000)
 {
 takeoff();
 }
 else if(az < .93)
 {
 hover(ax, ay, az);
 }
 else if((lat_diff < 2 && lat_diff > -2) && (long_diff < 2 && long_diff > -2) &&
(pos_data[4]/1000 > (ground_level + 1)))
 {
 land();
 }
 else if((lat_diff < 2 && lat_diff > -2) && (long_diff < 2 && long_diff > -2) &&
(pos_data[4]/1000 < (ground_level + 1)))
 {
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, SERVO_MIN);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, SERVO_MIN);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, SERVO_MIN);
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, SERVO_MIN);
 cout << "Done" << endl;
 }
 if(dest_dir == 2 || dest_dir == 8 || dest_dir == 1)
 {
 // turn drone north
 if((mx < 23 || mx > 28) && (my < 51 || my > 57) && az > .93)
 {
 rotate_CCW();
 }
 if(my >= 51 && my <= 57 && mx > 0)
 {
 forward_motion();
 }
 }
 else if(dest_dir == 4 || dest_dir == 5 || dest_dir == 6)
 {
 // turn drone south
 if((mx < -23 || mx > -17) && (my < 52 || my > 59) && az > .93)
 {
 rotate_CCW();
 }
 if(my >= 52 && my <= 59 && mx < 0)
 {
 forward_motion();
 }
 }
 else if(dest_dir == 3)
 {
 // turn drone east
 if((mx < -2 || mx > 8) && (my < 27 || my > 32) && az > .93)
 {
 rotate_CCW();
 }
 if(my >= 27 && my <= 32)
 {
 forward_motion();
 }
 }
 else if(dest_dir == 7)
 {
 // turn drone west
 if((mx < 2 || mx > 7) && (my < 74 || my > 80) && az > .93)
 {
 rotate_CCW();
 }
 if(my >= 74 && my <= 80)
 {
 forward_motion();
 }
 }
}
}
// Called just after a destination is selected to lift off
void takeoff()
{
PCA9685 pwm;
cout << "Taking off" << endl;
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
// Moves drone forward
void forward_motion()
{
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
// Keeps drone level using PID controller
// Called whenever accelerometer value of az drops below .93
void hover(double ax, double ay, double az)
{
 double Out_x, Out_y, Out_z;

 double Set, ControllerDirection = 0; 
 double Kp; double Ki; double Kd; double Input; double Output; double Setpoint;

 P PID_x;
 P PID_y;
 P PID_z;

 PCA9685 pwm;
 cout << "------------------------Hovering------------------------" << endl;
 // Repeatedly call PID controller until drone is level
 Set = 1;
 Input = az; // Input is sensor data
 Setpoint = Set; // Setpoint are values around 0 for stability
 PID_z.PID(Input, Output, Setpoint, Kp, Ki, Kd, ControllerDirection);
 Output = ((Output/4096) * .35) + 1.4;
 Out_z = Output;
 Set = 0;
 Input = ax; // Input is sensor data
 Setpoint = Set; // Setpoint are values around 0 for stability
 PID_x.PID(Input, Output, Setpoint, Kp, Ki, Kd, ControllerDirection);
 Output = ((Output/4096) * .35) + 1.4;
 Out_x = Output;
 Set = 0;
 Input = ay; // Input is sensor data
 Setpoint = Set; // Setpoint are values around 0 for stability
 PID_y.PID(Input, Output, Setpoint, Kp, Ki, Kd, ControllerDirection);
 Output = ((Output/4096) * .35) + 1.4;
 Out_y = Output;
 if(ay > .08)
 {
 // tilt forward
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, Out_y);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.4);
 }
 else if(ay < -.08)
 {
 // tilt backward
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, Out_y);
 }
 else if(ax > .08)
 {
 // tilt right
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.4);
 }
 else if(ax < -.08)
 {
 // tilt left
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.4);
 }
 else if(ax < -.08 && ay > .08)
 {
 // tilt forward-left
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, Out_y);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.4);
 }
 else if(ax > .08 && ay > .08)
 {
 // tilt forward-right
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, Out_y);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, 1.4);
 }
 else if(ax < -0.8 && ay < -.08)
 {
 // tilt backward-left
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, Out_y);
 }
 else if(ax > .08 && ay < -.08)
 {
 // tilt backward-right
 pwm.setPWMmS(NAVIO_RCOUTPUT_4, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_1, 1.4);
 pwm.setPWMmS(NAVIO_RCOUTPUT_3, Out_x);
 pwm.setPWMmS(NAVIO_RCOUTPUT_2, Out_y);
 }
}
// Used to point drone in the right direction after takeoff
void rotate_CCW()
{
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
// Called when GPS coordinates indicate the drone is at it's destination
void land()
{
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
// PID Functions ------------------------------------------------------------
// Constructor (executes PID related functions)
void P::PID(double Input, double Output, double Setpoint,
 double Kp, double Ki, double Kd, int ControllerDirection)
{
 myOutput = Output; // Output is the PWM signal sent to the motors
 myInput = Input; // Input is sensor data
 mySetpoint = Setpoint; // Setpoint are values for the sensors that they should not
cross to maintain stability
inAuto = true;
P::SetOutputLimits(0, 4096, myOutput); // 1.4ms = 0 and 1.75ms = 4096 (2^12)
 SampleTime = 1; //default Controller Sample
Time is 0.001 seconds
lastTime = millis() - SampleTime;
 P::SetControllerDirection(ControllerDirection);
 kp = 2; // Proportional tuning value
 ki = .5; // Integral tuning value
 kd = .05; // Derivative tuning value
 P::Compute(myOutput, myInput, mySetpoint);
}
// Returns true when the output is computed, false when nothing has been done.
bool P::Compute(double myOutput, double myInput, double mySetpoint)
{
 unsigned long now;

 now = millis();

 unsigned long timeChange = (now - lastTime);
 if(!inAuto) return false;
 if(timeChange >= SampleTime)
 {
 /*Compute all the working error variables*/
 double input = myInput;
 double error = mySetpoint - input;
 ITerm += (ki * error);
 if(ITerm > outMax) ITerm = outMax;
 else if(ITerm < outMin) ITerm = outMin;
 double dInput = (input - lastInput);
 /*Compute PID Output*/
 double output = kp * error + ITerm - kd * dInput;
 if(output > outMax) output = outMax;
 else if(output < outMin) output = outMin;
 myOutput = output;
 /*Remember some variables for next time*/
 lastInput = input;
 lastTime = now;
 return true;
 }
 else return false;
}
// Clamps output to between 0 and 4096
void P::SetOutputLimits(double Min, double Max, double myOutput)
{
 if(Min >= Max) return;
 outMin = Min;
 outMax = Max;
 if(inAuto)
 {
 if(myOutput > outMax) myOutput = outMax;
 else if(myOutput < outMin) myOutput = outMin;
 if(ITerm > outMax) ITerm = outMax;
 else if(ITerm < outMin) ITerm = outMin;
 }
}
/* SetControllerDirection
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.) */
void P::SetControllerDirection(int Direction)
{
 if(inAuto && Direction != controllerDirection)
 {
kp = (0 - kp);
 ki = (0 - ki);
 kd = (0 - kd);
 }
 controllerDirection = Direction;
}
long double millis()
{
 long double millisecond;
 millisecond = std::clock();
 return millisecond;
}