/*
                        SquareProgram.ino
                  Created by: Anthony Scalise

    This program uses blynk ui api to control an ESP32 over wifi
    from a phone to test and debug driving a square path.
*/

#include <WiFi.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BlynkSimpleEsp32.h>
#include <utility/imumaths.h>

//  #define SERIAL_DEBUG_LOGS //This line enables logs

//This creates the log function which can be turned off
#ifdef SERIAL_DEBUG_LOGS
#define logP(a) Serial.print(a);
#define logPL(a) Serial.println(a);
#else
#define logP(a)
#define logPL(a)
#endif

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const byte leftMotorForward = 19;  //Pin constant for left motor forwards
const byte rightMotorForward = 18;  //Pin constant for right motor forwards
const byte leftMotorBackward = 15;  //Pin constant for left motor backwards
const byte rightMotorBackward = 23;  //Pin constant for right motor backwards
const byte leftMotorENB = 2;  //Pin constant for left motor speed
const byte rightMotorENB = 5; //Pin constant for right motor speed
const byte motorPinList[] = {leftMotorForward, rightMotorForward, leftMotorBackward, rightMotorBackward, leftMotorENB, rightMotorENB};
const int pwmFrequency = 30000;  //Frequency constant for pwm output
const int pwmResolution = 10; //Resolution constant for pwm output
const int leftMotorPwmChannel = 0;  //Channel constant left channel pwm output
const int rightMotorPwmChannel = 1; //Channel constant right channel pwm output

//Timer variables
uint32_t squareTimer; //Timer for scheduling square program
int squareTimerDelay; //Variable to hold the specified delay for timer
int squareTimerIterationCount; //Variable to keep track of how many times timer has run
int squareInitialTurnAngle;  //Variable to reference for turning

//PID constants
double kp = 1;  //Proportional constant
double ki = 1;  //Integral constant
double kd = 1;  //Derivative constant

//PID used values
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double cumError, rateError;

double RobotYaw;
double PIDOutput;

char auth[] = "WNlAZ0F86VaOY9xX8EWSWp56Upvk45Yi"; //This is where you put your Blynk project ID
char ssid[] = "NETGEAR91";  //This is your LAN WiFi SSID
char pass[] = "curlyshrub060";  //This is your LAN WiFi password

sensor_t sensor;  //Variable for sensor object
sensors_event_t event;  //Varable for sensor returned data

enum RobotModes{teleop, autonomous, square, usdar, circleObject}; //Robot modes for Blynk selection
RobotModes robotModeList[] = {teleop, autonomous, square, usdar, circleObject}; //Array of modes
RobotModes robotMode = teleop;  //Current mode robot should be in

bool isEnabled = false; //Variable to hold state of enabled button

/*  This function allows for motor control by giving a value of power for each side from 1023 to -1023.
    Positive is forwards, negative is reverse, 0 is stopped */
void moveMotors(int powL, int powR) {
  limitRange(powL, 1023, -1023);  //Limits range of left power
  limitRange(powR, 1023, -1023);  //Limits range of right power
  logPL("\nLeft Power: "+String(powL)+"   Right Power: "+String(powR)+"\n");  //Print powers after limiting
  ledcWrite(leftMotorPwmChannel, abs(powL));  //Sets left motor power
  ledcWrite(rightMotorPwmChannel, abs(powR)); //Sets right motor power
  digitalWrite(leftMotorForward, (powL>0)? HIGH : LOW); //If left power is greater than zero set left forward true
  digitalWrite(rightMotorForward, (powR>0)? HIGH : LOW);  //If right power is greater than zero set right forwards true
  digitalWrite(leftMotorBackward, (powL<0)? HIGH : LOW);  //If left power is less than zero set left backwards true
  digitalWrite(rightMotorBackward, (powR<0)? HIGH : LOW); //If right power is less than zero set right backwards true
}

//This function initializes motors
void initializeMotors(void) {
  for(int i=0;i<6;i++) {  //Iterate through motor pin list
    pinMode(motorPinList[i], OUTPUT);  //Set motor pin as output
    digitalWrite(motorPinList[i], LOW);  //Set motor pin low
  }
  ledcSetup(leftMotorPwmChannel, pwmFrequency, pwmResolution);
  ledcSetup(rightMotorPwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(leftMotorENB, leftMotorPwmChannel);
  ledcAttachPin(rightMotorENB, rightMotorPwmChannel);
}

//This function sets up the Blynk controler connection
void initializeBlynk(void) {
  Blynk.begin(auth, ssid, pass, IPAddress(192,168,0,60), 8080); //Starts Blynk connection
  logP("Waiting for connections...");
  while(!Blynk.connected()) {logP(".");} logPL("Done!"); //Waits for connection
  Blynk.virtualWrite(V5, 0);  //Sets the initial value of enable button
  Blynk.virtualWrite(V0, 0);  //Sets the initial value of mode selector
}

BLYNK_WRITE(V0) {robotMode = robotModeList[param.asInt()];} //Set the robot mode
BLYNK_WRITE(V5) {isEnabled = (param.asInt()==1);} //Sets the state of enabled button


void setup(void) {
  Serial.begin(115200);
  initializeMotors(); //Setup motors
  initializeBlynk();  //Setup blynk connection
  bno.getSensor(&sensor); //Gets sensor data for debugging and testing use
  if(!bno.begin()) {while(1);}  //Makes sure imu is connected and if not does not continue
  delay(1000);  //Wait 1 second
  bno.setExtCrystalUse(true); //Set MPU to use onboard external oscillator
}

void loop(void) {
  Blynk.run();  //Update blynk data
  RobotYaw = readYaw(); //Set yaw display value to current yaw
  logP("HEADING YAW: "+String(RobotYaw)+"\t"); //Print current yaw
  if ((robotMode==square) && (isEnabled)) {  //Checks if enabled on blynk ui
    driveSquare();  //Do square program
  } else {  //If not enabled
    moveMotors(0, 0); //Stop motors
    squareScheduler("reset"); //Reset square timer
  }
}

//This function drives in a square using turn to
void driveSquare(void) {
  if (squareTimer==squareTimerIterationCount==0) { //Check if first time runing
    moveMotors(1023, 1023); //Start moving forwards for first side
    squareScheduler("set", delayLength=3000); //Start square schedual timer
  } else if (((squareTimerIterationCount%2)==0) && (squareScheduler("isDone"))) {  //Checks if timer is done and doing a side
    moveMotors(0, 0)  //Stop motors
    squareInitialTurnAngle = readYaw(); //Set reference orientation for turn
    squareTimerIterationCount++;  //Add one iteration
  } else if ((squareTimerIterationCount%2)==1) {  //Checks if doing a corner
    turnTo(addDegrees(squareInitialTurnAngle, 90)); //Turn towards 90 degree from initial angle
    if (isWithinTolerance(readYaw(), squareInitialTurnAngle, 3)) {
      squareTimerIterationCount++;  //Add one iteration
      moveMotors(1023, 1023)  //Drive forward for next line
      squareScheduler("set");  //Start square schedule timer for next line
    }
  }
}

//This function uses the yawPid to turn to the degree setPoint
void turnTo(double targetDeg) {
  int correction = yawPid(readYaw(), targetDeg); //Get the pid output for current yaw and target degree
  logPL("TURN TO CORRECTION: "+String(correction)); //Print log for correction calculated
  PIDOutput = correction; //Set display value to output
  moveMotors(correction, -correction);  //Set motors to the apropriate output of the pid
}

//This function interfaces with the timer for square
bool squareScheduler(String squareTimerRequest, int delayLength=squareTimerDelay) {
  switch(squareTimerRequest) {
    case("set"):  //squareTimer("set") called
      squareTimer = millis(); //Update square timer
      squareTimerDelay = delayLength; //Update delay length if given, if not just use last default
      return true;  //Return
    case("isDone"): //squareTimer("isDone") called
      if ((millis() - squareTimer) >= squareTimerDelay) //Check if square timer is up
        return true;  //Return true timer is up
      else  //If not passed
        return false; //Return  false timer is not up
    case("reset"):  //squareTimer("reset") called
      squareTimer = 0;  //Set timer to 0
      squareTimerIterationCount = 0;  //Set iteration count to 0
  }
}

//This function gets the difference in degrees between two given degrees. Negative is left, Positive is right
int degreeDifference(int currentDeg, int targetDeg) {
  int angleDiff = (targetDeg-currentDeg); //Get the difference between the angles
  if (angleDiff > 180) {angleDiff = ((360-angleDiff)*(-1));}  //Correct difference if greater than 180 to the right
  if (angleDiff < -180) {angleDiff += 360;} //Correct difference if greater than 180 to the left
  return angleDiff; //Returns corrected angle differnce
}

//This function allows for adding degrees within 0 and 360
double addDegrees(double degreeA, double degreeB) {
  double sum = degreeA+degreeB; //Add two degrees
  if (sum >= 0) //Check if sum is positive
    return (sum % 360); //Return sum wrapped to 360 degrees
  else  //Check if sum is negative
    return ((360-abs(sum))%360);  //Return sum wrapped to 360 degrees
}

//This function allows for checking if a value is within a tolerance
bool isWithinTolerance(double valueInQuestion, double targetValue, double tolerance) {
  if ((valueInQuestion >= (targetValue - tolerance)) && (valueInQuestion <= (targetValue + tolerance))) //Check if within tolerance
    return true;  //Return true, within tolerance
  else  //If not in tolerance
    return false; //Return false, not within tolerance
}

//Limits the variable input based on a given range
void limitRange(int& input, int maxLim, int minLim) {
  if (input > maxLim) {input = maxLim;} //Sets max if above range
  if (input < minLim) {input = minLim;} //Sets min if bellow range
}

//Reads and returns the BNO055 MPU yaw in degrees as a double
double readYaw(void) {
  bno.getEvent(&event); //Gets sensor data and puts it in event
  double yawVal = (event.orientation.x); //Gets yaw orientation in degrees after sensor fusion algorithm
  return yawVal; //Returns the yaw degree value
}

//This function calculates yawPid
double yawPid(double inp, double targetDeg) {
  currentTime = millis(); //Get current time
  elapsedTime = (double)(currentTime-previousTime); //Compute elapsed time
  error = degreeDifference(inp, targetDeg); //Compute difference from current to target
  cumError += (error - lastError)/elapsedTime;  //Compute integral
  rateError = (error - lastError)/elapsedTime;  //Computer derivative
  double out = (kp*error + ki*cumError + kd*rateError); //Calculate pid
  lastError = error;  //Remember error
  previousTime = currentTime; //Remember current time
  return out; //Return result
}
