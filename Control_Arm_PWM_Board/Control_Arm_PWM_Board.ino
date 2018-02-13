/* **************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This code controls the digital signals, PWM and linear actuators with a PWM feedback loop.
 *
 *Author: James Hain
 ****************************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <EEPROM.h>

//#define USE_USBCON     // May not need this line, depending on Arduino nano hardware
#include <ros.h>       // ROS Arduino library
#include <rover/ArmCmd.h> // ROS msg for arm commands

// PWM board initialisation
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define LOWERMIN 77     //Defining bounds of movement
#define LOWERMAX 148
#define UPPERMIN 75
#define UPPERMAX 156


#define l1 42.5
#define l2  56



//PWM Pins
const int spinPWMPin = 0;
const int lowerPWMPin = 1;
const int upperPWMPin = 2;
const int rollPWMPin = 3;
const int endPWMPin = 4;

const int azimuthPWMPin = 5;
const int horizonPWMPin = 6;




//Arduino Pins
const int lowerPin = 6;           
const int upperPin = 7;           
const int directionLowerPin = 5;  
const int directionUpperPin = 6;
const int directionSpinPin = 4;
const int directionRollPin = 7;
const int directionEndPin = 8;
const int directionAzimuthPin = 9;
const int directionHorizonPin = 10;


///////////////////////////////////  Variables to change ////////////////////////////

//Joint Angles
int horizonAngle = 90;    // Range: (40,160) 
int azimuthAngle = 60;    // Range: (0,180)   axis not really usable
int lowerAngle = 100;     // Range: (77,148)
int upperAngle = 100;     // Range: (75,156)


//Specific Speeds out of 4095
//Negative number changes direction
int spinSpeed = 0; // 1024 is good
int rollSpeed = 0; // 4095 is good
int endSpeed = 0; // 2048 is good
int azimuthSpeed = 0;
int horizonSpeed = 0;

// General motor speed and linear actuator speed
int speed = 2048;   // out of 4095


////////////////////////////////////////////////////////////////////////////////////
// Flags or 'semaphores' to prevent incrementing angles while actuator moving
bool lowerBusy = 0;
bool upperBusy = 0;

//Variables for angle read from potentiometers
int lowerReadAngle = 0;
int upperReadAngle = 0;

// Potentiometer endpoints
int lowerPotMin;
int lowerPotMax;
int upperPotMin;
int upperPotMax;

const double pi = 3.14159265;

////////////////////////////// ROS CALLBACK ////////////////////////////////////////

// Declare required ROS variables
ros::NodeHandle  nh;

// Callback function to execute on receiving arm command from base station
void msgCallback (const rover::ArmCmd& msg)
{
  int incrm = msg.sensitivity; // Amount to increment by, value 1-5

  int lastBack = 0;
  int back = 0;
  int start = 0;
  int lastStart = 0;
  int coordSystem = 0;
  double x;
  double y;
  
  lastBack = back;
  back = msg.back;

  lastStart = start;
  start = msg.start;

  // Increment each arm DoF according to the arm command

  if (coordSystem == 0)
  {
    if (!lowerBusy)         // Only increment if the actuator has reached the previously wanted position
    {
      lowerAngle   = constrain(lowerAngle   + incrm*msg.shoulder, LOWERMIN, LOWERMAX);
      lowerBusy = 1;        // Lock the semaphore
    }
    if (!upperBusy)         // Only increment if the actuator has reached the previously wanted position
    {
      upperAngle   = constrain(upperAngle   + incrm*msg.forearm,  UPPERMIN, UPPERMAX);
      upperBusy = 1;        // Lock the semaphore
    }
  } else if (coordSystem == 1 && (!lowerBusy && !upperBusy))
  {
    x =  -l1*cos(lowerAngle*(pi/180)) + l2*cos((lowerAngle - upperAngle)*(pi/180)) + incrm*msg.shoulder;
    y = l1*sin(lowerAngle*(pi/180)) - l2*sin((lowerAngle - upperAngle)*(pi/180)) + incrm*msg.forearm;

    lowerAngle = constrain(round( (pi-acos((pow(l1,2)-pow(l2,2)+pow(x,2)+pow(y,2))/(2*l1*sqrt(pow(x,2)+pow(y,2))))-atan(y/x))*(180/pi)), LOWERMIN, LOWERMAX);
    upperAngle = constrain(round( (pi-acos((-pow(l1,2)-pow(l2,2)+pow(x,2)+pow(y,2))/(2*l1*l2)))*(180/pi)),  UPPERMIN, UPPERMAX);
    lowerBusy = 1;
    upperBusy = 1; 
  }

  // wrist speed is a fraction of the speed of 4095, based on sensitivity
  horizonSpeed = (int) (4095.0*((float) msg.wrist_y)*((float) incrm)/5.0);
  azimuthSpeed = (int) (4095.0*((float) msg.wrist_x)*((float) incrm)/5.0);

  // gripper speed is a fraction of the speed of 4095, based on sensitivity
  endSpeed = (int) (4095.0*((float) msg.grip)*((float) incrm)/5.0); 
  
  
  // gripper roll speed is a fraction of the speed of 4095, based on sensitivity
  rollSpeed = (int) (4095.0*((float) msg.twist)*((float) incrm)/5.0); 

  // base spin speed is a fraction of the speed of 1024, based on sensitivity
  spinSpeed = (int) (1024.0*((float) msg.base)*((float) incrm)/5.0); 

  if (lastBack == 0 && back == 1 )
  {
    calibrate();
  }
  if (lastStart == 0 && start == 1)
  {
    coordSystem += 1;
    coordSystem = coordSystem % 2;
  }
  
}

// may need to subscribe to "mainframe/arm_cmd_data" instead if this doesn't work
ros::Subscriber<rover::ArmCmd> arm_sub("arm_cmd_data", &msgCallback);


//////////////////////////////// MAIN CODE /////////////////////////////////////////

// Setup function for pins, serial for debug and PWM
void setup()
{
  //  setup serial
  //Serial.begin(9600);    

  nh.initNode();
  nh.subscribe(arm_sub);

  //Set pins
  pinMode(directionLowerPin, OUTPUT);
  pinMode(directionUpperPin, OUTPUT);
  pinMode(directionSpinPin, OUTPUT);
  pinMode(directionRollPin, OUTPUT);
  pinMode(directionEndPin, OUTPUT);
  pinMode(directionAzimuthPin, OUTPUT);
  pinMode(directionHorizonPin, OUTPUT);

  pwm.begin();
  
  pwm.setPWMFreq(500);  // Higher frequency for smoother linear actuator movement
  yield();

  //Set all PWM's Initially to 0
  for (int i = 0; i<15; i++)
  {
    pwm.setPWM(i, 0, 0);
  }

  lowerPotMin = EEPROMReadlong(0);
  lowerPotMax = EEPROMReadlong(4);
  upperPotMin = EEPROMReadlong(8);
  upperPotMax = EEPROMReadlong(12);
  //calibrate();
}

// Function to decode motor direction, returns 1 if positive number, 0 otherwise
static inline int8_t isPositive(int val) {
 if (val <= 0) return 0;
 return 1;
}

//Function sets pulse width for servo if direct PWM instruction not wanted
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  //Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  //Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  //Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

// Function sets the lower linear actuator position given potentiometer voltage for this actuator's range of movement
void actuatorLower(int angle, int speed)
{
  
  lowerReadAngle = map(analogRead(lowerPin),lowerPotMin,lowerPotMax,148,77);        // Read PWM voltage and scale to joint angle
  

    if (lowerReadAngle > angle)                                     // Extend or retract the actuator accordingly
    {
      pwm.setPWM(lowerPWMPin, 0, speed);
      digitalWrite(directionLowerPin, HIGH);
    }
    else if(lowerReadAngle < angle)
    {
      pwm.setPWM(lowerPWMPin, 0, speed);
      digitalWrite(directionLowerPin, LOW);
    }
    else 
    {
      pwm.setPWM(lowerPWMPin, 0, 0);
      lowerBusy = 0;                                              // release the semaphore
    }
  
    if (angle > LOWERMAX - 5 || angle < LOWERMIN + 5) // prevent actuator from getting stuck at limits of range
    {
      lowerBusy = 0;
    }
    
    //Serial.println("Actuator Lower");
    //Serial.println(lowerReadAngle);
    lowerReadAngle = map(analogRead(lowerPin),lowerPotMin,lowerPotMax,148,77);                                  // Stop movement
  
}

// Function sets the upper linear actuator position given potentiometer voltage for this actuator's range of movement
void actuatorUpper(int angle, int speed)
{
  upperReadAngle = map(analogRead(upperPin),upperPotMin,upperPotMax,156,75);              // Read PWM voltage and scale to joint angle
  


    if (upperReadAngle > angle)                                           // Extend or retract the actuator accordingly
    {
      pwm.setPWM(upperPWMPin, 0, speed);
      digitalWrite(directionUpperPin, HIGH);
    }
    else if(upperReadAngle < angle)
    {
      pwm.setPWM(upperPWMPin, 0, speed);
      digitalWrite(directionUpperPin, LOW);
    }
    else {
      pwm.setPWM(upperPWMPin, 0, 0);
      upperBusy = 0;                                  // release the semaphore
    }
  
    if (angle > UPPERMAX - 5 || angle < UPPERMIN + 5) // prevent actuator from getting stuck at limits of range
    {
      upperBusy = 0;
    }
    
    //Serial.println("Actuator Upper");
    //Serial.println(upperReadAngle);
    
    upperReadAngle = map(analogRead(upperPin),upperPotMin,upperPotMax,156,75);
}

void calibrate() // Function to calibrate the potentiometer position readings for the arms available movement
{
      pwm.setPWM(lowerPWMPin, 0, 4095);
      pwm.setPWM(upperPWMPin, 0, 4095);
      digitalWrite(directionLowerPin, LOW);
      digitalWrite(directionUpperPin, LOW);           // Move to end point and wait
      delay(10000);
      lowerPotMin = analogRead(lowerPin);             // Set values read
      EEPROMWritelong(0, lowerPotMin);
      upperPotMin = analogRead(upperPin);
      EEPROMWritelong(8, upperPotMin);
      digitalWrite(directionUpperPin, HIGH);           // Move to other end point and wait
      digitalWrite(directionLowerPin, HIGH);
      delay(10000);
      lowerPotMax = analogRead(lowerPin);                // Set values read
      EEPROMWritelong(4, lowerPotMax);
      upperPotMax = analogRead(upperPin);
      EEPROMWritelong(12, upperPotMax);
}

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

//This function will return a 4 byte (32bit) long from the eeprom
//at the specified address to address + 3.
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

//////////////////////////////////// Main Software Loop /////////////////////////////////////
void loop()
{
  actuatorLower(lowerAngle,speed);                                          // Set lower actuator position
  
  actuatorUpper(upperAngle,speed);                                          // Set upper actuator position                                
                                                        

  pwm.setPWM(spinPWMPin, 0, abs(spinSpeed));                                // Set Motor PWM
  digitalWrite(directionSpinPin, isPositive(spinSpeed));                    // Set Motor direction

  pwm.setPWM(rollPWMPin,0,abs(rollSpeed));                                  // Set Motor PWM
  digitalWrite(directionRollPin,isPositive(rollSpeed));                     // Set Motor direction

  pwm.setPWM(endPWMPin,0,abs(endSpeed));
  digitalWrite(directionEndPin,isPositive(endSpeed));

  pwm.setPWM(azimuthPWMPin,0,abs(azimuthSpeed));
  digitalWrite(directionAzimuthPin,isPositive(azimuthSpeed));

  pwm.setPWM(horizonPWMPin,0,abs(horizonSpeed));
  digitalWrite(directionHorizonPin,isPositive(horizonSpeed));
    
  nh.spinOnce();
}
