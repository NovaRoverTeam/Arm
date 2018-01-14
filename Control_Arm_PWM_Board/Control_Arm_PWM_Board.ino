#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)


//PWM Pins
const int spinPWMPin = 0;
const int lowerPWMPin = 1;
const int upperPWMPin = 2;
const int rollPWMPin = 3;
const int endPWMPin = 4;

const int azimuthPWMPin = 5;
const int horizonPWMPin = 6;

const int endservoPWMPin = 7;

//Arduino Pins
const int lowerPin = 6;           
const int upperPin = 7;           
const int directionLowerPin = 5;  
const int directionUpperPin = 6;
const int directionSpinPin = 4;
const int directionRollPin = 7;




//Joint Angles
int horizonAngle = 90;    // Range: (40,160) 
int azimuthAngle = 60;    // Range: (-,-)   axis not really usable
int lowerAngle = 100;     // Range: (77,148)
int upperAngle = 100;     // Range: (75,156)
int endAngle = 120;        // Range: (35,120)

// General motor speed
int speed = 2048;   // out of 4095

//Specific Speeds out of 4095
//Negative number changes direction
int spinSpeed = 0; // 1024 is good
int rollSpeed = 0; // 4095 is good
               


//Angle read from potentiometers
int lowerReadAngle = 0;
int upperReadAngle = 0;


void setup()
{
  //  setup serial
  Serial.begin(9600);             

  //Set pins
  pinMode(directionLowerPin, OUTPUT);
  pinMode(directionUpperPin, OUTPUT);
  pinMode(directionSpinPin, OUTPUT);
  pinMode(directionRollPin, OUTPUT);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();

  //Set all PWM's Initially to 0
  for (int i = 0; i<15; i++)
  {
    pwm.setPWM(i, 0, 0);
  }
}
static inline int8_t isPositive(int val) {
 if (val < 0) return 0;
 if (val==0) return 0;
 return 1;
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void actuatorLower(int angle, int speed)
{
  
  lowerReadAngle = map(analogRead(lowerPin),291,555,148,77);
  


  
  while(lowerReadAngle != angle)
  {
    if (lowerReadAngle > angle)
    {
      pwm.setPWM(lowerPWMPin, 0, speed);
      digitalWrite(directionLowerPin, HIGH);
    }
    else if(lowerReadAngle < angle)
    {
      pwm.setPWM(lowerPWMPin, 0, speed);
      digitalWrite(directionLowerPin, LOW);
    }
    
    
    //Serial.println("Actuator Lower");
    //Serial.println(val1);
    lowerReadAngle = map(analogRead(lowerPin),291,555,148,77);
  }
  pwm.setPWM(lowerPWMPin, 0, 0);
  
}

void actuatorUpper(int angle, int speed)
{
  upperReadAngle = map(analogRead(upperPin),95,406,156,75);
  


  
  while(upperReadAngle != angle)
  {

    if (upperReadAngle > angle)
    {
      pwm.setPWM(upperPWMPin, 0, speed);
      digitalWrite(directionUpperPin, HIGH);
    }
    else if(upperReadAngle < angle)
    {
      pwm.setPWM(upperPWMPin, 0, speed);
      digitalWrite(directionUpperPin, LOW);
    }
    
    //Serial.println("Actuator Upper");
    //Serial.println(upperReadAngle);
    //Serial.println(angle);
    upperReadAngle = map(analogRead(upperPin),95,406,156,75);
  }
  pwm.setPWM(upperPWMPin, 0, 0);
}

void loop()
{
  actuatorLower(lowerAngle,speed);
  
  actuatorUpper(upperAngle,speed);

  pwm.setPWM(azimuthPWMPin, 0, map(azimuthAngle,0,180,150,600));
  delay(15);
  
  pwm.setPWM(horizonPWMPin, 0, map(horizonAngle,0,180,150,600));
  delay(15); 

  pwm.setPWM(endservoPWMPin, 0, map(endAngle,0,180,150,600));
  delay(15); 


  pwm.setPWM(spinPWMPin, 0, abs(spinSpeed));
  digitalWrite(directionSpinPin, isPositive(spinSpeed));

  pwm.setPWM(rollPWMPin,0,abs(rollSpeed));
  digitalWrite(directionRollPin,isPositive(rollSpeed));

  
  
  

  
}
