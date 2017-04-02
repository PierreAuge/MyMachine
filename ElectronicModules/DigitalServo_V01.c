/*
Digital Servo Motor
Source originally derived from: www.HomoFaciens.de/technics-base-circuits-encoder-disc_en_navion.htm
and https://github.com/danithebest91/ServoStrap

Modified for MyMachine project by: Raymond and Pierre Auge https://github.com/PierreAuge/MyMachine
This code allows an Arduino Mini/ProMini or any other ATMEGA328 AVR chip to function as a digital servo driver module in 
concert with a PWM driven dual H-Bridge DC Motor driver. Compatible with GRBL, Marlin, LinuxCNC, MachineKit or any other digital 
Step and Direction driven stepper system.
*/

#include <util/delay.h>

const int stepPin = 2;
const int dirPin = 1;
// Note that one full rotation of my test motor is 327 steps or a 1.1degree step angle at the output shaft

int stepStatus = 0; //Current status of the stepper motor
int sensorStatusA = 0; // Variable storing the sensor state of the first Optical Sensor
int sensorStatusB = 0; // Variable storing the sensor state of the second Optical Sensor
int count = 0;
int stepDone = 0; //Number of steps completed by the motor
int diffTime = 0;
unsigned long millisStart = 0;
unsigned long millisNow = 0;
unsigned long millisDiff = 0;
int dutyCycle = 0;
signed long setPoint = 0;
signed long actualPoint = 0;
byte readByte = 0;

#define SENSOR_A 3
#define SENSOR_B 4
#define ENABLE_R 8
#define ENABLE_L 9

#define MOTOR_DIRECTION 5
#define MOTOR_PWM 6

#define LED 13             //Status LED of servo

#define P_FRACTION 1.0     //Proportional factor of control loop 0.001 - 10.0 (1.0)
#define I_FRACTION 0.3     //Integral factor of control loop 0.0 - 10.0 (0.3)
#define SOFT_START 0.0005  //0.0 - 1.0 (0.0005)
#define STEP_MARGIN 1L     //10 - 1000 (1)

#define MIN_DUTYCYCLE 30   //0 - 255 (125)
#define MAX_DUTYCYCLE 225  //0 - 255 (255)

bool dir = false;

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('X');     // send a capital X to indicate that Arduino is running
    delay(300);
  }
}


void setup() {
  
  pinMode(stepPin, INPUT);
  pinMode(dirPin, INPUT);
  pinMode(SENSOR_A, INPUT);
  pinMode(SENSOR_B, INPUT);   
  pinMode(LED, OUTPUT);  
  pinMode(ENABLE_R, OUTPUT);  
  pinMode(ENABLE_L, OUTPUT);  
  pinMode(MOTOR_DIRECTION, OUTPUT);  
  pinMode(MOTOR_PWM, OUTPUT);    
  digitalWrite(ENABLE_R, 1);
  digitalWrite(ENABLE_L, 1);
  
  attachInterrupt(1, SensorInput, CHANGE); // encoder pin on interrupt 1 - pin 3
  attachInterrupt(0, countStep, RISING); //on pin 2
   
  // start serial port at 115200 bps:
  Serial.begin(115200);
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop() {    

      diffTime++;
 
      dutyCycle = (double)(abs(setPoint - actualPoint)) * (double)P_FRACTION;
      dutyCycle += (double)(diffTime * I_FRACTION);

      if(dutyCycle < MIN_DUTYCYCLE){
        dutyCycle = MIN_DUTYCYCLE;
      }
      if(dutyCycle > MAX_DUTYCYCLE){
        dutyCycle = MAX_DUTYCYCLE;
      }

      if(SOFT_START * (double)(millisNow - millisStart) < 1.0){
        dutyCycle = (double)(dutyCycle * SOFT_START * (millisNow - millisStart));
      }
      
      if(dutyCycle < MIN_DUTYCYCLE){
        dutyCycle = MIN_DUTYCYCLE;
      }
      if(dutyCycle > MAX_DUTYCYCLE){
        dutyCycle = MAX_DUTYCYCLE;
      }
      if(abs(setPoint - actualPoint) < STEP_MARGIN){
        stepDone = 1;
        diffTime = 0;
        analogWrite(MOTOR_PWM, 0);
        digitalWrite(MOTOR_DIRECTION, 0);
        digitalWrite(ENABLE_R, 0);
        digitalWrite(ENABLE_L, 0);
      }
      else{
        digitalWrite(ENABLE_R, 1);
        digitalWrite(ENABLE_L, 1);
        if(actualPoint < setPoint){
          digitalWrite(MOTOR_DIRECTION, 1);
          analogWrite(MOTOR_PWM, 255 - dutyCycle);
        }
        if(actualPoint > setPoint){
          digitalWrite(MOTOR_DIRECTION, 0);
          analogWrite(MOTOR_PWM, dutyCycle);
        }
      }
          
    millisNow = millis();
    if(millisStart > millisNow){
      millisStart = millisNow;
    }
  

}

void SensorInput(){
      
      sensorStatusA = digitalRead(SENSOR_A);
      sensorStatusB = digitalRead(SENSOR_B);
      
      if(sensorStatusB == 0 && sensorStatusA == 0){
        if(stepStatus == 1){
          actualPoint--;
          diffTime = 0;
        }
        if(stepStatus == 3){
          actualPoint++;
          diffTime = 0;
        }
        stepStatus = 0;
      }
    
      if(sensorStatusB == 1 && sensorStatusA == 0){
        if(stepStatus == 0){
          actualPoint++;
          diffTime = 0;
        }
        if(stepStatus == 2){
          actualPoint--;
          diffTime = 0;
        }
        stepStatus = 1;
      }
    
      if(sensorStatusB == 1 && sensorStatusA == 1){
        if(stepStatus == 3){
          actualPoint--;
          diffTime = 0;
        }
        if(stepStatus == 1){
          actualPoint++;
          diffTime = 0;
        }
        stepStatus = 2;
      }
    
      if(sensorStatusB == 0 && sensorStatusA == 1){
        if(stepStatus == 2){
          actualPoint++;
          diffTime = 0;
        }
        if(stepStatus == 0){
          actualPoint--;
          diffTime = 0;
        }
        stepStatus = 3;
      }

}

void countStep(){
  dir = digitalRead(DIR_PIN);
            if (dir) setPoint++;
            else setPoint--;
  millisStart = millis();
  stepDone = 0;
}
