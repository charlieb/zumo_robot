#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <SoftwareServo.h> 

/*********** REFLECTANCE ARRAY ************/

ZumoReflectanceSensorArray reflectanceSensors;

// Define an array for holding sensor values.
#define NUM_SENSORS 6
unsigned int sensorValues[NUM_SENSORS];

void calibrate_reflectance()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);        // turn on LED to indicate we are in calibration mode
  
  unsigned long startTime = millis();
  while(millis() - startTime < 10000)   // make the calibration take 10 seconds
  {
    reflectanceSensors.calibrate();
  }
  digitalWrite(13, LOW);         // turn off LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(reflectanceSensors.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

int read_reflectance()
{
  // read calibrated sensor values and obtain a measure of the line position.
  // Note: the values returned will be incorrect if the sensors have not been properly
  // calibrated during the calibration phase.
  unsigned int position = reflectanceSensors.readLine(sensorValues);

  // To get raw sensor values instead, call:  
  //reflectanceSensors.read(sensorValues);

  for (byte i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.print("    ");
  Serial.println(position);
  
}

 
/*********** SERVO AND SENSOR ************/
SoftwareServo myservo;  // create servo object to control the IR holder

const int SERVO = A4;
const int IR_SENSOR = A1;

int servo_angle = 90; // variable to read the value from the analog pin 

int dir = 10;
int servo_scan() {
  // read the input on analog pin 4:
  int sensorValue = digitalRead(IR_SENSOR);
  // print out the value you read:
  
  if(sensorValue) {
    if(servo_angle >= 180 || servo_angle <= 0) dir = -dir;
    servo_angle += dir;  // (value between 0 and 180) 
    myservo.write(servo_angle); // sets the servo position according to the scaled value 
  }
  else
  {
    Serial.println(servo_angle);
    return servo_angle;
  }

  return -1;

}

void servo_ping() {
  if(servo_angle == 180) 
    servo_angle = 0;
  else if(servo_angle == 0)
    servo_angle = 180;
  else 
    servo_angle = 0;
  myservo.write(servo_angle); // sets the servo position according to the scaled value 
}
int sense_ping() {
  // read the input on analog pin 4:
  int sensorValue = digitalRead(IR_SENSOR);
  // print out the value you read:
  Serial.println(sensorValue);
  
  if(sensorValue) return false;
  else return true;
}

/************* MOTORS ****************/

ZumoMotors motors;

int left_speed = 0, right_speed = 0;
void run_motors(int left, int right) {
  int dr, dl;
  if(left > left_speed) {
    dl = 1;
  } else {
    dl = -1;
  }
  if(right > right_speed) {
    dr = 1;
  } else {
    dr = -1;
  }
  
  while(dr != 0 || dl != 0) {
    if(right_speed == right) dr = 0;
    if(left_speed == left) dl = 0;
    right_speed += dr;
    left_speed += dl;
    motors.setLeftSpeed(left_speed);
    motors.setRightSpeed(right_speed);
    delay(2);
  }
}



/************* MAIN *****************/

// the setup routine runs once when you press reset:
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  myservo.attach(SERVO);  // attaches the servo on pin A4 to the servo object 

  motors.flipRightMotor(true); // I messed up the build D:

  // Turn on reflectance array but don't use the LED indicator
  reflectanceSensors.init(QTR_NO_EMITTER_PIN);

  calibrate_reflectance();
}

const int NONE = 0;

const int PING = 1;
const int SCAN = 2;

int sense_mode = PING;
unsigned long ping_time = 0;
unsigned long scan_time = 0;
unsigned long reflect_time = 0;

const int FORWARD = 4;
const int BACKWARD = 8;
const int LEFT = 16;
const int RIGHT = 32;
const int STOP = 64;
int move_mode = STOP;


const int REFL_ON = 128;
const int REFL_OFF = 256;
int reflect_mode = REFL_OFF;


// the loop routine runs over and over again forever:
void loop() {
  unsigned long time = millis();
  int got_ping = false;
  int ping_angle = -1;
  int reflect = false;

  SoftwareServo::refresh();

  switch(sense_mode) {
  case PING:
    if(time - ping_time > 250) {
      servo_ping();
      ping_time = time;
    }
    got_ping = sense_ping();
    break;
  case SCAN:
    if(time - scan_time > 100) {
      ping_angle = servo_scan();
      scan_time = time;
    }
    break;
  }

  switch(move_mode) {
    case FORWARD:
      run_motors(100,100);
      break;
    case BACKWARD:
      run_motors(-100,-100);
      break;
    case RIGHT:
      run_motors(100,-100);
      break;
    case LEFT:
      run_motors(-100,100);
      break;
    case STOP:
      run_motors(0,0);
      break;
  }

  switch(reflect_mode) {
  case REFL_OFF:
    break;
  case REFL_ON:
    if(time - reflect_time > 250) {
      read_reflectance();
    }
    break;
  }

  switch(sense_mode + move_mode + reflect_mode) {
    case STOP + PING + REFL_OFF:
      Serial.println("STOP + PING -> FORWARD + PING");
      move_mode = FORWARD;
      break;
    case FORWARD + PING + REFL_OFF:
      if(got_ping) {
        Serial.println("FORWARD + PING -> STOP + SCAN");
        sense_mode = SCAN;
        move_mode = STOP;
        got_ping = false; // Reset variable
      }
      break;
    case STOP + SCAN + REFL_OFF:
      if(ping_angle != -1) {
        move_mode = BACKWARD;
        reflect_mode = REFL_ON;
      }
      break;
    case BACKWARD + SCAN + REFL_ON:
      if(reflect) {
        move_mode = STOP;
        }
        break;
  }
      
}
