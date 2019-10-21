#include <stdio.h>
#include <stdlib.h>
#include <SparkFun_MAG3110.h>

int trigPin = 6;
int echoPin = 7;
long duration, cm;

MAG3110 mag = MAG3110(); //Instantiate MAG3110
int forward = NULL;
int backward = NULL;
int thresh;
int bearing;
int isForward;

int left_motor;
int right_motor;
int speed_amount;
int base_speed;
int speed_increase;

#define E1 5  // Enable Pin for motor 1
#define E2 11  // Enable Pin for motor 2
 
#define I1 3  // Control pin 1 for motor 1
#define I2 2  // Control pin 2 for motor 1
#define I3 4  // Control pin 1 for motor 2
#define I4 8  // Control pin 2 for motor 2

/** Senses the wall using the ultrasonic sensor
 *  @ouput distance in cm, saved in cm variable
 */
void sonar() {
  // The sensor sends a pulse by making trigger high then low.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo signal from the sensor from the echo pin
  // duration is time in microseconds
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Converts the time into distance 
  // divide 2 as echo, divide by 29.1 = (1 / (343 * 10^-2)) as speed of sound
  cm = ( duration / 2 ) / 29.1;
  return;
}

// Calibrates the magnometer
void calibration() {
  while (!mag.isCalibrated()) 
  {
    if(!mag.isCalibrating()) //And we're not currently calibrating
    {
      //Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else
    {
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling mag.exitCalMode();
      mag.calibrate(); 
      //Serial.println("Calibrating!");
    }
    delay(100);
  }
  //Serial.println("Calibrated!!");
  //Serial.print(mag.readOffset(mag.x_scale));
  //Serial.print(mag.readOffset(mag.y_scale));
  // set calibration data using mag.setOffset(axis, offset);
  digitalWrite(13, HIGH); // led turns on
  delay(5000); // Wait five seconds
  
  // gets the forward and backwards directions
  bearing = mag.readHeading();
  forward = bearing;
  if (forward > 0) {
    // forward is positive
    backward = forward - 180;
  } else {
    backward = forward + 180;
  }
}

void setup() {
  //Serial Port begin
  Serial.begin (9600);

  // Setup I2C bus for magnometer
  Wire.begin();             //setup I2C bus
  Wire.setClock(400000);    // I2C fast mode, 400kHz

  digitalWrite(13, HIGH); // led turns on
  delay(1000);
  digitalWrite(13, LOW); // led turns off
  delay(1000);

  pinMode(13, OUTPUT);
  
  //Ultra sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  analogWrite(E1, 0);
  analogWrite(E2, 0);
  delay(100);

  // Motor
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // magnometer 
  mag.initialize(); //Initialize the MAG3110
  thresh = 10;
  isForward = 1;
  
  // calibrate mag
  calibration();
}

// Move one motor backwards and one forward, led blinks while turning
void turning() {
  // Run the motors, given the motor speed
  int turning_time = 8; // 200ms * turning_time

  for (int i = 0; i < turning_time; i++) {
    
    //analogWrite(E1, right_motor); // Run in half speed 130 stal
    digitalWrite(E1, HIGH);
    //analogWrite(E2, left_motor); // Run in full speed
    digitalWrite(E2, HIGH);

  
    digitalWrite(13, HIGH); // led turns on
    
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    delay(200);
    
    digitalWrite(13, LOW); // led turns off
    bearing = mag.readHeading();
    if (thresh > abs(backward - bearing) || (360 - thresh) < abs(backward - bearing)) {
      Serial.println("Turned");
      break;
    }
  }
}

void driveMotor() {
  Serial.println(" Forward");
  // Run the motors, given the motor speed
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, HIGH);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, HIGH);
  
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);
  delay(200);
  
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, LOW);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, LOW);
  delay(1000);

}

void driveLeft() {
  left_motor = 220;
  right_motor = 220;
  // Run the motors, given the motor speed
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, HIGH);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, HIGH);
  
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
  delay(200);
  
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, LOW);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, LOW);
  delay(1000);

}

void driveRight() {
  left_motor = 220;
  right_motor = 220;
  // Run the motors, given the motor speed
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, HIGH);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, HIGH);
  
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);
  delay(200);
  
  //analogWrite(E1, right_motor); // Run in half speed 130 stal
  digitalWrite(E1, LOW);
  //analogWrite(E2, left_motor); // Run in full speed
  digitalWrite(E2, LOW);
  delay(1000);

}

void loop() {
  // Ultra sonic sonar detects the wall, outputs to cm
  sonar();

  // print ultra sonic sensor
  
  Serial.print("duration: ");
  Serial.print(duration);
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("cm");
  

  // Detects wall, in cm
  if (cm < 55) {
    if (!isForward) {
      // Stops when going back and finding a wall
      Serial.println("STOP!");
      digitalWrite(13, LOW); // led turns off
      delay(1000);
      digitalWrite(13, HIGH); // led turns off
      delay(1000);
      digitalWrite(13, LOW); // led turns off
      while (1);
    }
    Serial.print("\tWall detected!");
    isForward = 0;
    // Perform turning
    turning();
  }

  // print magnometer
  int x, y, z;
  
  mag.readMag(&x, &y, &z);
  /*
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Z: ");
  Serial.print(z);

  Serial.print(" Heading: ");
  */
  bearing = mag.readHeading();
  //Serial.println(bearing);

  // default motor value
  left_motor = 220;
  right_motor = 220;

  // Detects if the boat is not straight
  if (isForward && thresh <= abs(forward - bearing) && (360 - thresh) >= abs(forward - bearing)) {
    // Run PID to adjust
    //Serial.print(forward);
    
    speed_amount = abs(forward) - abs(bearing);
    // set the motor speed
    if ( 0 < (forward - bearing) && (forward - bearing) < 180 || (forward - bearing) < -180) {
      // Assume drift to left
      //right_motor = 0;
      //Serial.println(" Left");
      driveLeft();
    } else {
      //left_motor = 0;
      //Serial.println(" Right");
      driveRight();
    }
  }

  if (!isForward && thresh <= abs(backward - bearing) && (360 - thresh) >= abs(backward - bearing)) {
    // Run PID to adjust
    //Serial.print(backward);
    //Serial.println(" Backward");
    speed_amount = abs(forward) - abs(bearing);
    if ( 0 < (backward - bearing) && (backward - bearing) < 180 || (backward - bearing) < -180) {
      // Assume drift to left
      //left_motor = 0;
      driveRight();
    } else {
      //right_motor = 0;
      driveLeft();
    }
  }
  driveMotor();
  
}

// References: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
// https://learn.sparkfun.com/tutorials/mag3110-magnetometer-hookup-guide
