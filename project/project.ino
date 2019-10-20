#include <stdio.h>
#include <stdlib.h>
#include <SparkFun_MAG3110.h>

int trigPin = 2;
int echoPin = 3;
long duration, cm;

MAG3110 mag = MAG3110(); //Instantiate MAG3110
int forward = NULL;
int backward = NULL;
int thresh;
int bearing;
int isForward;

#define E1 5  // Enable Pin for motor 1
#define E2 11  // Enable Pin for motor 2
 
#define I1 2  // Control pin 1 for motor 1
#define I2 3  // Control pin 2 for motor 1
#define I3 8  // Control pin 1 for motor 2
#define I4 9  // Control pin 2 for motor 2

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

void calibrate() {
  while (!mag.isCalibrated()) {
    if(!mag.isCalibrating()) //And we're not currently calibrating
    {
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else
    {
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling mag.exitCalMode();
      mag.calibrate(); 
      Serial.println("Calibrating!");
    }
  }
  bearing = mag.readHeading();
  Serial.println("Calibrated!!");
  Serial.print(mag.readOffset(mag.x_scale));
  Serial.print(mag.readOffset(mag.y_scale));
  // set calibration data using mag.setOffset(axis, offset);
  
  // gets the backwards directions
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
  //Ultra sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

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
  calibrate();
}

// Move one motor backwards and one forward
void turning() {
  // Run the motors, given the motor speed
  analogWrite(E1, 255); // Run in half speed
  //analogWrite(E2, 255); // Run in full speed

  for (int i=0; i<255 ; i++)
  {    
    analogWrite(E1, i);
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    delay(100);
  }
  analogWrite(E1, 0);
  //analogWrite(E2, 0);
  delay(2000);
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
  if (cm < 50) {
    if (!isForward) {
      // Stops when going back and finding a wall
      while (1);
    }
    Serial.print("\tWall detected!");
    isForward = 0;
    // Perform turning
    turning();
  }
  Serial.println();

  // print magnometer
  int x, y, z, motor;
  
  mag.readMag(&x, &y, &z);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(", Y: ");
  Serial.print(y);
  Serial.print(", Z: ");
  Serial.print(z);

  Serial.print(" Heading: ");
  Serial.println(mag.readHeading());
  bearing = mag.readHeading();

  if (isForward && thresh <= abs(forward - bearing) && (360 - thresh) >= abs(forward - bearing)) {
    // Run PID to adjust
    Serial.print(forward);
    Serial.println(" Forward");
    // set the motor speed
    
  }

  if (!isForward && thresh <= abs(forward - bearing) && (360 - thresh) >= abs(forward - bearing)) {
    // Run PID to adjust
    Serial.print(backward);
    Serial.println(" Backward");
  }

  // Run the motors, given the motor speed
  analogWrite(E1, 255); // Run in half speed
  //analogWrite(E2, 255); // Run in full speed

  for (int i=0; i<255 ; i++)
  {    
    analogWrite(E1, i);
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    delay(100);
  }
  analogWrite(E1, 0);
  //analogWrite(E2, 0);
  delay(2000);
  
  delay(250);
}

/*
    analogWrite(E1, 255); // Run in half speed
    //analogWrite(E2, 255); // Run in full speed
 
    for (int i=0; i<255 ; i++)
    {    
    analogWrite(E1, i);
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    delay(100);
    }
     analogWrite(E1, 0);
  //  analogWrite(E2, 0);
    delay(2000);
*/





// Reference: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
// https://learn.sparkfun.com/tutorials/mag3110-magnetometer-hookup-guide
