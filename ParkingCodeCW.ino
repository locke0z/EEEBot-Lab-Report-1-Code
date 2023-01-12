float distance = 999;
long duration;
float distancecm;
int leftMotor_speed, rightMotor_speed, servoAngle;

const int trigPin = 25;
const int echoPin = 26;
float angle = 0;

#include <math.h>
#include <Wire.h>
#include <HCSR04.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
int x = 0;

UltraSonicDistanceSensor distanceSensor(25, 26);  // Initialize sensor that uses digital pins 25 and 26.

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop()
{

  //Forward 1 second
  leftMotor_speed = 255;
  rightMotor_speed = 255;
  servoAngle = 82; //82 appears to be the straightest angle
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);
  //Rotate 180 degrees clockwise (left)
  leftMotor_speed = 100; // any lower and the wheels will not spin
  rightMotor_speed = 255;
  servoAngle = 72;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle < 180) { //when going clockwise (left), angle goes positive
    angle = gyroscope();
  }
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);

  //reverse until 10cm away from wall
  leftMotor_speed = -100;
  rightMotor_speed = -100;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultraSonic();
    delay(100);
  }
  while (distance >= 10);
  distance = 999;
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(100);

  //Rotate 90 degrees clockwise (right)
  leftMotor_speed = 255;
  rightMotor_speed = 100; //Any lower and the wheels will not spin
  servoAngle = 92;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  while (angle > 90) { //while going clockwise, the angle goes negative
    angle = gyroscope();
  }
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(1000);
  //reverse untill 10cm from wall
  leftMotor_speed = -100;
  rightMotor_speed = -100;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  do {
    distance = ultraSonic();
    delay(100);
  }
  while (distance >= 10);
  distance = 999;
  //stop
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 82;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(999999999);

}

void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF));
  Wire.endTransmission();   // stop transmitting
}


float gyroscope() //basic gyro code
{
  mpu6050.update();
  return (mpu6050.getAngleZ());
}


float ultraSonic() //longer version of ultraSonic code, works better 
{ 
  float distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration * 0.034 / 2);
}