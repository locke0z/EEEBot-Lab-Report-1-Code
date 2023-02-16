 #define l3 15 //furthest left
 #define l2 2
 #define l1 4
 #define r1 13
 #define r2 12
 #define r3 14 //furthest right

 #include <Wire.h>
 #define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
 int x = 0;

 float error=0;
 float u=0;
 float errorSum=0;

int leftMotor_speed, rightMotor_speed, servoAngle;

 int centreAngle=82;
 int baseSpeed=125;

void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)


}

void loop() {
    // put your main code here, to run repeatedly:

float sumL3High,sumL3Low,averageL3High,averageL3Low, sumL2High,sumL2Low,averageL2High,averageL2Low, sumL1High,sumL1Low,averageL1High,averageL1Low, sumR1High,sumR1Low,averageR1High,averageR1Low, sumR2High,sumR2Low,averageR2High,averageR2Low, sumR3High,sumR3Low,averageR3High,averageR3Low;


int calibrated=0;
//CALIBRATION ALGORITHM
while (calibrated=0)
{
  delay (1000);
  servoAngle=50; //signal to user to place sensors over white space
  Transmit_to_arduino(leftMotor_speed,rightMotor_speed, servoAngle);
  delay (3000);
  
  for (int white=0; white=500; white++)
  {
    sumL3High=sumL3High+analogRead(l3);
    sumL2High=sumL2High+analogRead(l2);
    sumL1High=sumL1High+analogRead(l1);
    sumR1High=sumR1High+analogRead(r1);
    sumR2High=sumR2High+analogRead(r2);
    sumR3High=sumR3High+analogRead(r3);
  }
    averageL3High=sumL3High/500;
    averageL2High=sumL2High/500;
    averageL1High=sumL1High/500;
    averageR1High=sumR1High/500;
    averageR2High=sumR2High/500;
    averageR3High=sumR3High/500;

  servoAngle=82; //signal that white-space calibration is complete
  Transmit_to_arduino(leftMotor_speed,rightMotor_speed, servoAngle);
  delay (2000);

  servoAngle=110; //signal to user to place sensors over black line
  Transmit_to_arduino(leftMotor_speed,rightMotor_speed, servoAngle);
  delay (3000);

  for (int black=0; black=500; black++)
  {
    sumL3Low=sumL3Low+analogRead(l3);
    sumL2Low=sumL2Low+analogRead(l2);
    sumL1Low=sumL1Low+analogRead(l1);
    sumR1Low=sumR1Low+analogRead(r1);
    sumR2Low=sumR2Low+analogRead(r2);
    sumR3Low=sumR3Low+analogRead(r3);
    
  }
    averageL3Low=sumL3Low/500;
    averageL2Low=sumL2Low/500;
    averageL1Low=sumL1Low/500;
    averageR1Low=sumR1Low/500;
    averageR2Low=sumR2Low/500;
    averageR3Low=sumR3Low/500;

  // do the averages to find max and min variable values, plop those into the constrain/map code
constrain(L3,averageL3Low,averageL3High);
L3=map(L3,averageL3Low,averageL3High,0,4095); //maybe in brackets is l3 instead of L3???

constrain(L2,averageL2Low,averageL2High);
L2=map(L2,averageL2Low,averageL2High,0,4095);

constrain(L1,averageL1Low,averageL1High);
L1=map(L1,averageL1Low,averageL1High,0,4095);

constrain(R1,averageR1Low,averageR1High);
R1=map(R1,averageR1Low,averageR1High,0,4095);

constrain(R2,averageR2Low,averageR2High);
R2=map(R2,averageR2Low,averageR2High,0,4095);

constrain(R3,averageR3Low,averageR3High);
R3=map(R3,averageR3Low,averageR3High,0,4095);

  servoAngle=82; //signal that black-line calibration is complete, and place car on track
  Transmit_to_arduino(leftMotor_speed,rightMotor_speed, servoAngle);
  delay 5000;

calibrated=1; //stops the calibration loop

}



//delay (150);
float L3=analogRead(l3);
float L2=analogRead(l2);
float L1=analogRead(l1); //mid 15 bad pin, corner 15 good
float R1=analogRead(r1); //WORKS WELL
float R2=analogRead(r2); //WORKS WELL
float R3=analogRead(r3);

//constrain(L3,low,high);
//L3=map(L3,low,high,0,4095);

//WEIGHTED AVERAGE
//the weights of ea\ch sensor (distance in mm from midpoint of car)
float l1w=9;
float l2w=27;
float l3w=47;
float r1w=-9;
float r2w=-27;
float r3w=-42;

float errorLast=error;
error= ( (L3*l3w)+(L2*l2w)+(L1*l1w)+(R3*r1w)+(R2*r2w)+(R3*r3w) ) / (L3+L2+L1+R1+R2+R3);
errorSum=errorSum+error;

float Kp=5.0; 
float Ki=0.005;
float Kd=0.01; 


//Serial.println(errorSum);

u=(Kp*error)+(Ki*errorSum)+(Kd*(error-errorLast));

  servoAngle=centreAngle+u;

  float k=0.5;
  leftMotor_speed=baseSpeed+(k*u);
  rightMotor_speed=baseSpeed-(k*u);

if ((L3>2500)&&(L2>2500)&&(L1>2500)&&(R3>2500)&&(R2>2500)&&(R1>2500))
{

  if (errorLast<0){ //if line was on left side of sensor array
      servoAngle=60; //reverse at an according angle
  leftMotor_speed=-125;
  rightMotor_speed=-100;
  }

  if (errorLast>0){ //if line was on right side of sensor array
          servoAngle=100; //reverse at an according angle
  leftMotor_speed=-100;
  rightMotor_speed=-125;
  }
  
}

  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);



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
