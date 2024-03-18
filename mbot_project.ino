#include "MeMCore.h"
#define ADC_REF 5 // reference value to convert voltage read from IR sensor into range 0-5
#define TIMEOUT 2000
#define SPEED_OF_SOUND 340
#define ULTRASONIC 12
// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 //in milliseconds
// Define time delay before taking another LDR reading
#define LDRWait 10 //in milliseconds
 
MeLineFollower lineFinder(PORT_2);
MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
MeBuzzer buzzer;
//floats to hold colour arrays
float blackArray[3] = {452, 694 ,657 };
float greyDiff[3] = {384, 194, 216};
float colourArray[] = {0, 0, 0};

int red[6] = {190,240,50,130,30,140}; // RGB range for red color
int green[6] = {30,120,130,210,70,170}; //RGB range for green color
int orange[6] = {200,255,130,240,80,170}; //RGB range for orange color
int purple[6] = {150,210,120,220,160,230}; //RGB range for purple color
int blue[6] = {50,150,130,240,190,255}; // RGB range for blue color
int white[6] = {200,255,200,255,200,255}; //RGB range for white color

uint8_t baseSpeed = 240; // the speed of the robot when following wall on the left
uint8_t baseSpeed_ir = 240; // the speed of the robot when following wall on the right
int DISTANCE = 13; // target distance when following wall on the left
float DISTANCE_IR = 350; // target voltage when following wall on the right

//Walls in Left: PID values for ultrasonic sensor
float Kp = 3;
float Ki = 0.03;
float Kd = 1;

//PID terms for ultrasonic sensor
float error = 0;
float correction = 0;
float integral = 0;
float derivative = 0;
float lastError = 0;

//Obstacle in Right: PID values for IR proximity sensor
float Kp_ir = 4;
float Ki_ir = 0.05;
float Kd_ir = 1;

//PID terms for IR proximity sensor
float error_ir = 0;
float correction_ir = 0;
float integral_ir = 0;
float derivative_ir = 0;
float lastError_ir = 0;


uint8_t speedL = 0;
uint8_t speedR = 0;
long straightLineBegin = 0;
long leftTurnBegin = 0;
long rightTurnBegin = 0;
long CurrDist = 0;
long CurrDist_ir = 0;
bool isWhite = 0;

void setup() {
  delay(2000);
  pinMode(A0,INPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  onEmitter(); // turn on IR emitter;
}

void forward_in_seconds(long speedL, long speedR, long seconds) { // Code for going straight in a time duration
  leftMotor.run(-speedL);
  rightMotor.run(speedR);
  delay(seconds);
  stopMotor();
}

void forward(uint8_t speedL, uint8_t speedR) { // Code for going straight 
  leftMotor.run(-speedL);
  rightMotor.run(speedR);
}

void turnLeft(uint8_t speedL, uint8_t speedR,long t) { //Code for turning left
  leftMotor.run(speedL);
  rightMotor.run(speedR);
  delay(t);
  stopMotor();
}

void turnRight(uint8_t speedL, uint8_t speedR, long t) { //Code for turning right
  leftMotor.run(-speedL);
  rightMotor.run(-speedR);
  delay(t);
  stopMotor();
}

void doubleLeftTurn() {// Code for double left turn
  turnLeft(190,190,390);
  forward_in_seconds(230,230,630);
  delay(200);
  turnLeft(0,200,720);
  forward_in_seconds(230,230,100);
}

void doubleRightTurn() {// Code for double right turn
  turnRight(190,190,400);
  forward_in_seconds(230,230,735);
  delay(200);
  turnRight(200,0,700);
  forward_in_seconds(230,230,100);
}

void uTurn() {// Code for u-turn
  turnRight(210,210,350);
  stopMotor();
  turnRight(210,210,345);
  forward_in_seconds(190,190,100);
}

void onEmitter(){ // Code for turning on IR emitter
  digitalWrite(A2,LOW);
  digitalWrite(A3,LOW);
  delay(3000);
}

void stopMotor() {// Code for stopping motor
  leftMotor.stop();
  rightMotor.stop();
}

void PID_ultrasonic() { //PID controller for ultrasonic sensor
  CurrDist = getDistance(); // measure distance between the robot and the left walls
  if (CurrDist == 0 || CurrDist >= 20) {
     PID_IR();  // using PID controller for IR proximity sensor when there are no walls on the left 
   
  } else {
    error = CurrDist - DISTANCE;
    integral = error + integral;
    derivative = error - lastError;
    correction = Kp * error + Kd * derivative + Ki * integral;
    lastError = error;
    if (error <= 1) {
      if (straightLineBegin == 0) { //just in the beginning of straight line, set the variables to zero
        initialize();
      }
       follow_wall();
       leftTurnBegin = 0;
       straightLineBegin = 1;  
    } else {
      if (leftTurnBegin == 0) { // just in the beginning of left turn, set the variables to zero
        initialize();
      }
      reach_distance();
      leftTurnBegin = 1;
      straightLineBegin = 0;
    }
  }
}

void initialize() { //initialize variables of PID controller for ultrasonic sensor to zero
  integral = 0;
  derivative = 0;
  lastError = 0;
}

void reach_distance() {// Code for making the robot turning back to the target position when travelling too far from the left side

  int motorSpeed = 7 * error + 1 * derivative;
  if (motorSpeed > 35 && motorSpeed > 0) {
    motorSpeed = 35;
  }
  if (motorSpeed < -35 && motorSpeed < 0) {
    motorSpeed = -35;
  }
  //adjust the left and right motors' speed
  speedL = baseSpeed + motorSpeed;
  speedR = baseSpeed - motorSpeed;  
  forward(speedL,speedR);

}

void follow_wall() {// Code for following the left walls using ultrasonic sensor when the error is less than 2
  if (correction > 35 && correction > 0) {
    correction = 35;
  }
  if (correction < -35 && correction < 0) {
    correction = -35;
  }
  //adjust the left and right motors' speed
  speedL = baseSpeed + correction;
  speedR = baseSpeed - correction; 
  forward(speedL,speedR);
  
}

long getDistance() {// Measure the distance between the robot and the left walls using ultrasonic sensor
  pinMode(ULTRASONIC, OUTPUT); 
  digitalWrite(ULTRASONIC, LOW); 
  delayMicroseconds(2); 
  digitalWrite(ULTRASONIC, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(ULTRASONIC, LOW); 
   
  pinMode(ULTRASONIC, INPUT); 
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  return (duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100);
}

void PID_IR() { //PID controller for IR sensor
  error_ir = CurrDist_ir - DISTANCE_IR;
  integral_ir = error_ir + integral_ir;
  derivative_ir = error_ir - lastError_ir;
  correction_ir = Kp_ir * error_ir + Kd_ir * derivative_ir + Ki_ir * integral_ir;
  lastError_ir = error_ir;
  if (error_ir <= 3) {
    if (straightLineBegin == 0) {//just in the beginning of straight line, set the variables to zero
      initialize_ir();
    }
     follow_wall_ir();
     leftTurnBegin = 0;
     straightLineBegin = 1;  
  } else {
    if (rightTurnBegin == 0) {//just in the beginning of right turn, set the variables to zero
      initialize_ir();
    }
    reach_distance_ir();
    rightTurnBegin = 1;
    straightLineBegin = 0;
    }
}
void initialize_ir() {//initialize variables of PID controller for IR proximity sensor to zero
  integral_ir = 0;
  derivative_ir = 0;
  lastError_ir = 0;
}
void reach_distance_ir() {// Code for making the robot turning back to the target position when travelling too far from the right side

  int motorSpeed = 10 * error_ir + 1 * derivative_ir;
  if (motorSpeed > 127 && motorSpeed > 0) {
    motorSpeed = 127;
  }
  if (motorSpeed < -127 && motorSpeed < 0) {
    motorSpeed = -127;
  }
  //adjust the left and right motors' speed
  speedL = baseSpeed_ir + motorSpeed;
  speedR = baseSpeed_ir - motorSpeed;
  forward(speedL,speedR);
}

void follow_wall_ir() {// Code for following the right walls using IR proximity sensor when the error is less than 3
  if (correction_ir > 35 && correction_ir > 0) {
    correction_ir = 35;
  }
  if (correction_ir < -35 && correction_ir < 0) {
    correction_ir = -35;
  }
  //adjust the left and right motors' speed
  speedL = baseSpeed_ir + correction_ir;
  speedR = baseSpeed_ir - correction_ir;
  forward(speedL,speedR);
}

float getVoltage() {// Measure the voltage that estimates the distance between the robot and the right walls using IR proximity sensor
  float sensor_value;
   float sum = 0;
  for (int i = 0 ; i < 20 ; i += 1)
  {
    sensor_value = analogRead(A0);
    sum += sensor_value;
  }  
  sensor_value = sum / 20;
  float volt ;
  volt = (float)sensor_value*ADC_REF/1024;
  return volt; 
}

void shineIR() {// Code for turning on the IR emitter only
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void shineRed() {// Code for turning on the red LED only
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
}

void shineGreen() {// Code for turning on the green LED only}
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
}

void shineBlue() {// Code for turning on the blue LED only}
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
}


int getAvgReading(int times) {//find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(A1);
    total += reading;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}

int detectColour()  //Code for shining Red, Green, Blue leds to detect colour papers
{
  // Shine Red, read LDR after some delay
  shineRed();
  delay(200);
  colourArray[0] = getAvgReading(5);
  colourArray[0] = (colourArray[0] - blackArray[0]) / (greyDiff[0]) * 255;
  delay(RGBWait);

  // Shine Green, read LDR after some delay
  shineGreen();
  delay(RGBWait);
  colourArray[1] = getAvgReading(5);
  colourArray[1] = (colourArray[1] - blackArray[1]) / (greyDiff[1]) * 255;
  delay(RGBWait);

  // Shine Blue, read LDR after some delay
  shineBlue();
  delay(RGBWait);
  colourArray[2] = getAvgReading(5);
  colourArray[2] = (colourArray[2] - blackArray[2]) / (greyDiff[2]) * 255;
  delay(RGBWait);

  // Run algorithm for colour decoding
  identifyColour(colourArray);
}

void identifyColour(float *colourArray) { //code for indentify colours
  // red
  if (colourArray[0] >= red[0] && colourArray[1] >= red[2] && colourArray[1] <= red[3] && 
      colourArray[2] >= red[4] && colourArray[2] <= red[5]) {
    turnLeft(210,210,365);
    forward_in_seconds(190,190,300);
  }
  // green
  else if (colourArray[0] <= green[1] && colourArray[1] >= green[2] && colourArray[2] >= green[4] && colourArray[2] <= green[5]) {
    turnRight(200,200,390);
    forward_in_seconds(190,190,100);
  }
  // orange
  else if (colourArray[0] >= orange[0] && colourArray[1] <= orange[3] && colourArray[1] >= orange[2] && colourArray[2] >= orange[4] && colourArray[2] <= orange[5]) {
    uTurn();
  }
  // purple
  else if (colourArray[0] >= purple[0] && colourArray[0] <= purple[1] && colourArray[1] >= purple[2] && 
            colourArray[1] <= purple[3] && colourArray[2] <= purple[5] && colourArray[2] >= purple[4]) {
    doubleLeftTurn();
  }
  // light blue
  else if (colourArray[0] <= blue[1] && colourArray[1] >= blue[2] && colourArray[1] <= blue[3] && colourArray[2] >= blue[4] && colourArray[2] <= blue[5]) {
    doubleRightTurn();
  }
  // white
  else if (colourArray[0] >= white[0] && colourArray[1] >= white[1] && colourArray[2] >= white[2]) {
    celebrate();
    isWhite = 1;
  }
}

void celebrate() {// Code for playing celebratory tune
  buzzer.tone(262, 100);
  buzzer.tone(262, 100);
  buzzer.tone(392, 100);
  buzzer.tone(392, 100);
  buzzer.tone(440, 100);
  buzzer.tone(440, 100);
  buzzer.tone(392, 100);
  buzzer.tone(392, 100);
  buzzer.tone(349, 100);
  buzzer.tone(349, 100);
  buzzer.tone(330, 100);
  buzzer.tone(330, 100);
  buzzer.tone(294, 100);
  buzzer.tone(294, 100);
  buzzer.tone(262, 100);
  buzzer.noTone(); 
}

void loop() {

 int sensorState = lineFinder.readSensors(); // read the line sensor's state
  if (sensorState == S1_IN_S2_IN || sensorState == S1_IN_S2_OUT || sensorState == S1_OUT_S2_IN ) { // There is a black line below the line sensor
    stopMotor();
    delay(100);
    if (!isWhite) forward_in_seconds(-190,-190,100); //move backwards a little to make the robot less likely to crash into walls when turning. 
                                                     //isWhite variable is used to check whether the color paper below the robot is white. If yes, 
                                                     //the robot will not move backwards again.
    detectColour();
    shineIR(); // turn on IR emitter after detecting color
  } else { // There is no black line below the line sensor
      isWhite = 0;
      PID_ultrasonic(); 
  } 
}
