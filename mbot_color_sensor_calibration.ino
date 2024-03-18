#include <MeMCore.h>
// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 // in milliseconds
//floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float whiteArray[] =  {0, 0, 0};
float blackArray[] = {0, 0, 0};
float greyDiff[] = {0, 0, 0};

void shineIR() {// Code for turning on the IR emitter only
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void shineRed() {// Code for turning on the red LED only
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
}

void shineGreen() {// Code for turning on the green LED only
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
}

void shineBlue() {// Code for turning on the blue LED only
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
}

void calibration(){
  //set white balance
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);           //delay for five seconds for getting sample ready
  //scan the white sample.
  //go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  // scan with red led
  shineRed();
  delay(RGBWait);
  whiteArray[0] = getAvgReading(5);
  delay(RGBWait);

  //scan with green led
  shineGreen();
  delay(RGBWait);
  whiteArray[1] = getAvgReading(5);
  delay(RGBWait);

  //scan with blue led
  shineBlue();
  delay(RGBWait);
  whiteArray[2] = getAvgReading(5);
  delay(RGBWait);
  //done scanning white, time for the black sample.

  //set black balance
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);     //delay for five seconds for getting sample ready 
  //go through one colour at a time, set the minimum reading for red, green and blue to the black array
  
  //scan with red led
  shineRed();
  delay(RGBWait);
  blackArray[0] = getAvgReading(5);
  delay(RGBWait);
  greyDiff[0] = whiteArray[0] - blackArray[0];

  //scan with green led
  shineGreen();
  delay(RGBWait);
  blackArray[1] = getAvgReading(5);
  delay(RGBWait);
  greyDiff[1] = whiteArray[1] - blackArray[1];

  //scan with blue led
  shineBlue();
  delay(RGBWait);
  blackArray[2] = getAvgReading(5);
  delay(RGBWait);
  greyDiff[2] = whiteArray[2] - blackArray[2];
}

int getAvgReading(int times) { //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(A1);
    total += reading;
    delay(1000);
  }
  //calculate the average and return it
  return total / times;
}

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  Serial.begin(9600);
  calibration();
}

void loop() {
  //print calibration results
  for (int i = 0 ; i <= 2 ; i += 1) {
   Serial.print("color ");
   Serial.print(i);
   Serial.print(" ");
   Serial.print(blackArray[i]);
   Serial.print(" ");
   Serial.println(greyDiff[i]);
   delay(500);
  }
}
