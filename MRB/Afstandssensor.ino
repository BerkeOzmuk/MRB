#include "Adafruit_VL53L0X.h"
#define BUTTON_MINUS_PIN 22
#define BUTTON_PLUS_PIN 24
#define WINDOW_SIZE 5

int i = 0;
int value = 0;
int sum = 0;
int valuesList[WINDOW_SIZE];
int avg = 0;

//define lox
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// defines pins numbers
const int trigPin = 22;
const int echoPin = 24;
// defines variables
long duration;
int distance;
//define PID'
int dt = 0.01;
double kp = 2.6;
double ki = 600;
double kd = 1;
//define setpoint
double setPoint = 20;
//define errors
int error = 0;
int error_sum = 0;
int error_div = 0;
int prev_error = 0; 
//define fan pins
const int fanControlPin = 9;
const int fanPulsePin = 2;
//define pulseDuration
unsigned long pulseDuration;

void setup() {
  pinMode(BUTTON_MINUS_PIN, INPUT);
  pinMode(BUTTON_PLUS_PIN, INPUT);
  pinMode(fanControlPin, OUTPUT);
  Serial.begin(115200); // Starts the serial communication

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void movingAverageFilter() { //Deze functie gooit een filter op de laser/afstand sensor en laat op de serialPlotter de grafiek zien met de normale filter en de grafiek met de movingAverage filter.
  sum -= valuesList[i];       // Remove the oldest entry from the sum
  value = lox.readRange();        // Read the next sensor value
  valuesList[i] = value;           // Add the newest reading to the window
  sum += value;                 // Add the newest reading to the sum
  i = (i+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  avg = sum / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

  Serial.print("Value: ");
  Serial.println(value);
  Serial.print("Average: ");
  Serial.println(avg);
  
  delay(25); 
}

void readPulse() { //Deze functie is voor het uitlezen van de rpm van de fan
  pulseDuration = pulseIn(fanPulsePin, LOW);
  double frequency = 1000000/pulseDuration;

  
  Serial.print("pulse duration:");
  Serial.println(pulseDuration);

  Serial.print("time for full rev. (microsec.):");
  Serial.println(pulseDuration*2);
  Serial.print("freq. (Hz):");
  Serial.println(frequency/2);
  Serial.print("RPM:");
  Serial.println(frequency/2*60);
  
} 

void loop() {
  byte buttonMinusState = digitalRead(BUTTON_MINUS_PIN);
  byte buttonPlusState = digitalRead(BUTTON_PLUS_PIN);

  if(buttonMinusState == LOW){
    if(setPoint >= 21){
      setPoint -= 1;
    }
  }

  if(buttonPlusState == LOW){
    if(setPoint <= 30){
      setPoint += 1;
    }
  }

  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(lox.readRange() / 10);
  } else {
    Serial.println(" out of range ");
  }

  int distance = lox.readRange() / 10;
  error = setPoint - distance;
  error_div = (error - prev_error) / dt;

  
  double P = kp * error ;
  double I = ki * error_sum;
  double D = kd * error_div;

  double stuurActie = P + I + D;
  prev_error = error;
  error_sum = error_sum + error * dt;


  double speed = stuurActie + 120;


  if(speed <= 0){
    speed = 0;
  }
  if(speed >= 120){
    speed = 120;
  }
   
  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("PID: ");
  Serial.println(stuurActie);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Setpoint: ");
  Serial.println(setPoint);

  analogWrite(fanControlPin, speed);

  // movingAverageFilter();


}
