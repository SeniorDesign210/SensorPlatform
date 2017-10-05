// Variable resistance stretch cord testing code! Made Jan 29 by Erin Gee for Instructables

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 11;      // select the pin for the LED
int onboardLED = 13;
int sensorValue = 0;  // variable to store the value coming from the sensor
byte breaths = 0; 
int fadeValue = 0;
int scaleValue = 1;  //This scales the input into something the LED can handle
int sensorMin = 1000; // minimum sensor value (initial value set low)
int sensorMax = 0; // maximum sensor value (initial value set high)

byte shift_reg = 0;

int THRESHOLD = (sensorMin + sensorMax)/2; //this is our MINIMUM resistance for a breath
//note that resistance drops when the sensor is flexed

unsigned long previousTime = 30000;
byte seconds = 0;

int state=0;
const unsigned int NUMBER_OF_MODES = 2;
__attribute__((section(".noinit"))) unsigned int mode;



void setup() {
  // declare the ledPin as an OUTPUT:
  if(++mode >= NUMBER_OF_MODES) mode=0;
  pinMode(ledPin, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.print("Current mode: ");
  Serial.println(mode);
}

void loop() {
  if(mode==0){
    digitalWrite(ledPin, HIGH); // turn on LED to signal the start of the calibration period
  
    // calibrate during the first 30 seconds
    while (millis() < 30000) {
    sensorValue = analogRead(sensorPin);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

  // signal the end of the calibration period
  digitalWrite(ledPin, LOW);
  THRESHOLD = (sensorMin + sensorMax)/2;

    // start recording breaths for 10 seconds
    if(millis()>=(previousTime)){
      previousTime = previousTime+1000;
      seconds++;
    }
    if(seconds==11){
      Serial.print("Resp. Rate: ");
      Serial.println(breaths*6);
      mode=1;
    }
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    
    //shitty breath debouncing??
    if((sensorValue<=THRESHOLD) && (shift_reg==0)){
      shift_reg=1;
      breaths++;
    }
    else if((sensorValue>THRESHOLD) && (shift_reg==1)){
      shift_reg=0;
    }
    // divide it into a value from 0-255
    fadeValue = sensorValue/scaleValue;
    //write these values to Serial Window
    Serial.println(fadeValue);
    //write to LED 
    analogWrite(ledPin, fadeValue);
    digitalWrite(onboardLED,0);
    delay(5);
    
  }
  else {
    analogWrite(ledPin, 0);
    digitalWrite(onboardLED,1);
  }               
}
