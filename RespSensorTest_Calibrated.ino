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

unsigned long previousTime = 0;
int seconds = 0;

int state=0;
//const unsigned int NUMBER_OF_MODES = 2;
unsigned int mode=0;



void setup() {
  // declare the ledPin as an OUTPUT:
//  if(++mode >= NUMBER_OF_MODES) mode=0;
  pinMode(ledPin, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
//  Serial.print("Current mode: ");
//  Serial.println(mode);
}

void loop() {
  if(millis()>=(previousTime)){
    previousTime = previousTime+1000;
    seconds++;

//    Serial.println("This many seconds: ");
    Serial.print("t=");
    Serial.println(seconds);
  }
  THRESHOLD = (sensorMin + sensorMax)/2;
  if(mode==0){
    digitalWrite(ledPin, HIGH); // turn on LED to signal the start of the calibration period
  
    // calibrate for 10 seconds
    if(seconds<10){
      sensorValue = analogRead(sensorPin);
      if(sensorValue > sensorMax){
        sensorMax = sensorValue;
      }
      if(sensorValue<sensorMin){
        sensorMin = sensorValue;
      }
//      Serial.println(sensorValue);
    } else {
      mode=1;
      seconds = 0;
    }
  }
  else if(mode==1){
    // signal the end of the calibration period
    digitalWrite(ledPin, LOW);
  
    // start recording breaths for 10 seconds
    if(seconds==30){
      Serial.print("Resp. Rate: ");
      Serial.println(breaths*2);
      breaths = 0;
      seconds = 0;
      mode=0;
    }
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    
    //shitty breath debouncing??
    if((sensorValue<=THRESHOLD) && (shift_reg==0)){
      shift_reg=1;
      breaths++;
      Serial.print("Breath");
    }
    else if((sensorValue>THRESHOLD) && (shift_reg==1)){
      shift_reg=0;
    }
  }
  delay(5); 
}
