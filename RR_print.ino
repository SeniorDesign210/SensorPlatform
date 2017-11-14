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

unsigned long previousTime = 0;
int seconds = 0;

int state=0;
unsigned int mode=0;


void setup() {
  Serial.begin(115200);
}

void loop() {
    sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue);
}
