#include <XBee.h>
#include <Printers.h>

#include <Comm.h>

/*****************************************************************
*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

//Comm xbee = Comm(2,3);

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 1 // 250 ms between prints
#define HR_SAMPLE_TIME 15000// 15 s
static unsigned long lastPrint = 0; // Keep track of print time
static unsigned long lastRead_HR = 0; //keep track of HR sample time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.45 // Declination (degrees) in Newark, DE

int mode;
bool lock;
int beats;
XBee xbee = XBee();

void setup() 
{
  
//  Serial.begin(115200);
  Serial.begin(9600);
  xbee.setSerial(Serial);

  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  mode = 0;
  lock = 0;
  beats = 0;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    
//    while (1){
//      xbee.sendERR(1);
//    }
  }
  Serial.println("IMU is setup");
  pinMode(10, INPUT);
}


void loop()
{
  bool hr_in = digitalRead(10);
  if(hr_in==0 and lock==0){
//    Serial.println("beat");
    beats+=1.0;
    lock=1;
  }
  else if(hr_in==1 and lock==1){
    lock=0;
  }
  //currently doing the sampling on here
  if((lastRead_HR + HR_SAMPLE_TIME) < millis()){ // enough beat samples, multiply and send
    float BPM = beats*(60/(HR_SAMPLE_TIME/1000));
//    Serial.print("HR:");
//    Serial.print(beats*(60/(HR_SAMPLE_TIME/1000)));
//    Serial.print("\n");
//    xbee.sendHR(beats*(60/(HR_SAMPLE_TIME/1000)) ); // 60s / (HR_SAMPLE_TIME s/sample) = samples multiplier
    beats = 0;
    lastRead_HR = millis();
  }
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == "TX_TRANSMIT_STATUS") {
      xbee.getResponse();
    }
  }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro();
    printAccel();
    printMag();
    if(mode==0) { //send heart rate
//      xbee.sendHR(46.0);
      mode++; //increment mode
    }
    else if(mode==1){ //send IMU data, using mode to switch between the different data points
        //this looks like a lot, but it really isn't. Find documentation on github about this.
        //https://github.com/SeniorDesign210/SensorPlatform/wiki/Communications-System
        uint8_t payload[37];
        float gx = imu.calcGyro(imu.gx);
        float gy = imu.calcGyro(imu.gy);
        float gz = imu.calcGyro(imu.gz);
        byte *b_gx = (byte *)&gx;
        byte *b_gy = (byte *)&gy;
        byte *b_gz = (byte *)&gz;
        float ax = imu.calcAccel(imu.ax);
        float ay = imu.calcAccel(imu.ay);
        float az = imu.calcAccel(imu.az);
        byte *b_ax = (byte *)&ax;
        byte *b_ay = (byte *)&ay;
        byte *b_az = (byte *)&az;
        float mx = imu.calcMag(imu.mx);
        float my = imu.calcMag(imu.my);
        float mz = imu.calcMag(imu.mz);
        byte *b_mx = (byte *)&mx;
        byte *b_my = (byte *)&my;
        byte *b_mz = (byte *)&mz;
        payload[0] = 'I';
        payload[1] = b_gx[0];
        payload[2] = b_gx[1];
        payload[3] = b_gx[2];
        payload[4] = b_gx[3];
        payload[5] = b_gy[0];
        payload[6] = b_gy[1];
        payload[7] = b_gy[2];
        payload[8] = b_gy[3];
        payload[9] = b_gz[0];
        payload[10] = b_gz[1];
        payload[11] = b_gz[2];
        payload[12] = b_gz[3];

        payload[13] = b_ax[0];
        payload[14] = b_ax[1];
        payload[15] = b_ax[2];
        payload[16] = b_ax[3];
        payload[17] = b_ay[0];
        payload[18] = b_ay[1];
        payload[19] = b_ay[2];
        payload[20] = b_ay[3];
        payload[21] = b_az[0];
        payload[22] = b_az[1];
        payload[23] = b_az[2];
        payload[24] = b_az[3];

        payload[25] = b_mx[0];
        payload[26] = b_mx[1];
        payload[27] = b_mx[2];
        payload[28] = b_mx[3];
        payload[29] = b_my[0];
        payload[30] = b_my[1];
        payload[31] = b_my[2];
        payload[32] = b_my[3];
        payload[33] = b_mz[0];
        payload[34] = b_mz[1];
        payload[35] = b_mz[2];
        payload[36] = b_mz[3];
        
        Tx16Request tx = Tx16Request(0x0000, payload, sizeof(payload));

        xbee.send(tx);
        


//        Serial.write(b_gx, 4);
//        Serial.write(b_gy, 4);
//        Serial.write(b_gz, 4);


//        Serial.print(gx,2);
//        Serial.print(',');
//        Serial.print(gy,2);
//        Serial.print(',');
//        Serial.print(gz,2);
//        Serial.print('\n');

        
//        Serial.print(imu.calcGyro(imu.gx), 2);
//        Serial.print(',');
//        Serial.print(imu.calcGyro(imu.gy), 2);
//        Serial.print(',');
//        Serial.print(imu.calcGyro(imu.gz), 2);
//        Serial.print('\n');
//      mode=0; //reset mode
    }
    
    lastPrint = millis(); // Update lastPrint time
  }
}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
