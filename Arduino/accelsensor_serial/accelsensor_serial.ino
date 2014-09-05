#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

byte DATA_SYNC = 0x71;
byte DATA_ACK = 0x7E;
byte DATA_10B = 0x7F;
byte DATA_26B = 0x7D;
byte DATA_TYPE = 0x44;
byte SYNC_TYPE = 0x53;
byte END_MSG = 0x0A;
byte START_MSG = 0xAA;

unsigned long lastTick = 0;
unsigned long millisAtSync = 0; 
unsigned long loggingStatus = 0;
/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(5);
}

void setup(void) 
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no accel LSM303 detected ... Check your wiring!");
    while(1);
  }
  /* Initialise the sensor */
  if(!mag.begin())
  {
    Serial.println("Ooops, no mag LSM303 detected ... Check your wiring!");
    while(1);
  }
  gyro.begin();
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    Serial.println("Ooops, no gyro L3GD20 detected ... Check your wiring!");
    while(1);
  }
  loggingStatus = 0;
  /* Display some basic information on this sensor */
  displaySensorDetails();
}



void loop(void) 
{
  unsigned long thisTick = millis ();
  if ( thisTick >= lastTick + 2) { 
    lastTick = thisTick; 
    /* Get a new sensor event */ 
    sensors_event_t event;    
    sensors_event_t mag_event;  
    sensors_event_t gyro_event;  
    accel.getEvent(&event);
    mag.getEvent(&mag_event);
    gyro.getEvent(&gyro_event);
    byte mydata[26];
    mydata[0] = START_MSG;      
    mydata[1] = DATA_TYPE; 
    mydata[2] = DATA_26B; 
    mydata[3] = (int)((thisTick >> 24) & 0xFF) ;
    mydata[4] = (int)((thisTick >> 16) & 0xFF) ;
    mydata[5] = (int)((thisTick >> 8) & 0XFF);
    mydata[6] = (int)((thisTick & 0XFF));
    mydata[7] = event.acceleration.xlo;
    mydata[8] = event.acceleration.xhi;
    mydata[9] = event.acceleration.ylo;
    mydata[10] = event.acceleration.yhi;
    mydata[11] = event.acceleration.zlo;
    mydata[12] = event.acceleration.zhi;
    mydata[13] = event.magnetic.xlo;
    mydata[14] = event.magnetic.xhi;
    mydata[15] = event.magnetic.ylo;
    mydata[16] = event.magnetic.yhi;
    mydata[17] = event.magnetic.zlo;
    mydata[18] = event.magnetic.zhi;
    mydata[19] = gyro_event.gyro.xlo;
    mydata[20] = gyro_event.gyro.xhi;
    mydata[21] = gyro_event.gyro.ylo;
    mydata[22] = gyro_event.gyro.yhi;
    mydata[23] = gyro_event.gyro.zlo;
    mydata[24] = gyro_event.gyro.zhi;
    mydata[25] = END_MSG;
    String sData;
    sData += String(millis());
    sData += ";";
    sData += String(event.acceleration.x);
    sData += ";";
    sData += String(event.acceleration.y);
    sData += ";";
    sData += String(event.acceleration.z);
    sData += ";";
    sData += String(mag_event.magnetic.x);
    sData += ";";
    sData += String(mag_event.magnetic.y);
    sData += ";";
    sData += String(mag_event.magnetic.z);
    sData += ";";
    sData += String(gyro_event.gyro.x);
    sData += ";";
    sData += String(gyro_event.gyro.y);
    sData += ";";
    sData += String(gyro_event.gyro.z);
    Serial.println(sData);
    Serial1.write(mydata,26);
  }
}
