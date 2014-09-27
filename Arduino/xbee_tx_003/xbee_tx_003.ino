/*

 */
#include <SD.h>
#include <Wire.h> 
#include <XBee.h>
//#define PRINT_DEBUG       // comment out if you don't want anything to go to serial monitor

unsigned long istatus = 0;
unsigned long iopen = 0;
unsigned long iflush = 0;
unsigned long last_sync = 0;
unsigned long prog_sync = 0;
unsigned long ireg = 0;
unsigned long start = millis();
unsigned long timedelay = 300000; // 5minuti
unsigned long last_tick = 0;
unsigned long idelay = 0;
// allocate array to hold bytes to send to other xbee.  Size is 2x the numer if integers being sent  
uint8_t pyld[30];
uint8_t ackp[10];
XBee xbee = XBee();
// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request tx = Tx16Request(0x000, pyld, sizeof(pyld));
Tx16Request ack_tx = Tx16Request(0x000, ackp, sizeof(ackp));
TxStatusResponse txStatus = TxStatusResponse();

XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

uint8_t data = 0;

int RXLED = 99;

// ADXL377
int scale = 200;
boolean micro_is_5V = false; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((uint8_t) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

// 9DOF Sensor Stick

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false
float accel[3];
float gyro[3];
float magnetom[3];
float scale_fact = 3.9; //  3.9 mg/LSB scale factor
byte accel_raw[6];

//============================================================================
//  Flash LEDs to indicate varius states
//============================================================================
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void send_ack(unsigned long at_now, int mode) {
  if(  mode == 0)  {
    ackp[0] = 'C'; //67
    ackp[1] = 'H'; //72
  } else if (mode == 1) {
    ackp[0] = 'S'; 
    ackp[1] = 'U';
  }
  ackp[2] = istatus >> 24 & 0xff;
  ackp[3] = istatus >> 16 & 0xff;
  ackp[4] = istatus >> 8 & 0xff;
  ackp[5] = istatus & 0xff;
  ackp[6] = at_now >> 24 & 0xff;
  ackp[7] = at_now >> 16 & 0xff;
  ackp[8] = at_now >> 8 & 0xff;
  ackp[9] = at_now & 0xff;
  xbee.send(ack_tx);
  if(xbee.readPacket(500)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)  {
      xbee.getResponse().getTxStatusResponse(txStatus);
    }
  }
}


File myFile;

void closeMyfile() {
  if(istatus == 9 && iopen==1) {
    myFile.close();
    iopen=0;
  }
}

void openMyFile(uint8_t mode) {
  if( iopen == 0) {
    String f = "log";
    f += last_sync;
    f += ".csv";
    myFile = SD.open(f.c_str(),  mode);
    iopen = 1;
  }
}


void checkMessages(unsigned long at_now) {
  xbee.readPacket();
  if(xbee.getResponse().isAvailable()  ) 
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE ) {
      xbee.getResponse().getRx16Response(rx16);
      if( rx16.getDataLength () > 0 ) {
  	data = rx16.getData(0);
        // CHECK C = 67
        if( data == 67) {
          File cf = SD.open("sync.csv", O_CREAT | O_APPEND | O_WRITE);
          char myChar;
          String sD = String(prog_sync );
          last_sync = prog_sync;
          sD += ";";
          myChar = rx16.getData(1);
          sD += String(myChar );
          myChar = rx16.getData(2);
          sD += String(myChar);
          sD += ";";
          sD += String(at_now);
          sD += ";END";
          cf.println(sD);
          cf.close();
          last_tick = at_now;
          send_ack(at_now,0);
          prog_sync++;
        }
        // START switch S = 83
        if( data == 83) {
          istatus = 1;
          send_ack(at_now,0);
        }
        // STOP switch T = 84
        if( data == 84) {
          closeMyfile();
          istatus = 0;
          send_ack(at_now,0);
        }
        // SEND DATA TO COORDINATOR R = 82
        if( data == 82) {
          closeMyfile();
          istatus = 2;            
          send_ack(at_now,0);
        }
        // LOG DATA TO SD L = 76
        if( data == 76) {
          istatus = 9;           
          last_tick = at_now; 
          send_ack(at_now,0);
        }
        
      } else {
        Serial.println("RX_16 empty");
      }
    }
  }
}
//============================================================================
//  Initialize 
//============================================================================
void setup() 
{
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(115200);
  xbee.setSerial(Serial1);
  Wire.begin();
  Accel_Init();
  Gyro_Init();
  Magn_Init();
  istatus = 0;
  last_tick = 0;
  if (!SD.begin(9,SPI_FULL_SPEED)) { // modificato libraries/SD/SD.h SD.cpp in begin e 
    Serial.println("sd failed");
    return;
  }
  iopen = 0;
} // setup()



//============================================================================
//  main loop
//============================================================================
void loop() 
{
    unsigned long at_now = millis();
    // Serial.println(at_now,0);
    /* Read ADXL377
    int rawX = analogRead(A0);
    int rawY = analogRead(A1);
    int rawZ = analogRead(A2);  
    float scaledX, scaledY, scaledZ; // Scaled values for each axis
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
    */
    RAcc();
    RMag();
    RGyr();
    if (at_now - start > 15000) {
      if( ireg == 0 || at_now - last_tick > timedelay ) {
        if(ireg==1) idelay = 3000;
        last_tick = at_now;
        send_ack(at_now,1);
        ireg = 1;
        Serial.println(" End Node Subscribed!");
      } else if(istatus > 0 && idelay > 0) {
        if( idelay % 1000 == 0 )  {
          Serial.print(" Try to submit a message - ");
          Serial.println(idelay/1000);
        }
        idelay--;
        checkMessages(at_now);
      } else if(ireg == 1 && istatus > 0) {
        if (istatus == 2) {
          File root = SD.open("/");
          while(true) {
            File fe =  root.openNextFile();
            if (! fe) {
               // no more files
             break;
            }
            if (!fe.isDirectory()) {
              
            }
            fe.close();
          }
          //xbee.send(tx);xbee.readPacket(500);
          root.close();
          istatus = 0;
          send_ack(at_now,0);
        } else if (istatus == 9) {
          openMyFile(O_CREAT | O_WRITE);
          String sD;
          sD += String(at_now);
          sD += ";";
          sD += accel[0];
          sD += ";";
          sD += accel[1];
          sD += ";";
          sD += accel[2];
          sD += ";";
          sD += gyro[0];
          sD += ";";
          sD += gyro[1];
          sD += ";";
          sD += gyro[2];
          sD += ";";
          sD += magnetom[0];
          sD += ";";
          sD += magnetom[1];
          sD += ";";
          sD += magnetom[2];
          myFile.println(sD);
          /* ogni tot chiamare flush */
          iflush++;
          if(iflush > 500) {
            iflush = 0;
            myFile.flush();
          }
        }
      } else {
        checkMessages(at_now);     
        //flashLed(RXLED, 5, 500); 
      }
    } // wait 15 seconds
}  // void()



void Accel_Init()
{
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2D);  // Power register
  WIRE_SEND(0x08);  // Measurement mode
  Wire.endTransmission();
  delay(5);  
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  // Data format register
  WIRE_SEND(0x0B);  // Set to 16g full resolution
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  WIRE_SEND(0x0D);  // Set to 800 hz bandwith 400 nz 1101
  Wire.endTransmission();
  delay(5);
}


// Reads x, y and z accelerometer registers
void RAcc()
{
  int i = 0;
  byte buff[6];
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  if (i == 6)  // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    accel[0] = (((int) buff[1]) << 8) | buff[0]; // X axis (external label is y axis)
    accel[0] = (scale_fact*accel[0])/1000;
    accel[1] = (((int) buff[3]) << 8) | buff[2]; // Y axis (external label is x axis)
    accel[1] = (scale_fact*accel[1])/1000;
    accel[2] = (((int) buff[5]) << 8) | buff[4]; // Z axis (external label is z axis)
    accel[2] = (scale_fact*accel[2])/1000;
    accel_raw[0] = buff[1]; // HX
    accel_raw[1] = buff[0]; // LX
    accel_raw[2] = buff[3]; // HY
    accel_raw[3] = buff[2]; // LY
    accel_raw[4] = buff[5]; // HZ
    accel_raw[5] = buff[4]; // LZ
  }
}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);
  // 0x01 Main Gain
  
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x01);
  WIRE_SEND(0x20);  // GAIN +/- 1.3
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 75Hz
  Wire.endTransmission();
  delay(5);

}


void RMag()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
  }
}

void Gyro_Init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  // full-scale range of the gyro sensors,low pass filter configuration and internal sampling rate configuration
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x19);  // 11001 FS_SEL = 3 (11) ±2000°/sec DLPF_CFG = 1 (001) 188Hz 1kHz
  Wire.endTransmission();
  delay(5);
  // Set sample ratio to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x01);  //  SMPLRT_DIV = 1 (500Hz)
  Wire.endTransmission();
  delay(5);
  
  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
}


// Reads x, y and z gyroscope registers
void RGyr()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  if (i == 6)  // All bytes received?
  {
    gyro[0] = -1 * ((((int) buff[0]) << 8) | buff[1]);     // X axis (external label y axis)
    gyro[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);    // Y axis (external label x axis)
    gyro[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);    // Z axis (external label z axis)
  }
}
