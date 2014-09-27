/*

XBee Series 1 Tranmitter
16-Bit 

Source: http://code.google.com/p/xbee-arduino/
Xbee ver 0.3 wont compile unless you change NewSoftSerial.h to SoftwareSerial.h. 
see: http://arduino.cc/forum/index.php?topic=84789.0
How to configure xBees http://code.google.com/p/xbee-api/wiki/XBeeConfiguration
Good explanation of sketch http://code.google.com/p/xbee-arduino/wiki/DevelopersGuide
xbee.h Documentation http://xbee-arduino.googlecode.com/svn/trunk/docs/api/index.html



Configure this Xbee with X-CTU (values are hex)
 PAN 3636  Personal Area Network ID - all xBees need to be on same PAN
 DL 51     Lower Byte Address (not used in 16-bit addressing, but I like to set something unuque anyway)
 MY 100    ID of xBee, all xBees need to be different
 CE 0      Coordinator - sets this XBee as the END NODE
 AP 2      Use API Mode

To send data to a particular xbee, this transmitter uses the Receiver's MY address, not the DL address
See http://www.digi.com/support/kbase/kbaseresultdetl?id=2187

 
 */
#include <SD.h>
#include <Wire.h> 
#include <XBee.h>  //http://code.google.com/p/xbee-arduino/     Modified per http://arduino.cc/forum/index.php/topic,111354.0.html
#include <SoftwareSerial.h>

#define PRINT_DEBUG       // comment out if you don't want anything to go to serial monitor
#define MY_ADDR_RX 0x000  // The MY address of the Rx XBee 
#define SERIAL_BAUD_RATE 9600    // Baud for both Xbee and serial monitor
#define XBEE_BAUD_RATE 115200    // Baud for both Xbee and serial monitor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to pin connected to CS 17 = D14
const int chipSelect = 9;   

unsigned long istatus = 0;
unsigned long iregistered = 0;
unsigned long start = millis();
unsigned long timedelay = 300000; // 5minuti
unsigned long last_tick = 0;
unsigned long sampleno = 0;
unsigned long idelay = 0;

#define NUM_DATA_PTS   15  // Number of integers (data points) to upload. Can't exceed 100 bytes or 50 integers unless you change MAX_FRAME_DATA_SIZE in XBee.h

// allocate array to hold bytes to send to other xbee.  Size is 2x the numer if integers being sent  
uint8_t payload[NUM_DATA_PTS * 2];
uint8_t ack_payload[10];

XBee xbee = XBee();
// 16-bit addressing: Enter address of remote XBee, typically the coordinator
Tx16Request tx = Tx16Request(MY_ADDR_RX, payload, sizeof(payload));
Tx16Request ack_tx = Tx16Request(MY_ADDR_RX, ack_payload, sizeof(ack_payload));
TxStatusResponse txStatus = TxStatusResponse();

XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

uint8_t option = 0;
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
byte gyro_raw[6];
byte magnetom_raw[6];

int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

float _hmc5883_Gauss_LSB_XY = 1100;
float _hmc5883_Gauss_LSB_Z  = 980;
//============================================================================
//  Flash LEDs to indicate varius states
//============================================================================
void flashLed(int pin, int times, int wait) 
{
    for (int i = 0; i < times; i++) 
    {
      TXLED1;
      delay(wait);
      TXLED0;
      if (i + 1 < times) 
      {delay(wait);}
    }
} // flashLed()

// Same functionality as Arduino's standard map function, except using floats
// mapf(rawX, 0, 1023, -scale, scale);
// rawX *2*scale / (1023-scale)
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void send_ack(unsigned long at_now) {
  ack_payload[0] = 'C'; //67
  ack_payload[1] = 'H'; //72
  ack_payload[2] = istatus >> 24 & 0xff;
  ack_payload[3] = istatus >> 16 & 0xff;
  ack_payload[4] = istatus >> 8 & 0xff;
  ack_payload[5] = istatus & 0xff;
  ack_payload[6] = at_now >> 24 & 0xff;
  ack_payload[7] = at_now >> 16 & 0xff;
  ack_payload[8] = at_now >> 8 & 0xff;
  ack_payload[9] = at_now & 0xff;
  xbee.send(ack_tx);
  if(xbee.readPacket(500)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)  {
      xbee.getResponse().getTxStatusResponse(txStatus);	
      // get the delivery status, 0 = OK, 1 = Error, 2 = Invalid Command, 3 = Invalid Parameter 
      if (txStatus.getStatus() == SUCCESS) 
      {
        // OK
      } 
      else 
      {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(RXLED, 1, 250);
        #ifdef PRINT_DEBUG
          Serial.print("send_ack ");
          Serial.print(istatus);
          Serial.print(" - At "); Serial.print(at_now);
          Serial.print(" Tx Failed, xbee status = ");
          Serial.println(txStatus.getStatus());
        #endif
      }
    }
  } else if (xbee.getResponse().isError()) {
    flashLed(RXLED, 2, 250);
    #ifdef PRINT_DEBUG
      Serial.print("send_ack ");
      Serial.print(istatus);
      Serial.print(" - At "); Serial.print(at_now);
      Serial.print(" Error reading packet.  Error code: ");  
      Serial.println(xbee.getResponse().getErrorCode());
    #endif
  } else {
    // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
    flashLed(RXLED, 3, 250);
    #ifdef PRINT_DEBUG
      Serial.print("send_ack ");
      Serial.print(istatus);
      Serial.print(" - At "); Serial.print(at_now);
      Serial.println(" XBee did not provide a timely Tx Status Response");  
    #endif
  } 
}


void send_subscribe(unsigned long at_now) {
  ack_payload[0] = 'S'; //83
  ack_payload[1] = 'U'; //85
  ack_payload[2] = istatus >> 24 & 0xff;
  ack_payload[3] = istatus >> 16 & 0xff;
  ack_payload[4] = istatus >> 8 & 0xff;
  ack_payload[5] = istatus & 0xff;
  ack_payload[6] = at_now >> 24 & 0xff;
  ack_payload[7] = at_now >> 16 & 0xff;
  ack_payload[8] = at_now >> 8 & 0xff;
  ack_payload[9] = at_now & 0xff;
  xbee.send(ack_tx);
  if(xbee.readPacket(500)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)  {
      xbee.getResponse().getTxStatusResponse(txStatus);	
      // get the delivery status, 0 = OK, 1 = Error, 2 = Invalid Command, 3 = Invalid Parameter 
      if (txStatus.getStatus() == SUCCESS) 
      {
        // OK
      } 
      else 
      {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(RXLED, 1, 250);
        #ifdef PRINT_DEBUG
          Serial.print("send_subscribe ");
          Serial.print(istatus);
          Serial.print(" - At "); Serial.print(at_now);
          Serial.print(" Tx Failed, xbee status = ");
          Serial.println(txStatus.getStatus());
        #endif
      }
    }
  } else if (xbee.getResponse().isError()) {
    flashLed(RXLED, 2, 250);
    #ifdef PRINT_DEBUG
      Serial.print("send_subscribe ");
      Serial.print(istatus);
      Serial.print(" - At "); Serial.print(at_now);
      Serial.print(" Error reading packet.  Error code: ");  
      Serial.println(xbee.getResponse().getErrorCode());
    #endif
  } else {
    // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
    flashLed(RXLED, 3, 250);
    #ifdef PRINT_DEBUG
      Serial.print("send_subscribe ");
      Serial.print(istatus);
      Serial.print(" - At "); Serial.print(at_now);
      Serial.println(" XBee did not provide a timely Tx Status Response");  
    #endif
  } 
}


void checkMessages(unsigned long at_now) {
  xbee.readPacket();
  if(xbee.getResponse().isAvailable()  ) 
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE ) {
      Serial.print("IN checkMessages THERE IS A RX_16_RESPONSE ");
      xbee.getResponse().getRx16Response(rx16);
      if( rx16.getDataLength () > 0 ) {
        uint16_t remoteAddr = rx16.getRemoteAddress16();
  	option = rx16.getOption();
  	data = rx16.getData(0);
        Serial.print(" from ");
        Serial.print( remoteAddr );
        Serial.print(" first char is ");
        Serial.println(data);
        // CHECK C = 67
        if( data == 67) {
          send_ack(at_now);
          Serial.println("CHECK ACK Sent!");
        }
        // START switch S = 83
        if( data == 83) {
          istatus = 1;
          send_ack(at_now);
          Serial.println("START ACK Sent!");
        }
        // STOP switch T = 84
        if( data == 84) {
          istatus = 0;
          send_ack(at_now);
          Serial.println("STOP ACK Sent!");
        }
        // SEND DATA R = 82
        if( data == 82) {
          istatus = 2;            
          send_ack(at_now);
          Serial.println("SEND DATA ACK Sent!");
        }
      } else {
        Serial.println("RX_16_RESPONSE has empty payload!");
      }
    }
  }
}
File myFile;
//============================================================================
//  Initialize 
//============================================================================
void setup() 
{
  pinMode(chipSelect, OUTPUT);
  
  delay(5000);
  #ifdef PRINT_DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
    }
    Serial.println("Xbee Tx/End Node - setup()");
  #endif
  Serial1.begin(XBEE_BAUD_RATE);
  xbee.setSerial(Serial1);
  Wire.begin();
  Serial.println("I2C Initializad!");
  delay(2000);
  Accel_Init();
  Serial.println("Accel Initializad!");
  delay(2000);
  Gyro_Init();
  Serial.println("Gyro Initializad!");
  delay(2000);
  Magn_Init();
  Serial.println("Magn Initializad!");
  flashLed(RXLED,   5, 50);
  istatus = 0;
  last_tick = 0;
  if (!SD.begin(9)) {
    #ifdef PRINT_DEBUG
    Serial.println("initialization failed!");
    #endif
    return;
  }
  #ifdef PRINT_DEBUG
  Serial.println("initialization done.");
  if (SD.exists("example.txt")) {
    Serial.println("example.txt exists.");
  }
  else {
    Serial.println("example.txt doesn't exist.");
  }
  #endif

  
} // setup()

//============================================================================
//  
//============================================================================
void loop() 
{
    int16_t xbeeData[NUM_DATA_PTS];  // Array to hold integers that will be sent to other xbee
    unsigned long at_now = millis();
    // Serial.println(at_now);
    // Read ADXL377
    int rawX = analogRead(A0);
    int rawY = analogRead(A1);
    int rawZ = analogRead(A2);  
    float scaledX, scaledY, scaledZ; // Scaled values for each axis
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
    /* Print out scaled X,Y,Z accelerometer readings
    Serial.print("X: "); Serial.print(scaledX);
    Serial.print(" Y: "); Serial.print(scaledY);
    Serial.print(" Z: "); Serial.print(scaledZ); Serial.print(" g");
    Serial.println(); */
    Read_Accel();
    Read_Magn();
    Read_Gyro();
    // output_sensors_text('R');
    float fheading = atan2(magnetom[1], magnetom[0]);
    if(fheading < 0)
      fheading += 2 * M_PI;
    //Serial.print("heading:\t");
    //Serial.println(fheading * 180/M_PI);
    // wait up to 15 seconds 
    if (at_now - start > 15000) {
      if( iregistered == 0 || at_now - last_tick > timedelay ) {
        if(iregistered==1) idelay = 3000;
        last_tick = at_now;
        send_subscribe(at_now);
        iregistered = 1;
        Serial.print("At "); Serial.print(at_now);
        Serial.println(" End Node Subscribed!");
        // checkMessages(at_now);        
      } else if(istatus > 0 && idelay > 0) {
        if( idelay % 1000 == 0 )  {
          Serial.print("At "); Serial.print(at_now);
          Serial.print(" Try to submit a message - ");
          Serial.println(idelay/1000);
        }
        idelay--;
        checkMessages(at_now);
      } else if(iregistered == 1 && istatus > 0) {
        sampleno++;
        payload[0] = 'S'; // in decimal 83
        payload[1] = 'T'; // in decimal 84
        // 9DOF ACCEL
        payload[2] = accel_raw[0]; // X HI
        payload[3] = accel_raw[1]; // X LO
        payload[4] = accel_raw[2]; // Y HI
        payload[5] = accel_raw[3]; // Y LO
        payload[6] = accel_raw[4]; // Z HI
        payload[7] = accel_raw[5]; // Z LO
        // 9DOF GYRO
        payload[8] = gyro_raw[0];   // X HI
        payload[9] = gyro_raw[1];   // X LO
        payload[10] = gyro_raw[2];   // Y HI
        payload[11] = gyro_raw[3];   // Y LO
        payload[12] = gyro_raw[4];  // Z HI
        payload[13] = gyro_raw[5];  // ZLO
        // 9DOF MAGN
        payload[14] = magnetom_raw[0]; // X HI
        payload[15] = magnetom_raw[1]; // X LO
        payload[16] = magnetom_raw[2]; // Y HI
        payload[17] = magnetom_raw[3]; // Y LO
        payload[18] = magnetom_raw[4]; // Z HI
        payload[19] = magnetom_raw[5]; // ZLO
        // ACC ADXL377
        payload[20] = rawX >> 8 & 0xff;   // X HI
        payload[21] = rawX & 0xff;        // X LO
        payload[22] = rawY >> 8 & 0xff;   // Y HI
        payload[23] = rawY & 0xff;        // Y LO
        payload[24] = rawZ >> 8 & 0xff;   // Z HI
        payload[25] = rawZ & 0xff;        // ZLO
        // Timestamp
        payload[26] = at_now >> 24 & 0xff;
        payload[27] = at_now >> 16 & 0xff;
        payload[28] = at_now >> 8 & 0xff;
        payload[29] = at_now & 0xff;      
        if (istatus == 1) {
          xbee.send(tx);
          if(xbee.readPacket(500)) {
            if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE)  {
              xbee.getResponse().getTxStatusResponse(txStatus);	
              // get the delivery status, 0 = OK, 1 = Error, 2 = Invalid Command, 3 = Invalid Parameter 
              if (txStatus.getStatus() == SUCCESS) 
              {
                // OK
              } 
              else 
              {
                // the remote XBee did not receive our packet. is it powered on?
                flashLed(RXLED, 1, 250);
                #ifdef PRINT_DEBUG
                  Serial.print(istatus);
                  Serial.print(" - At "); Serial.print(at_now);
                  Serial.print(" Tx Failed, xbee status = ");
                  Serial.println(txStatus.getStatus());
                #endif
              }
            }
          } else if (xbee.getResponse().isError()) {
            flashLed(RXLED, 2, 250);
            #ifdef PRINT_DEBUG
              Serial.print(istatus);
              Serial.print(" - At "); Serial.print(at_now);
              Serial.print(" Error reading packet.  Error code: ");  
              Serial.println(xbee.getResponse().getErrorCode());
            #endif
          } else {
            // local XBee did not provide a timely TX Status Response.  Radio is not configured properly or connected
            flashLed(RXLED, 3, 250);
            #ifdef PRINT_DEBUG
              Serial.print("loop ");
              Serial.print(istatus);
              Serial.print(" - At "); Serial.print(at_now);
              Serial.println(" XBee did not provide a timely Tx Status Response");  
            #endif
          } 
        } else if (istatus == 3) {
          istatus = istatus;
        } else if (istatus == 2) {
          istatus = istatus;
          myFile = SD.open("log.txt", FILE_WRITE);
          String sData;
          
           // 9DOF ACCEL
        payload[2] = accel_raw[0]; // X HI
        payload[3] = accel_raw[1]; // X LO
        payload[4] = accel_raw[2]; // Y HI
        payload[5] = accel_raw[3]; // Y LO
        payload[6] = accel_raw[4]; // Z HI
        payload[7] = accel_raw[5]; // Z LO
        // 9DOF GYRO
        payload[8] = gyro_raw[0];   // X HI
        payload[9] = gyro_raw[1];   // X LO
        payload[10] = gyro_raw[2];   // Y HI
        payload[11] = gyro_raw[3];   // Y LO
        payload[12] = gyro_raw[4];  // Z HI
        payload[13] = gyro_raw[5];  // ZLO
        // 9DOF MAGN
        payload[14] = magnetom_raw[0]; // X HI
        payload[15] = magnetom_raw[1]; // X LO
        payload[16] = magnetom_raw[2]; // Y HI
        payload[17] = magnetom_raw[3]; // Y LO
        payload[18] = magnetom_raw[4]; // Z HI
        payload[19] = magnetom_raw[5]; // ZLO
        // ACC ADXL377
        payload[20] = rawX >> 8 & 0xff;   // X HI
        payload[21] = rawX & 0xff;        // X LO
        payload[22] = rawY >> 8 & 0xff;   // Y HI
        payload[23] = rawY & 0xff;        // Y LO
        payload[24] = rawZ >> 8 & 0xff;   // Z HI
        payload[25] = rawZ & 0xff;        // ZLO
        // Timestamp
        payload[26] = at_now >> 24 & 0xff;
        payload[27] = at_now >> 16 & 0xff;
        payload[28] = at_now >> 8 & 0xff;
        payload[29] = at_now & 0xff;    
          myFile.println(sData);
          myFile.close();
        }
      } else {
        checkMessages(at_now);     
        flashLed(RXLED, 5, 500); 
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
void Read_Accel()
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
  else
  {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
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


void Read_Magn()
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
    magnetom_raw[0] = buff[0]; // HX
    magnetom_raw[1] = buff[1]; // LX
    magnetom_raw[2] = buff[4]; // HY
    magnetom_raw[3] = buff[5]; // LY
    magnetom_raw[4] = buff[2]; // HZ
    magnetom_raw[5] = buff[3]; // LZ
  }
  else
  {
    num_magn_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
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
void Read_Gyro()
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
    gyro_raw[0] = buff[0]; // HX
    gyro_raw[1] = buff[1]; // LX
    gyro_raw[2] = buff[2]; // HY
    gyro_raw[3] = buff[3]; // LY
    gyro_raw[4] = buff[4]; // HZ
    gyro_raw[5] = buff[5]; // LZ
  }
  else
  {
    num_gyro_errors++;
    if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}


void output_sensors_text(char raw_or_calibrated)
{
   
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();
  /*
  
  int iax = (((int) accel_raw[0] ) << 8) | accel_raw[1];
  int iay =  (((int) accel_raw[2] ) << 8) | accel_raw[3];
  int iaz = (((int) accel_raw[4] ) << 8) | accel_raw[5];
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(iax); Serial.print(",");
  Serial.print(iay); Serial.print(",");
  Serial.print(iaz); Serial.println();*/
  
  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();
    
  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}


