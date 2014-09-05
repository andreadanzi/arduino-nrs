#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
unsigned long millisAtSetup; 
unsigned long millisNow; 
unsigned long millisDelta;
unsigned long lastTick = 0;
unsigned long lastSyncTick = 0;
short loggingStatus = 0;
unsigned long syncInterval = 120000; // 120000 = 20 minuti
unsigned long pauseInterval = 5000; // 5 secondi
// unsigned long baudrate = 115200
unsigned long baudrate = 115200;
/*******************
* Creato uno script python udp_mc_client.py che dialoga tramite Firmata con questo sketch per
* 1) leggere i dati dei sensori e metterli su disco
* 2) sincronizzare il clock ad intervalli prestabiliti interroganto NTP lato python e associarlo al millis() che viene passato dallo sketch
          -> python a frequenza stabilita invia allos sketch tramite Firmata la propria ora da NTP e un'epoch in millisecondi a partire dal 2000
          -> sketch nel momento della ricezione del messaggio legge i millis e li restituisce allo script python  
          -> un paccketto UDP viene spedito in broadcast dal laptop a tutti i linino che tramite lo script python stanno in ascolto su una porta specifica. 
          -> quando sullo script python viene ricevuto il pacchetto, allora tramite un messagio via seriale si comunica allo sketch di restituire il millis che verrà usato come riferimento per quel nodo
* 3) notificare all'esterno lo stato del nodo (minuti che mancano a prossima syncronizzazione, disponibilità di dati, etc)
* 4) gestire il log rotating
*/

byte DATA_SYNC = 0x71;
byte DATA_ACK = 0x7E;
byte DATA_10B = 0x7F;
byte DATA_26B = 0x7D;
byte DATA_TYPE = 0x44;
byte SYNC_TYPE = 0x53;
byte END_MSG = 0x0A;
byte START_MSG = 0xAA;
int msgBytesRead = 0;
byte storedInputData[32];
boolean parsingMessage;
typedef void (*stringCallbackFunction)(char*);

stringCallbackFunction currentStringCallback;

void processSerialInput(void) {
  byte inputData = Serial1.read();
  if(parsingMessage) {
    String strOut="processSerialInput:";
    strOut +=String(inputData);
    Serial.println(strOut);
    if(inputData == END_MSG) {
      parsingMessage = false;
      if(currentStringCallback) {
         int bufferLength = msgBytesRead;
         char *buffer = (char*)malloc(bufferLength * sizeof(char));
         byte j = 0;
         while(j < bufferLength) {
            buffer[j] = (char)storedInputData[j];
            j++;
         }
        (*currentStringCallback)(buffer);
      }
    } else {
        //normal data byte - add to buffer
        storedInputData[msgBytesRead] = inputData;
        msgBytesRead++;
    }
  } else {
    if(inputData == START_MSG) {
      msgBytesRead = 0;
      parsingMessage = true;
    }
  }
}

void sendStatus(unsigned long thisTick) {  
  byte mydata[10];
  mydata[0] = START_MSG;      
  mydata[1] = DATA_TYPE; // 0x44 D=DATA; 0x53 S=SYNC
  mydata[2] = DATA_ACK; // DATA_SYNC
  mydata[3] = (int)((thisTick >> 24) & 0xFF) ;
  mydata[4] = (int)((thisTick >> 16) & 0xFF) ;
  mydata[5] = (int)((thisTick >> 8) & 0XFF);
  mydata[6] = (int)(thisTick & 0XFF);
  mydata[7] = (int)(loggingStatus & 0XFF);
  mydata[8] = (int)((loggingStatus >>8 ) & 0XFF);
  mydata[9] = END_MSG;
  int numBytes = Serial1.write(mydata,10);
  Serial.print("sendStatus sents ");
  Serial.print(numBytes);
  Serial.println(" bytes");
}

void sendSync(unsigned long thisTick) {  
  byte mydata[10];
  mydata[0] = START_MSG;      
  mydata[1] = DATA_TYPE; // 0x44 D=DATA; 0x53 S=SYNC
  mydata[2] = DATA_SYNC; // DATA_SYNC SYNC_TYPE
  mydata[3] = (int)((thisTick >> 24) & 0xFF) ;
  mydata[4] = (int)((thisTick >> 16) & 0xFF) ;
  mydata[5] = (int)((thisTick >> 8) & 0XFF);
  mydata[6] = (int)(thisTick & 0XFF);
  mydata[7] = (int)(loggingStatus & 0XFF);
  mydata[8] = (int)((loggingStatus >>8 ) & 0XFF);
  mydata[9] = END_MSG;
  int numBytes = Serial1.write(mydata,10);
  Serial.print("sendSync sents ");
  Serial.print(numBytes);
  Serial.println(" bytes");
}


void sendAMGData(unsigned long thisTick, sensors_event_t acc_event,  sensors_event_t mag_event, sensors_event_t gyro_event) {  
  byte mydata[26];
  mydata[0] = START_MSG;      
  mydata[1] = DATA_TYPE; // 0x44 D=DATA; 0x53 S=SYNC
  mydata[2] = DATA_26B; // DATA_26BDATA_26BDATA_26BDATA_26BDATA_26B
  mydata[3] = (int)((thisTick >> 24) & 0xFF) ;
  mydata[4] = (int)((thisTick >> 16) & 0xFF) ;
  mydata[5] = (int)((thisTick >> 8) & 0XFF);
  mydata[6] = (int)((thisTick & 0XFF));
  mydata[7] = acc_event.acceleration.xlo;
  mydata[8] = acc_event.acceleration.xhi;
  mydata[9] = acc_event.acceleration.ylo;
  mydata[10] = acc_event.acceleration.yhi;
  mydata[11] = acc_event.acceleration.zlo;
  mydata[12] = acc_event.acceleration.zhi;
  mydata[13] = mag_event.magnetic.xlo;
  mydata[14] = mag_event.magnetic.xhi;
  mydata[15] = mag_event.magnetic.ylo;
  mydata[16] = mag_event.magnetic.yhi;
  mydata[17] = mag_event.magnetic.zlo;
  mydata[18] = mag_event.magnetic.zhi;
  mydata[19] = gyro_event.gyro.xlo;
  mydata[20] = gyro_event.gyro.xhi;
  mydata[21] = gyro_event.gyro.ylo;
  mydata[22] = gyro_event.gyro.yhi;
  mydata[23] = gyro_event.gyro.zlo;
  mydata[24] = gyro_event.gyro.zhi;
  mydata[25] = END_MSG;
  int numBytes = Serial1.write(mydata,26);
}

void processInputString(char *myString) {
    Serial.print("processInputString ");
    Serial.println(myString);
    String sIn(myString);
    String sRet;
    byte mydata[14];
    unsigned long thisTick = millis ();
    if( sIn.startsWith("SYNC") ){
      sendSync(thisTick);      
      Serial.print("SYNC;");Serial.println(thisTick);
    } else {
      if( sIn.startsWith("STOP") ){
        loggingStatus = 0;
        Serial.println("Status is: Stopped (0)");
      }
      if( sIn.startsWith("STRT") ){
        loggingStatus = 1;
        Serial.println("Status is: Started (1)");
      }
      sendStatus(thisTick);
    }
}

void setup(void) 
{  
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  delay(500);
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  Serial.begin(baudrate);
  Serial1.begin(baudrate);
  // Wait for U-boot to finish startup.  Consume all bytes until we are done.
  do {
     while (Serial1.available() > 0) {
        Serial1.read();
        }
    delay(1000);
  } while (Serial1.available()>0);
  delay(500);
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  accel.begin();
  /* Initialise the sensor */
  if(!accel.begin())
  {
    while(1);
  }
  mag.begin();
  /* Initialise the sensor */
  if(!mag.begin())
  {
    while(1);
  }
  gyro.enableAutoRange(false);
  gyro.begin(GYRO_RANGE_2000DPS);
  /* Initialise the sensor */
  if(!gyro.begin(GYRO_RANGE_2000DPS))
  {
    while(1);
  }
  millisAtSetup = millis();
  lastTick = millisAtSetup;
  lastSyncTick =  lastTick;
  loggingStatus = 0;
  delay(500);
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(300);
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
  delay(300);
  digitalWrite(13,HIGH);
  delay(5000);
  currentStringCallback = processInputString;
  parsingMessage = false;
  Serial.println("Started!");
  Serial.println("Status is: Stopped (0)");
}

void loop(void) 
{
  while(Serial1.available()>0) {
    processSerialInput();
  }
  if(digitalRead(13)==HIGH) digitalWrite(13,LOW);
  unsigned long thisTick = millis ();
  if(loggingStatus > 0) {
    if ( thisTick >= lastTick + 2 ) {   
      sensors_event_t acc_event;    
      sensors_event_t mag_event;    
      sensors_event_t gyro_event;  
      accel.getEvent(&acc_event);
      mag.getEvent(&mag_event);
      gyro.getEvent(&gyro_event);
      lastTick = thisTick;
      String sData;
      sData += "DATA;";
      sData += String(lastTick);
      sData += ";";
      sData += String(acc_event.acceleration.x);
      sData += ";";
      sData += String(acc_event.acceleration.y);
      sData += ";";
      sData += String(acc_event.acceleration.z);
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
      //Serial1.println(sData);
      sendAMGData(lastTick,acc_event,mag_event, gyro_event);  
    }
  } else {
    digitalWrite(13,LOW);
    delay(1000);
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(1000);
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(1000);
    digitalWrite(13,HIGH);
    delay(2000);
  }
}

