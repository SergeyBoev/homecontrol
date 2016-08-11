/*
 * Modbus управление освещением
 * D0, D1, D13 fot RS-485
 ******OUT******
 * D3 Нагрузка 1 Лампа        mb_c_rw 10  mb_r_rw 10
 * D5 Нагрузка 2 Лампа        mb_c_rw 11  mb_r_rw 11
 * D6 Нагрузка 3 Теплый пол   mb_c_rw 12  mb_r_rw 12
 * D9 Нагрузка 4 Эл замок     mb_c_rw 13  mb_r_rw 13
 * D10 Нагрузка 5             mb_c_rw 14  mb_r_rw 14
 * D11 Нагрузка 6             mb_c_rw 15  mb_r_rw 15
 ******IN*******
 * D2 Датчик окна             mb_c_ro 16 
 * D4 Датчик двери            mb_c_ro 17
 * D7 Датчик движения         mb_c_ro 18
 * D8 1-wire
 * D12 DHT-11 температура влажность mb_r_ro 16   mb_r_ro 17
 * A0 Датчик тока нагрузки 1  mb_r_ro 20
 * A1 Датчик тока нагрузки 2  mb_r_ro 21
 * A2 Датчик тока нагрузки 3  mb_r_ro 22
 * A3 Датчик тока нагрузки 4  mb_r_ro 23
 * A4 SDA
 * A5 SCL
 * A6 Датчик тока нагрузки 5  mb_r_ro 24
 * A7 Датчик тока нагрузки 6  mb_r_ro 25
 */

#include <EEPROM.h>
#include "DHT.h"
#include <ModbusSlave.h>

#define DHTPIN 12     // what digital pin we're connected to
#define DHTTYPE DHT11
#define mb_r_ro16 16
#define mb_r_ro17 17

#define INOUTCOUNT 6
#define INOUTEEPROMSTART

#define OUTD00 3
#define OUTD01 5
#define OUTD02 6
#define OUTD03 9
#define OUTD04 10
#define OUTD05 11
#define MBOUTD 10 

unsigned char OUTPI[6] = {
  OUTD00, OUTD01, OUTD02, OUTD03, OUTD04, OUTD05};
unsigned char OUTD[6];
#define OUTD_EEPROM_BASE 10

void SetupAnalogOUT(){
  for (byte i=0; i++; i<INOUTCOUNT) {
    pinMode(OUTPI[i], OUTPUT);
    SetAnalogOUT(i, EEPROM.read(OUTD_EEPROM_BASE+i));
  }
};
void SetAnalogOUT(unsigned char index, unsigned char value){
  if (index >=6) return;
  if (EEPROM.read(OUTD_EEPROM_BASE+index) != value) 
    EEPROM.write(OUTD_EEPROM_BASE+index, value);
  OUTD[index]=value;
  switch (value) {
   case 0: 
     digitalWrite(OUTPI[index], LOW);
    break;
   case 255:
     digitalWrite(OUTPI[index], HIGH);
    break;
   default:
     analogWrite(OUTPI[index], value);
  }; 
};
unsigned char GetAnalogOUT(unsigned char index){
  return OUTD[index];
};

#define INA00 0
#define INA01 1
#define INA02 2
#define INA03 3
#define INA04 6
#define INA05 7

byte INPI[6] = {
  INA00, INA01, INA02, INA03, INA04, INA05};
int INA[6];

void SetupAnalogIN(){
  for (byte i=0; i++; i<INOUTCOUNT) {
    pinMode(INPI[i], INPUT);     
  }
  readAnalogIn();
};

void readAnalogIn(){
  for (byte i=0; i++; i<INOUTCOUNT) {
    INA[i]=analogRead(INPI[i]);
  }
}

#define IND0 2 // window sensor
#define IND1 4 // door sensor
#define IND2 7 // movement sensor
unsigned int IND=0;
unsigned char INDI[3]={IND0, IND1,IND2};
void SetupDigitalIN(){
  pinMode(IND0, INPUT);     
  pinMode(IND1, INPUT);     
  pinMode(IND2, INPUT);     
};

#define SLAVE_ID 1
#define CTRL_PIN 13
#define BAUDRATE 9600

Modbus slave(SLAVE_ID, CTRL_PIN);

DHT dht(DHTPIN, DHTTYPE);
int dht_t;
int dht_h;

void cronEvery1min(){
  dht_t=dht.readTemperature();
  dht_h=dht.readHumidity();
  readAnalogIn();  
}

void writeOUT(byte i, byte value) {
  if (i<INOUTCOUNT) return;
  EEPROM.write(i, value);
  analogWrite(OUTPI[i], value);
} 

unsigned long diff_millis(unsigned long mStart, unsigned long mEnd)
{
  return mEnd-mStart;
}

void setup(){
  SetupAnalogOUT();
  SetupAnalogIN();
  SetupDigitalIN();
  dht.begin();
  
  slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
  slave.cbVector[CB_READ_COILS] = readDigitalIn;
  slave.cbVector[CB_READ_REGISTERS] = readAnalogIn;
  Serial.begin( BAUDRATE );
  slave.begin( BAUDRATE );  
};

unsigned long tmCron1m, tmCron01s;

void loop(){
  if (diff_millis(tmCron1m, millis()) > 1*60*1000) {
    tmCron1m=millis();
    //Call Cron routine
    cronEvery1min();
  };
  if (diff_millis(tmCron01s, millis()) > 100) {
    tmCron01s=millis();
    //Call Cron routine
  };
  // Call modbus poll
  slave.poll();
};

/**
 * Handel Force Single Coil (FC=05)
 * set digital output pins (coils) on and off
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
     digitalWrite(address, status);
}

/**
 * Handel Read Input Status (FC=02/01)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read digital input
    for (int i = 0; i < length; i++) {
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }
}

/**
 * Handel Read Input Registers (FC=04/03)
 * write back the values from analog in pins (input registers).
 */
void readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read analog input
    for (int i = 0; i < length; i++) {
      switch (address + i) {
        case mb_r_ro16:
          slave.writeRegisterToBuffer(i, dht_t);
          break;
        case mb_r_ro17:
          slave.writeRegisterToBuffer(i, dht_h);
          break;
        default:
          slave.writeRegisterToBuffer(i, 0);
          break;
      }
    }
}

