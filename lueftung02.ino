// Lueftung02 - Garage ventilation control with Atmega328
// Powered by 15V Wallwart, uC and sensors running at 3.3V 
// (via DSN-MINI-360 buck module from Aliexpress)
// using SHT21, DS18B20, a capacitive humidity sensor of own construction, 
// and an IRLI540N to switch two fans for intake and exhaust.
// (C) Copyright 2014 Sebastian Wangnick. All rights reserved.
// See http://s.wangnick.de/doku.php for design details.

// Layout
//                                          +-\/-+
//                    DTR - 680n - !RESET  1|o   |28  PC5 (A5,D19) SCL - SHT common SCL (passed on via SCLEN) - BC547C#1,2,3 Emitter
//                     TXD - RXD (D0) PD0  2|    |27  PC4 (A4,D18) SDA - SHT common SDA, 1k0 - SENS VCC
//                     RXD - TXD (D1) PD1  3|    |26  PC3 (A3,D17) - 100k - BC547C#1 Basis - Collector - SHT#1 Zuluft, 1k0 - SENS VCC
//                               (D2) PD2  4|    |25  PC2 (A2,D16) - 100k - BC547C#2 Basis - Collector - SHT#2 Hangwand, wallhumislave, 1k0 - SENS VCC 
// SENS VCC - 1k5 - DS18B20 DQ - (D3) PD3  5|    |24  PC1 (A1,D15) - 100k - BC547C#3 Basis - Collector - SHT#3 Abluft, 1k0 - SENS VCC 
//           SENS VCC via wire ! (D4) PD4  6|    |23  PC0 (A0,D14) - SENS VCC (auch fuer SHT21)
//                     MCP VOUT 3V3 - VCC  7|    |22  GND
//                          MCP GND - GND  8|    |21  AREF
//                                    PB6  9|    |20  AVCC - VCC, 100n - GND
//                                    PB7 10|    |19  PB5 (D13) SCK - ST7735 SCL [- Programmer]
//                               (D5) PD5 11|    |18  PB4 (D12) MISO [- Programmer]
//                               (D6) PD6 12|    |17  PB3 (D11) MOSI - ST7735 SDA [- Programmer]
//                               (D7) PD7 13|    |16  PB2 (D10) SS - ST7735 RS/DC
//                  ST7735 RES - (D8) PB0 14|    |15  PB1 (D9) - IRLI540N G 
//                                          +----+
// Fuses: efuse:0xFD hfuse:0xDA lfuse:0xE2 lock:0x0F
// BODLEVEL 2.7V, BOOTSZ 1024W_3C00, BOOTRST, ~CKDIV8, ~CKOUT, INTRCOSC_8MHZ_6CK_14CK_65MS
// Bootloader is ATmegaBOOT_168_atmega328_pro_8MHz.hex as of arduino-1.0.5 (unlock 0x3F)
// TODO: 

// SHT21 #-: Broken
// SHT21 #1 (Zuluft): 30011846901C8000
// SHT21 #2 (Hangwand): 330118B7151C8000
// SHT21 #3 (Abluft): ???
// [SHT21 #4 (Tuerwand): ???]

// TODO
// Watchdog
// Async?
// Timeouts?
// Use older values upon error, but watch their age and bail out if they get TOO old

#include <avr/pgmspace.h>

#define INVALDECI ((int16_t)0x8000)
#define INVALUINT ((uint16_t)0xFFFF)
#define SENSORVCC_PIN 14
#define CYCLE 5

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONEWIRE_PIN 3
#define TEMPERATURE_PRECISION 12
OneWire owbus(ONEWIRE_PIN);
DallasTemperature dsbus(&owbus);

enum {MITTEV, DECKEV, HEIZER, DSN};   
const DeviceAddress dsid[DSN] PROGMEM = {
    {0x28,0x29,0x5B,0x88,0x04,0x00,0x00,0x80}/*TO92*/,
    {0x28,0xAE,0x6C,0x87,0x04,0x00,0x00,0xA0}/*TO92*/,
    {0x28,0x13,0xE8,0x1A,0x05,0x00,0x00,0xEE}/*TO92*/,
//    {0x28,0xD8,0xD2,0xCD,0x04,0x00,0x00,0x9C}/*Rohr*/,
//    {0x28,0xB0,0x78,0xCE,0x04,0x00,0x00,0xF8}/*Rohr*/,
//    {0x28,0x8E,0xD3,0xCD,0x04,0x00,0x00,0xCD}/*Rohr*/
};
const char dsname0[] PROGMEM = "MitteV";
const char dsname1[] PROGMEM = "DeckeV";
const char dsname2[] PROGMEM = "Heizng";
typedef struct {
  int16_t temp;
} CompDsData;
CompDsData compDsData[DSN];

#include <Wire.h>
enum {SHT21_ADDR=0x40};
enum {SHT21_TEMPHOLD=0xE3, SHT21_HUMIHOLD=0xE5, SHT21_TEMPPOLL=0xF3, SHT21_HUMIPOLL=0xF5};
enum {AUSSEN, BODENV, BODENH, SHTN};
const char shtname0[] PROGMEM = "Zuluft";
const char shtname1[] PROGMEM = "Wand H";
const char shtname2[] PROGMEM = "Abluft";
const uint8_t shtpin[SHTN] PROGMEM = {17, 16, 15};
typedef struct {
  int16_t temp, humi, dewp;
} CompShtData;
CompShtData compShtData[SHTN];

enum {CAPHUM_ADDR=0x1C};
enum {CAPHUM_RESET=0x00};
enum {CAPHUMN = 1};
#define CAPHUM_MIN 10000
#define CAPHUM_MAX 40000
const uint8_t caphumpin[CAPHUMN] PROGMEM = {16};
const char caphumname0[] PROGMEM = "SteinH";
typedef struct {
  uint16_t period;
} CompCaphumData;
CompCaphumData compCaphumData[CAPHUMN];

enum {FANN = 1};
const uint8_t fanpin[FANN] PROGMEM = {9};
const char fanname0[] PROGMEM = "Lufter";
typedef struct {
  uint8_t on;
  uint32_t decisec;
} CompFanData;
CompFanData compFanData[FANN];

typedef enum {COMP_DS,COMP_SHT,COMP_CAPHUM,COMP_FAN,COMPTYPES} ComponentType;
typedef enum {SETUP, LOOP} ProcessorTask;
typedef void (*ComponentTypeProcessor)(uint8_t comp, uint8_t task);
ComponentTypeProcessor componentTypeProcessor[COMPTYPES] = {compDsProcessor, compShtProcessor, compCaphumProcessor, compFanProcessor};

enum {COMPONENTS = 8};   
typedef struct {
  const char* name_P;
  ComponentType type;
  uint8_t idx;
} Component;
Component component[COMPONENTS] = {
   {shtname0, COMP_SHT, 0},
   {shtname1, COMP_SHT, 1},
   {shtname2, COMP_SHT, 2},
   {dsname0, COMP_DS, 0},
   {dsname1, COMP_DS, 1},
   {dsname2, COMP_DS, 2},
   {caphumname0, COMP_CAPHUM, 0},
   {fanname0, COMP_FAN, 0},
};

// Checksum calculation, polynomial x^8+x^5+x^4+1. This is however not identical to _crc_ibutton_update!
uint8_t crcu (uint8_t crc, uint8_t data) {
    crc = crc ^ data;
    for (uint8_t i = 0; i<8; i++) {
        crc = crc&0x80? (crc<<1)^0x131: crc<<1;
    }
    return crc;
}

class MyHardwareSerial : public Print { public:
  uint8_t crc;
  virtual size_t write(uint8_t val) {
    crc = crcu(crc,val);
    return Serial.write(val);
  }
};
MyHardwareSerial MySerial;

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#define ST7735_RSDC_PIN 10
#define ST7735_RES_PIN 8
#define ST7735_CS_PIN 0

Adafruit_ST7735 Display(ST7735_CS_PIN,ST7735_RSDC_PIN,ST7735_RES_PIN);
#define FONTWIDTH 6
#define FONTHEIGHT 8
#define ROWS 16
#define COLUMNS 26
#define DATACOL (8*FONTWIDTH)

const uint16_t BG=Display.Color565(0x00,0x00,0x00), FG=Display.Color565(0xFF,0xFF,0xFF), 
    RED=Display.Color565(0xFF,0x40,0x40), YLW=Display.Color565(0xFF,0xFF,0x40), GRN=Display.Color565(0x40,0xFF,0x40);


int16_t dewp (int16_t relhumi, int16_t temp) {
    // http://www.mikrocontroller.net/topic/306226#3292459
    // http://wwwcaps.ou.edu/arpsbrowser/arps5.2.4browser/html_code/adas/mthermo.f90.html#DWPT
    //
    //    baker, schlatter  17-may-1982     original version.
    //
    //   this function returns the dew point (celsius) given the temperature
    //   (celsius) and relative humidity (%). the formula is used in the
    //   processing of u.s. rawinsonde data and is referenced in parry, h.
    //   dean, 1969: "the semiautomatic computation of rawinsondes,"
    //   technical memorandum wbtm edl 10, u.s. department of commerce,
    //   environmental science services administration, weather bureau,
    //   office of systems development, equipment development laboratory,
    //   silver spring, md (october), page 9 and page ii-4, line 460.

    if (relhumi==INVALDECI || temp==INVALDECI) return INVALDECI;
    float x = (1000-relhumi)/1000.0;
    float dpd = (14.55+0.0114*temp)*x + pow((2.5+0.0007*temp)*x,3) + (15.9+0.0117*temp)*pow(x,14);
    return temp-(int16_t)(dpd*10.0);
}

uint8_t i2c_read8crc (uint8_t addr, uint8_t* val8, uint8_t sendstop) {
    Wire.requestFrom(addr,(uint8_t)2,sendstop);
    val8[0] = Wire.read();
    return crcu(0,val8[0])!=Wire.read();
}

uint8_t i2c_read16crc (uint8_t addr, uint16_t* val16, uint8_t sendstop) {
    Wire.requestFrom(addr,(uint8_t)3,sendstop);
    uint8_t* val8 = (uint8_t*) val16;
    // AVR uses little endianness (lsb are first in storage, then msb)
    val8[1] = Wire.read();
    val8[0] = Wire.read();
    return crcu(crcu(0,val8[1]),val8[0])!=Wire.read();
}

char* printDeci (int16_t deci) {
  static char buffer[7];
  if (deci==INVALDECI) {
    strcpy_P(buffer,PSTR("ERROR"));
  } else {
    itoa(deci,buffer,10);
    uint8_t len = strlen(buffer);
    if (len>=5) {
      buffer[4]='.'; buffer[5]=0; 
      if (len==6) buffer[3]='k'; 
    } else {
      if (len<=1+(deci<0)) {buffer[len]=buffer[len-1]; buffer[len-1]='0'; len++;} // len is now >=2 (>=3 for negative)
      buffer[len]=buffer[len-1]; buffer[len-1]='.'; len++; buffer[len]=0; // len is now 3-5 (4-5 for negative)
      memmove(buffer+5-len,buffer,len);
      memset(buffer,' ',5-len);
    }
  }
  return buffer;
}

char* printUint (uint16_t uint) {
  static char buffer[7];
  if (uint==INVALUINT) {
    strcpy_P(buffer,PSTR("ERROR"));
  } else {
    utoa(uint,buffer,10);
    uint8_t len = strlen(buffer);
    if (len<5) {
      memmove(buffer+5-len,buffer,len);
      memset(buffer,' ',5-len);
    }
  }
  return buffer;
}
    
void compDsProcessor (uint8_t comp, uint8_t task) {
  if (task==SETUP) {
    Display.print(F("###.#"));
  } else { // task==LOOP
    uint8_t idx = component[comp].idx;
    DeviceAddress di;
    memcpy_P(di,dsid[idx],sizeof(di));
    int16_t temp = dsbus.getTemp(di);
    if (temp<=DEVICE_DISCONNECTED_RAW) {
      temp = INVALDECI;
    } else {
      temp = (temp+(temp>>2))>>4; // This is temp/128*10 without losing precision
    }
    compDsData[idx].temp = temp;
    char* txt = printDeci(temp);
    Display.print(txt);
    if (temp!=INVALDECI) {
      while (*txt==' ') txt++;
      MySerial.print(F("&c"));
      MySerial.print(comp);
      MySerial.print(F("t="));
      MySerial.print(txt);
    }
  }
}

void compShtProcessor (uint8_t comp, uint8_t task) {
  uint8_t idx = component[comp].idx;
  uint8_t pin = pgm_read_byte(shtpin+idx);
  if (task==SETUP) {
    Display.print(F("###.# ###.#% ###.#"));
    pinMode(pin,OUTPUT);
  } else { // task==LOOP
    digitalWrite(pin,HIGH);
    Wire.beginTransmission(SHT21_ADDR);
    Wire.write(SHT21_TEMPHOLD);
    Wire.endTransmission(false);
    uint16_t uval;
    int16_t val;
    uint8_t err;
    err = i2c_read16crc(SHT21_ADDR,&uval,false);
    val = (int16_t)(-468.5 + 1757.2 / 65536.0 * (float)uval);
    compShtData[idx].temp = err||val<-600||val>999? INVALDECI: val; 
    char* txt = printDeci(compShtData[idx].temp);
    Display.print(txt);
    if (compShtData[idx].temp!=INVALDECI) {
      while (*txt==' ') txt++;
      MySerial.print(F("&c"));
      MySerial.print(comp);
      MySerial.print(F("t="));
      MySerial.print(txt);
    }
    Wire.beginTransmission(SHT21_ADDR);
    Wire.write(SHT21_HUMIHOLD);
    Wire.endTransmission(false);
    err = i2c_read16crc(SHT21_ADDR,&uval,true);
    digitalWrite(pin,LOW);
    val = (int16_t)(-60.0 + 1250.0 / 65536.0 * (float)uval);
    compShtData[idx].humi = err||val<-200||val>1200? INVALDECI: val;
    compShtData[idx].dewp = dewp(compShtData[idx].humi,compShtData[idx].temp);
    txt = printDeci(compShtData[idx].humi);
    Display.write(' ');
    Display.print(txt);
    Display.write('%');
    if (compShtData[idx].humi!=INVALDECI) {
      while (*txt==' ') txt++;
      MySerial.print(F("&c"));
      MySerial.print(comp);
      MySerial.print(F("f="));
      MySerial.print(txt);
    }
    txt = printDeci(compShtData[idx].dewp);
    Display.write(' ');
    Display.print(txt);
  }
}

void compCaphumProcessor (uint8_t comp, uint8_t task) {
  uint8_t idx = component[comp].idx;
  uint8_t pin = pgm_read_byte(caphumpin+idx);
  if (task==SETUP) {
    Display.print(F("#####"));
    pinMode(pin,OUTPUT);
  } else { // task==LOOP
    digitalWrite(pin,HIGH);
    Wire.beginTransmission(CAPHUM_ADDR);
    Wire.write(CAPHUM_RESET);
    Wire.endTransmission(false);
    uint16_t uval;
    uint8_t err;
    err = i2c_read16crc(CAPHUM_ADDR,&uval,true);
    digitalWrite(pin,LOW);
    compCaphumData[idx].period = err||uval<CAPHUM_MIN||uval>CAPHUM_MAX? INVALUINT: uval;
    char* txt = printUint(compCaphumData[idx].period);
    Display.print(txt);
    if (compCaphumData[idx].period!=INVALUINT) {
      while (*txt==' ') txt++;
      MySerial.print(F("&c"));
      MySerial.print(comp);
      MySerial.print(F("f="));
      MySerial.print(txt);
    }
  }
}

void compFanProcessor (uint8_t comp, uint8_t task) {
  uint8_t idx = component[comp].idx;
  uint8_t pin = pgm_read_byte(fanpin+idx);
  if (task==SETUP) {
    compFanData[idx].on = 0;
    pinMode(pin,OUTPUT);
  } else { // task==LOOP
    compFanData[idx].on = compShtData[AUSSEN].temp!=INVALDECI && compShtData[AUSSEN].humi!=INVALDECI 
        && compShtData[BODENV].temp!=INVALDECI && compShtData[BODENV].humi!=INVALDECI
        && compShtData[BODENV].humi>600 
        && compShtData[AUSSEN].dewp+10<compShtData[BODENV].dewp 
        && (compShtData[BODENV].temp>100 || compShtData[AUSSEN].temp>compShtData[BODENV].temp);
    digitalWrite(pin,compFanData[idx].on?HIGH:LOW);
    MySerial.print(F("&fan="));
    MySerial.print(compFanData[idx].on);
  }
  Display.print(compFanData[idx].on?F("An "):F("Aus"));
}

void setup(void) {
  // Go to full speed (8MHz internal OSC in our case)
  CLKPR = 1<<CLKPCE;
  CLKPR = 0;

  Serial.begin(115200);
  Serial.println(F("setup(" __FILE__ ", " __DATE__ " " __TIME__ ")"));
  Display.initR(INITR_REDTAB);
  Display.setRotation(1);
  Display.fillScreen(BG);
  Display.setTextColor(FG,BG);
  Display.setCursor(0,(ROWS-3)*FONTHEIGHT); Display.print(F("Source: ")); Display.print(F(__FILE__));
  Display.setCursor(0,(ROWS-2)*FONTHEIGHT); Display.print(F("Date:   ")); Display.print(F(__DATE__ " " __TIME__));
  Display.setCursor(0,(ROWS-1)*FONTHEIGHT); Display.print(F("Uptime: "));
  pinMode(SENSORVCC_PIN,OUTPUT);
  Wire.begin();
  for (uint8_t comp=0; comp<COMPONENTS; comp++) {
    Display.setCursor(0,(comp+1)*FONTHEIGHT);
    Display.print((const __FlashStringHelper *)component[comp].name_P);
    Display.print(F(": "));
    componentTypeProcessor[component[comp].type](comp,SETUP);
  }
}

#define hex(digit) ((digit)+((digit)>9?'A'-10:'0'))

#define SECONDSPERDAY ((uint32_t)24*60*60)
#define SECONDSPERHOUR ((uint32_t)60*60)
#define SECONDSPERMINUTE ((uint32_t)60)

void loop(void) {
  static uint32_t uptime = 0;
  static uint32_t lasttime = 0;
  while (millis()-lasttime<1000*CYCLE) continue;
  lasttime += 1000*CYCLE;
  uptime += CYCLE;
  Display.setCursor(DATACOL,(ROWS-1)*FONTHEIGHT); 
  uint32_t seconds = uptime;
  uint16_t val = seconds/SECONDSPERDAY;
  Display.print(val);
  Display.write(',');
  seconds -= val*SECONDSPERDAY;
  val = seconds/SECONDSPERHOUR;
  if (val<=9) Display.write('0');
  Display.print(val);
  Display.write(':');
  seconds -= val*SECONDSPERHOUR;
  val = seconds/SECONDSPERMINUTE;
  if (val<=9) Display.write('0');
  Display.print(val);
  Display.write(':');
  seconds -= val*SECONDSPERMINUTE;
  val = seconds;
  if (val<=9) Display.write('0');
  Display.print(val);
  Serial.write('\t');
  MySerial.crc = 0;
  digitalWrite(SENSORVCC_PIN,HIGH);
  delay(15);
  dsbus.begin(); // This searches for and establishes the number of devices
  dsbus.setResolution(TEMPERATURE_PRECISION);
  dsbus.requestTemperatures(); // This will block for 750ms (at 12 bit resolution)
  for (uint8_t comp=0; comp<COMPONENTS; comp++) {
    Display.setCursor(DATACOL,(comp+1)*FONTHEIGHT);
    componentTypeProcessor[component[comp].type](comp,LOOP);
  }
  digitalWrite(SENSORVCC_PIN,LOW);
  Serial.write('\r');
  Serial.write(hex(MySerial.crc>>4));
  Serial.write(hex(MySerial.crc&0xF));
  Serial.write('\n');
}
