/* This file started from Akeru library http://akeru.cc copyleft Snootlab, 2014
 and has been modified for HidnSeek by Stephane D, 2014.

 This library is free software: you can redistribute it and/or
 modify it under the terms of the GNU General Public License as published
 by the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with HidnSeek.  If not, see <http://www.gnu.org/licenses/>.*/

#include <Arduino.h>
#include "HidnSeek.h"

HidnSeek::HidnSeek(uint8_t rxPin, uint8_t txPin) :
    _serial(rxPin, txPin) {
     //Since _lastSend is unsigned, this is infinity
    _lastSend = -1;
}

HidnSeek::~HidnSeek() {
}

int HidnSeek::begin() {
    _serial.begin(9600);

    //Remove un-ended commands from TST's buffer
    _serial.write((uint8_t)'\0');
    _serial.write((uint8_t)';');

    char dataRX[5] = "";
    int  length    = 3;
    int  timeout   = 50;
    int  bread     = 0;

    long previousMillis = millis();
    while((millis()-previousMillis)<timeout)
    {
        if(_serial.available()>0)
        {
            char c = _serial.read();
            if(bread>length)
            {
                return(-1); // string received is too long
            }
            else
            {
                dataRX[bread]=c;
            }
            bread++;
        }
    }
    if (strcmp (dataRX,"KO;") == 0) return(bread); else return(-1);
}

bool HidnSeek::isReady() {

	// IMPORTANT WARNING. PLEASE READ BEFORE MODIFYING THE CODE
	//
	// The Sigfox network operates on public frequencies. To comply with
	// radio regulation, it can send radio data a maximum of 1% of the time
	// to leave room to other devices using the same frequencies.
	//
	// Sending a message takes about 6 seconds (it's sent 3 times for
	// redundancy purposes), meaning the interval between messages should
	// be 10 minutes.
	//
	// Also make sure your send rate complies with the restrictions set
	// by the particular subscription contract you have with your Sigfox
	// network operator.
	//
	// FAILING TO COMPLY WITH THESE CONSTRAINTS MAY CAUSE YOUR MODEM
	// TO BE BLOCKED BY YOUR SIFGOX NETWORK OPERATOR.
	//
	// You've been warned!

    unsigned long currentTime = millis();
    if(currentTime >= _lastSend && ((unsigned long)(currentTime - _lastSend) <= 600000UL)) {
        return false;
    }

    // Time is ok, ask the modem's status
    _serial.write((uint8_t)'\0');
    _serial.write((uint8_t)'S');
    _serial.write((uint8_t)'F');
    _serial.write((uint8_t)'P');
    _serial.write((uint8_t)';');

    return _nextReturn() == OK;
}

bool HidnSeek::send(const void* data, uint8_t len) {
	uint8_t* bytes = (uint8_t*)data;

/*    if(!isReady()) {
        return false;
    } */

    // See comment in isReady()
    _lastSend = millis();

    _serial.write((uint8_t)'\0');
    _command(PSTR("SFM"));
    _serial.write(len);
    for(uint8_t i = 0; i < len; ++i) {
        _serial.write(bytes[i]);
    }
    _serial.write(';');

    uint8_t ok = _nextReturn();
    if(ok == OK) {
        _nextReturn(); //SENT
        return true;
    }
    return false;
}

uint8_t HidnSeek::getRev() {
    _serial.write((uint8_t)'\0');
    _serial.write((uint8_t)'S');
    _serial.write((uint8_t)'F');
    _serial.write((uint8_t)'v');
    _serial.write((uint8_t)';');

    while(_serial.available()<3);

    if(_serial.peek() == 'K') {
        _serial.read(); //'K'
        _serial.read(); //'O'
        _serial.read(); //';'
        return 0;
    } else {
        while(_serial.available()<5);
        uint8_t rev = 10 * (_serial.read() - '0') + (_serial.read() - '0');

        _serial.read(); //'O'
        _serial.read(); //'K'
        _serial.read(); //';'

        return rev;
    }
}

void HidnSeek::_command (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    _serial.print(c);
  }
}

void HidnSeek::flashRed(int num) {
  while (num > 0) {
    PORTD |= (1 << redLEDpin);
    delay(50);
    PORTD &= ~(1 << redLEDpin);
    if (--num) delay(50);
  }
}

void HidnSeek::NoflashRed() {
  delay(25);
  PORTD &= ~(1 << redLEDpin) & ~(forceSport << bluLEDpin);
  delay(50);
  PORTD |= (1 << redLEDpin) | (forceSport << bluLEDpin);
}

void HidnSeek::serialString (PGM_P s) {
  char c;
  while ((c = pgm_read_byte(s++)) != 0)
    Serial.print(c);
}

unsigned long HidnSeek::getID() {
    _serial.write((uint8_t)'\0');
    _command(PSTR("SFID;"));

    //Response is [byte1, byte2, ..., byteN, 'O', 'K']
    uint8_t response[8] = {0};
    uint8_t i = 0;
    while(!_serial.available());
    while(_serial.peek() != ';' || i<6) {
        response[i] = _serial.read();
        while(!_serial.available());
        ++i;
    }
    _serial.read(); //';'

    unsigned long id = 0;

    for(uint8_t j = 0; j < i-2; ++j) {
        id = (id << 8) + response[j];
    }

    return id;
}

//Power value:
bool HidnSeek::setPower(uint8_t power) {
    // 13,9dBm with 0,4,47 for the parameter
    if (power > 0) _command(PSTR("AT$MT=0,4,47")); else {
      _command(PSTR("ATZ"));
    }
    _serial.println();

    char dataRX[5] = "";
    int  length    = 3;
    int  timeout   = 50;
    int  bread     = 0;

    long previousMillis = millis();
    while((millis()-previousMillis)<timeout)
    {
        if(_serial.available()>0)
        {
            char c = _serial.read();
            if(bread>length)
            {
                return(-1); // string received is too long
            }
            else
            {
                dataRX[bread]=c;
            }
            if (c == 'O') bread++;
            if (c == 'K') break;
        }
    }
    if (strcmp (dataRX,"OK") != 0) return false; else return true;
}

uint8_t HidnSeek::_nextReturn() {
    while(!_serial.available()) if (millis()-_lastSend > 2000) PORTD &= ~(1 << 7);
    char fstChar = _serial.read();
    while(_serial.read() != ';');
    return fstChar;
}

int HidnSeek::freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void HidnSeek::setSupply(boolean shd) {
    if (shd) {
        PORTB |= (1 << 1); //Pin 1 of portb is now confirm supply
        DDRB |= B00000010; //Pin 1 of portb is an output
    }
    else {
        PORTC = 0;
        DDRC = 0;
        PORTD = 0;
        DDRD = 0;
        PORTB = 0;
        DDRB = B00000010;
    }
}

void HidnSeek::changeCurrent500mA(boolean current){
    if(current) digitalWrite(chg500mA, HIGH);
    else digitalWrite(chg500mA, LOW);
}

void HidnSeek::initGPIO() {
    // PORTB = B00111010;
    // DDRB  = B00000111;
    // DDRC  = B01000101;
    // PORTC = B00000000;
    // if (discret) {
    //     DDRD  = B00000010;
    //     PORTD = B00011000;
    // }
    // else {
    //     DDRD  = B11000010;
    //     PORTD = B00011000;
    // }
    PORTB = (DIGITAL_PULLUP >> 8) & 0xff;
    DDRB  = (DIGITAL_OUTPUT >> 8) & 0xff;
    PORTC = 0x00;
    DDRC  = DDRC_MASK;
    PORTD = DIGITAL_PULLUP & 0xff;
    DDRD  = DIGITAL_OUTPUT & 0xff;
}

void HidnSeek::checkBattery() {
    if (millis() - _lastCheck > 2000UL) {
        _lastCheck = millis();
        boolean _grst = PORTB & (1<<2);
        PORTB |= (1 << 2);
        if (!_grst) delay(50);
        unsigned int value = analogRead(A1);
        if (value < 900) setSupply(false);   // Below 3,7V shutdown the supply
        if (!_grst) PORTB &= ~(1<<2);
    }
}

/* GPS FUNCTIONS */

void HidnSeek::gpsCmd(PGM_P s) {
  int XOR = 0;
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    Serial.print(c);
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  if (XOR < 0x10) Serial.print("0");
  Serial.println(XOR, HEX);
}

bool HidnSeek::initGPS()
{
  boolean GPSready = false;
  digitalWrite(rstPin, HIGH);
  unsigned long startloop = millis();
  while ((uint16_t) (millis() - startloop) < 6000 ) {
    if (Serial.available() > 0 && Serial.read() == '*') {
      GPSready = true;
      break;
    }
    delay(100);
  }
  if (GPSready) {
    gpsCmd(PSTR(PMTK_SET_NMEA_OUTPUT));
    gpsCmd(PSTR(PMTK_SET_NMEA_UPDATE_1HZ));   // 1 Hz update rate
  } else digitalWrite(rstPin, LOW);
  return GPSready;
}

void HidnSeek::gpsStandby() {
  GPSactive = false;
  digitalWrite(rstPin, LOW);
}

bool HidnSeek::gpsProcess()
{
  boolean newGpsData = false;
  boolean newSerialData = false;
  float distance;
  unsigned long start = millis();
  unsigned int waitime = 2000;
  // Parse GPS data for 2 second
  while ((uint16_t) (millis() - start) < waitime)
  {
    if (Serial.available() > 0) {
      newSerialData = true;
      waitime = 100;
      start = millis();
      redLEDon;
    }
    while (Serial.available())
    {
      char c = Serial.read();
      // New valid NMEA data available
      if (gps.encode(c))
      {
        newGpsData = true;
      }
    }
  }

  // Check if NMEA packet received, wake up GPS otherwise
  if (!newSerialData) initGPS();

  // 12 octets = 96 bits payload
  // lat: 32, lon: 32, alt: 13 , spd: 7, cap: 2, bat: 7, mode: 3 (0-3 sat view, more is MSG)
  // lat: 32, lon: 32, alt:0-8191m, spd:0-127Km/h, bat:0-100%, mode:0-7, cap: N/E/S/W
  // int is 16 bits, float is 32 bits. All little endian

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);

  if (newGpsData) { // computeData
    gps.f_get_position(&p.lat, &p.lon, &fix_age);
    sat = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    alt = abs(round(gps.f_altitude()));
    spd = round(gps.f_speed_kmph());
    syncSat += sat;
    noSat = 0;
  }
  else noSat++;

  //printData(newGpsData); // For debug purpose this use 2Ko of flash program

  redLEDoff;
  return newSerialData;
}

void HidnSeek::print_date()
{
  char sz[24];
  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
    month, day, year, hour, minute, second);
  Serial.print(sz);
}

void HidnSeek::printData(bool complete) {
  print_date();
  serialString(PSTR("fix="));
  Serial.print(fix_age);
  if (complete) {
    serialString(PSTR(", lat="));
    Serial.print(p.lat, 7);
    serialString(PSTR(", lon="));
    Serial.print(p.lon, 7);
    serialString(PSTR(", alt="));
    Serial.print(alt);
    serialString(PSTR(", cap="));
    Serial.print((gps.course() / 90) % 4);
    serialString(PSTR(", spd="));
    Serial.print(spd);
    serialString(PSTR(", sat="));
    Serial.print(sat);
  }
  if (GPSactive) serialString(PSTR(", GPS "));
  if (forceSport) {
    serialString(PSTR(", sport="));
    Serial.print(limitSport);
  }
  serialString(PSTR(", bat="));
  Serial.print(batteryPercent);
  serialString(PSTR("%, noSat="));
  Serial.print(noSat);
  serialString(PSTR(", syncSat="));
  Serial.println(syncSat);
}

void HidnSeek::makePayload() {
  uint8_t cap;
  if (sat > 3) {
    if (spd > 120 && alt > 1250) {
      airPlaneSpeed = true;  // Abort GPS message if Airplane detected
      syncSat = 255;
    }
    if (spd < 120) airPlaneSpeed = false;

    if (alt > 4096) alt = (uint16_t)(alt / 16) + 3840; // 16m step after 4096m
    if (alt > 8191) alt = 8191;                        // 69632m is the new limit ;)

    if (spd > 127) spd = (uint16_t)(spd / 16) + 94; // 16Km/h step after 127Km/h
    else if (spd > 90) spd = (uint16_t)(spd / 3) + 60; // 3Km/h step after 90Km/h
    if (spd > 126) spd = 127;      // limit is 528Km/h
    cap = (gps.course() / 90) % 4;
  } else cap = (accelPosition < 3) ? accelPosition : 3;

  p.cpx = (uint32_t) alt << 19;
  p.cpx |= (uint32_t) spd << 12; // send in Km/h
  p.cpx |= (uint32_t) cap << 10;  // send N/E/S/W
  p.cpx |= (uint32_t) ( 127 & batteryPercent) << 3; // bat (7bits)
  if (sat > 8) sat = 8;
  p.cpx |= (uint32_t) 3 & (sat / 4); // sat range is 0 to 14
}

void HidnSeek::decodPayload() {
  unsigned int alt_ = p.cpx >> 19;
  unsigned int cap_ = (p.cpx >> 10) & 3;
  unsigned int spd_ = (p.cpx >> 12) & 127;
  unsigned int bat_ = (p.cpx >> 3) & 127;
  unsigned int mod_ = p.cpx & 7;
  print_date();
  serialString(PSTR("msg="));
  Serial.print(MsgCount);
  serialString(PSTR(" lat="));
  Serial.print(p.lat, 7);
  serialString(PSTR(", lon="));
  Serial.print(p.lon, 7);
  serialString(PSTR(", alt="));
  Serial.print(alt_);
  serialString(PSTR(", cap="));
  Serial.print(cap_);
  serialString(PSTR(", spd="));
  Serial.print(spd_);
  serialString(PSTR(", bat="));
  Serial.print(bat_);
  serialString(PSTR(", mode="));
  Serial.println(mod_);
}

/* Sigfox Functions */

bool HidnSeek::initSigFox() {
  serialString(PSTR("SigFox: "));
  unsigned long previousMillis = millis();
  while ((uint16_t) (millis() - previousMillis) < 6000) {
    if (begin() == 3) {
      Serial.println(getID(), HEX);
      return true;
    }
    else delay(200);
  }
  serialString(PSTR("Fail\r\n"));
  return false;
}

void HidnSeek::sendSigFox(byte msgType) {
  PORTD |= (1 << redLEDpin);
  // isReady check removed in the library due to reset of millis during sleep time
  makePayload();
  if (msgType > 0) {
    if (baromPresent) {
      bmp180Measure(&Temp, &Press);
      if (Press > 1000) airPlaneSpeed = false;
    } else {
      Temp = 20;
      Press = 1030;
    }
    if (msgType == MSG_NO_MOTION) {
      p.lat = Temp;
      p.lon = Press;
    }
    p.cpx &= ~(7 << 0);
    p.cpx |= (uint32_t) (  7 & msgType); // mode (2bits)
  }
  decodPayload();
  unsigned long previousMillis = millis();
  if ( !(msgType > 0 && airPlanePress) && !airPlaneSpeed) {
    send(&p, sizeof(p));
    stepMsg(); // Init the message number per day and time usage counters
    while ((uint16_t) (millis() - previousMillis) < 6000) delay(100);
  }
  PORTD &= ~(1 << redLEDpin);
}

/* Barometer */

bool HidnSeek::initBaro(void)
{
    return (bmp180.init());
}

bool HidnSeek::bmp180Measure(float *Temp, unsigned int *Press)
{
  if (Temp) *Temp = bmp180.bmp085GetTemperature(bmp180.bmp085ReadUT()) - 3.6;
  if (Press) {
    *Press = (unsigned int) (bmp180.bmp085GetPressure(bmp180.bmp085ReadUP()) / 100);
    airPlanePress = *Press < 880 ? true : false;
  }
  return true;
}

void HidnSeek::bmp180Print()
{
  serialString(PSTR("Temp: "));
  Serial.print(Temp, 2);
  serialString(PSTR("'C Press: "));
  Serial.print(Press);
  serialString(PSTR("mb\r\n"));
}

/* EEPROM */

void HidnSeek::stepMsg()
{
  if (day > 0) {
    if (day != today) { // Reset or new day
      today = EEPROM.read(ADDR_TODAY);
      if (today != day) { // New day
        saveEEprom();
        EEPROM.write(ADDR_TODAY, day);
        EEPROM.write(day, 0);
        today = day;
        MsgCount = 0;
      }
    }
    MsgCount++;
  } else MsgCount = 0;
}

void HidnSeek::saveEEprom() {
  if (today > 0 && today < 32) {
    EEPROM.write(today, MsgCount);
  }
}

void HidnSeek::dumpEEprom() {
  // Display the last day used by the tracker
  today = EEPROM.read(ADDR_TODAY);
  if (today == 255) {
    today = 0;
    EEPROM.write(ADDR_TODAY, 0);;
  }
  serialString(PSTR("Last day:"));
  Serial.println(today);
  // Display numbers of messages sent to sigfox network on day 1, 2, 3, ... , 30, 31
  serialString(PSTR("Usage per days:"));
  for (int i = ADDR_SENT; i < ADDR_SENT + 31; i++) {
    MsgCount = EEPROM.read(i);
    if (MsgCount == 255) {
      EEPROM.write(i, 0);
      MsgCount = 0;
    }
    Serial.print(MsgCount);
    Serial.print(" ");
  }
  MsgCount = EEPROM.read(today);
  Serial.println();
}

/* Accelerometer */

bool HidnSeek::initMems() {
  serialString(PSTR("Init Mems:"));
  Wire.begin();
  accel.begin(false, ACCEL_MODE);
  if (accel.update() != 0) {
    serialString(PSTR("Fail"));
    Serial.println();
    return false;
  }
  else {
    serialString(PSTR("OK"));
    Serial.println();
    return true;
  }
}

bool HidnSeek::accelStatus() {
  static int8_t x, y, z;
  static byte seq;
  if (accelPresent == false) return true;

  boolean accelMove = false;
  byte error = accel.update();

  if (error != 0) accelMove = true;
  else {
    accelMove = ((uint8_t)(abs((int8_t)(accel.getX() - x))) > 2) ? true :
                ((uint8_t)(abs((int8_t)(accel.getY() - y))) > 2) ? true :
                ((uint8_t)(abs((int8_t)(accel.getZ() - z))) > 2) ? true : false;
    x = accel.getX();
    y = accel.getY();
    z = accel.getZ();

    if (accelMove) { // Compute accelDance on move
      byte newPosition;
      if      (abs(x) < ACCEL_FLAT && abs(y) < ACCEL_FLAT && z > ACCEL_TRIG) newPosition = POS_FACE_UP;
      else if (abs(x) < ACCEL_FLAT && abs(y) < ACCEL_FLAT && z < -ACCEL_TRIG) newPosition = POS_FACE_DN;
      else if (abs(x) < ACCEL_FLAT && y < -ACCEL_TRIG && abs(z) < ACCEL_FLAT) newPosition = POS_SIDE_UP;
      else if (abs(x) < ACCEL_FLAT && y > ACCEL_TRIG && abs(z) < ACCEL_FLAT) newPosition = POS_SIDE_DN;
      else if (x > ACCEL_TRIG && abs(y) < ACCEL_FLAT && abs(z) < ACCEL_FLAT) newPosition = POS_SIDE_RT;
      else if (x < -ACCEL_TRIG && abs(y) < ACCEL_FLAT && abs(z) < ACCEL_FLAT) newPosition = POS_SIDE_LT;
      else {
        seq = 0;
        newPosition = POS_NULL;
      }
      if (seq == 0 && accelPosition == POS_FACE_UP && newPosition == POS_SIDE_UP) {
        seq++;
        NoflashRed();
      }
      else if (seq == 1 && accelPosition == POS_SIDE_UP && newPosition == POS_SIDE_DN) {
        seq++;
        NoflashRed();
      }
      else if (seq == 2 && accelPosition == POS_SIDE_DN && newPosition == POS_SIDE_UP) {
        seq++;
        NoflashRed();
      }
      else if (seq == 2 && accelPosition == POS_SIDE_DN && newPosition == POS_FACE_UP) {
        seq += 2;
        NoflashRed();
      }
      else seq = 0;
      accelPosition = newPosition;
      if (seq == 3) { // seq == 3
        saveEEprom();
        flashRed(20);
        PORTC = 0;
        DDRC = 0;
        PORTD = 0;
        DDRD = 0;
        PORTB = 0;
        DDRB = B00000010;
        seq = 0;
      }
      if (seq == 4) { // Places the device into sports mode
        if (MsgCount < 90) {
          forceSport = 1 - forceSport;
          limitSport = 0;
          if (forceSport && !GPSactive) initGPS();
          flashRed(8);
        }
        seq = 0;
      }
    }
  }
  return accelMove;
}

void HidnSeek::initSense() {
  byte lowByte = EEPROM.read(ADDR_CAL_LOW);
  byte highByte = EEPROM.read(ADDR_CAL_HIGH);
  sensorMax = ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
  serialString(PSTR("SensorMax: "));
  Serial.print(sensorMax);
  sensorMax = 980;
}

unsigned int HidnSeek::calibrate(unsigned int sensorValue) {
  // record the maximum sensor value
  byte lowByte = ((sensorValue >> 0) & 0xFF);
  byte highByte = ((sensorValue >> 8) & 0xFF);
  EEPROM.write(ADDR_CAL_LOW, lowByte);
  EEPROM.write(ADDR_CAL_HIGH, highByte);
  return sensorValue;
}

bool HidnSeek::batterySense() {
  digitalWrite(rstPin, HIGH);
  analogReference(EXTERNAL);
  delay(100); // RC need 30ms
  // read multiple values and sort them to take the median value. Require 24ms
  uint8_t sortedValues[NUM_READS];
  for (uint8_t i = 0; i < NUM_READS; i++) {
    uint8_t value = analogRead(sensorBatt) >> 2;
    uint8_t j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (uint8_t k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  batteryValue = 0;
  //return scaled mode of 3 values
  for (uint8_t i = NUM_READS / 2 - 4; i < (NUM_READS / 2 + 4); i++) {
    batteryValue += sortedValues[i];
  }
  batteryValue = batteryValue >> 1;

  unsigned int batteryComp = batteryValue - 4; // remove 16mV
  if (batteryComp > sensorMax) sensorMax = calibrate(batteryComp);
  unsigned int bat = map(batteryValue, 0, sensorMax, 0, BATT_MAX); // represent the battery voltage
  batteryPercent = map(bat, min(bat, BATT_MIN), max(bat, BATT_MAX), 0, 100);
  digitalWrite(rstPin, (forceSport || GPSactive) ? HIGH : LOW);
  return (bat < BATT_MIN);
}

void HidnSeek::shutdownSys() { // 3.57V on battery voltage
  digitalWrite(rstPin, LOW);
  serialString(PSTR("Low Bat: "));
  saveEEprom();
  sendSigFox(MSG_WEAK_BAT);
  digitalWrite(shdPin, LOW);
  delay(500);
}
