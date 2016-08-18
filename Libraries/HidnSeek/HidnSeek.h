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

#ifndef HIDNSEEK_H
#define HIDNSEEK_H

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "TinyGPS.h"
#include "LowPower.h"
#include "Barometer.h"
#include "EEPROM.h"
#include "MMA8653.h"
/****************** ATMega328p pin values *******************************/

#define rxGPS            0     // PD0 RX Serial from GPS
#define txGPS            1     // PD1 TX Serial to GPS
#define usbDP            2     // PD2 Shutdown supply after power off
#define accINT           3     // PB0 Accelerometer Interruption
#define usbDM            4     // PD4
#define txSigfox         5     // PD5 TX Serial to Sigfox modem
#define bluLEDpin        6     // PD6 Piezzo Output
#define redLEDpin        7     // PD7 Red LED Status
#define rxSigfox         8     // PD3 RX Serial from Sigfox modem
#define shdPin           9     // PB1 Shutdown pin
#define rstPin          10     // PB2 SS   SDCARD
//                      11     // PB3 MOSI SDCARD
//                      12     // PB4 MISO SDCARD
//                      13     // PB5 SCK  SDCARD
#define sensorA0        A0     // PC0 VUSB present
#define sensorBatt      A1     // PC1 battery voltage
#define chg500mA        A2
#define satLEDpin       A3
#define sensorA4        A4     // PC4 A4 SDA
//                      A5     // PC5 A5 SCL
#define sensorA6        A6     // PC6
#define chgFLAG         A7

#define blueLEDon  PORTD |= (1 << bluLEDpin)
#define blueLEDoff PORTD &= ~(1 << bluLEDpin)
#define redLEDon   PORTD |= (1 << redLEDpin)
#define redLEDoff  PORTD &= ~(1 << redLEDpin)

/****************** Pins output values *******************************/

#define DIGITAL_PULLUP ((1 << shdPin) | (1 << accINT) | (1 << usbDP) | (1 << usbDM))

/****************** Pins direction ***********************************/

#define DDRC_MASK (1 << 2)
#define DIGITAL_OUTPUT ((1 << shdPin) | (1 << redLEDpin) | (1 << bluLEDpin) | (1 << rxSigfox) | (1 << rstPin))

/****************** GPS Commands *******************************/

#define PMTK_SET_NMEA_OUTPUT  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*" // 28"
#define PMTK_AWAKE   "$PMTK010,001*" // 2E"
#define PMTK_STANDBY "$PMTK161,0*"   // 28"
#define PMTK_VERSION "$PMTK605*"     // 31"
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*" // 1F"
#define PMTK_ENABLE_SBAS "$PMTK313,1*" // 2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*" // 2E"

/****************** Accelerometer *******************************/

#define ACCEL_MODE 2        // 2G scale for the accelerometer
#define ACCEL_TRIG 40
#define ACCEL_FLAT 10
#define SPORT_LIMIT 48      // 4h limit sport duration
#define PERIOD_LOOP 10      // number of minutes between messages, must be > 10
#define MOTION_MIN_NUMBER 2
#define PERIOD_COUNT ((PERIOD_LOOP * 15) >> 1)

/****************** Battery *******************************/

#define BATT_MIN 3570
#define BATT_MAX 4200
#define NUM_READS 100

/****************** EEPROM Map *******************************/

#define ADDR_TODAY     0
#define ADDR_SENT      1 // byte01-31: number of messages sent
#define ADDR_CAL_LOW  32 // byte32-33: battery calibration
#define ADDR_CAL_HIGH 33

/****************** Barometer *******************************/

#define ALTITUDE 252.0 // Altitude of HidnSeek's HQ in Grenoble in meters

/*********************************************************************/

typedef enum {
  POS_FACE_UP = 0, POS_FACE_DN = 1, POS_SIDE_UP = 2,
  POS_SIDE_DN = 3, POS_SIDE_RT = 4, POS_SIDE_LT = 5,
  POS_NULL = 6
} positions;

typedef enum {
  MSG_POSITION = 0, MSG_OPTION = 3, MSG_NO_MOTION = 4, MSG_NO_GPS = 5,
  MSG_MOTION_ALERT = 6, MSG_WEAK_BAT = 7
} msgs;

typedef enum {
    OK = 'O',
    KO = 'K',
    SENT = 'S'
} RETURN_CODE;

extern boolean GPSactive;
extern int year;
extern byte month, day, hour, minute, second, hundredths;
extern uint16_t alt, spd;
extern uint8_t  sat, syncSat, noSat;
extern unsigned long fix_age;

struct Payload {
  float lat;
  float lon;
  uint32_t cpx;
};

extern struct Payload p;

extern uint8_t forceSport;
extern uint8_t limitSport;

extern uint8_t loopGPS;

extern float    Temp;
extern uint16_t Press;

extern boolean accelPresent;
extern boolean baromPresent;
// For automatic airplane mode detection
extern boolean airPlaneSpeed;
extern boolean airPlanePress;

extern byte batteryPercent;

extern int8_t   detectMotion;

extern unsigned long start;

extern uint16_t batteryValue;

extern byte     accelPosition;

extern uint8_t  today;
extern uint8_t  MsgCount;

void serialString (PGM_P s);


/*********************************************************************/

class HidnSeek {
    public:
        HidnSeek(uint8_t rxPin, uint8_t txPin);
        ~HidnSeek();
        int begin();
        void initGPIO();
        /* Power Management Fuctions */
        bool setPower(uint8_t power);
        void setSupply(boolean shd);
        void changeCurrent500mA(boolean current);
        void checkBattery();
        /* ATMega328p Fuctions */
        int freeRam();
        void serialString(PGM_P s);
        void flashRed(int num);
        void NoflashRed();
        /* HidnSeek Modes */

        /* SIGFOX Functions */
        bool send(const void* data, uint8_t len);
        bool isReady();
        unsigned long getID();
        uint8_t getRev();
        bool initSigFox();
        void sendSigFox(byte msgType);
        /* GPS Functions */
        /* Accelerometer Functions */
        bool initMems();
        bool accelStatus();
        /* Barometer Functions */
        bool bmp180Measure(float *Temp, unsigned int *Press);
        void bmp180Print();
        bool initBaro(void);
        /* EEPROM Functions */
        void saveEEprom();
        void dumpEEprom();
        /* Battery Functions */
        void initSense();
        unsigned int calibrate(unsigned int sensorValue);
        bool batterySense();
        void shutdownSys();

    private:
        /* SIGFOX Variables */
        SoftwareSerial _serial;
        unsigned long _lastSend, _lastCheck;
        uint8_t _rxPin;
        uint8_t _txPin;
        uint8_t loopGPS = 0;

        /* Battery */
        unsigned int sensorMax;


        void stepMsg();

        uint8_t _nextReturn();
        void _command(PGM_P s);


        // BMP180 measurements

};

class GPS {
    public:
        GPS();
        bool initGPS();
        void gpsCmd(PGM_P s);
        void gpsStandby();
        bool gpsProcess();
        void makePayload();
        void decodPayload();

    private:
        TinyGPS gps;
        MMA8653 accel;
        Barometer bmp180;
        void printDate();
        void printData(bool complete);

};

#endif
