/*  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  th  e Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  HidnSeek by StephaneD 2015 03 13 Not for commercial use              */

#define FILE "HidnSeek_v3_28"
#include "EEPROM.h"
#include "LowPower.h"
#include "HidnSeek.h"
#include "def.h"

HidnSeek HidnSeek(txSigfox, rxSigfox);

int powerDownLoop(int msgs) {
  if (HidnSeek.batterySense()) HidnSeek.shutdownSys(); // else digitalWrite(shdPin, HIGH);
  if (forceSport) {
    if ((batteryPercent < 25) || (limitSport++ >= SPORT_LIMIT)) {
      //serialString(PSTR("Sport Limit")); Serial.println();
      forceSport = 0;
    }
  } else HidnSeek.gpsStandby();

  if (syncSat < 255) {
    HidnSeek.sendSigFox(msgs); // if not arround previous location send new position
  }

  HidnSeek.accelStatus(); // record the current angle
  if (msgs == MSG_POSITION && spd > 5 && noSat == 0) detectMotion = MOTION_MIN_NUMBER << 1; else detectMotion = 0;

  // Loop duration 8s. 75x 10mn, 150x 20mn,
  static uint8_t countNoMotion;
  if (msgs != MSG_NO_MOTION) countNoMotion = 0;

  uint8_t modeSport = forceSport;

  unsigned int waitLoop;
  if (msgs == MSG_NO_MOTION) {
    waitLoop = 420 << countNoMotion; // 1h loop
    hour += (1 << countNoMotion);
    if (hour > 23) {
      hour = 0;
      day++;
    }
    if (day > 31) {
      day = 0;
      month++;
    }
    if (month > 12) {
      month = 0;
      year++;
    }
    if (countNoMotion < 3) countNoMotion++;
  } else {
    // waitLoop = (PERIOD_COUNT >> forceSport) - loopGPS;
    waitLoop = PERIOD_COUNT - loopGPS;  // 10mn loop: 6mn sleep + 4mn for GPS
  }

  unsigned int i = 0;

  period_t sleepDuration;
  if (msgs != MSG_NO_MOTION && detectMotion == 0 && forceSport == 0) sleepDuration = SLEEP_4S; else sleepDuration = SLEEP_8S;

  Serial.flush();

  while (i < waitLoop) {
    LowPower.powerDown(sleepDuration, ADC_OFF, BOD_OFF);
    PORTD |= (1 << redLEDpin) | (forceSport << bluLEDpin);
    if (GPSactive) {
      HidnSeek.batterySense();
      if (batteryPercent < 98 && !forceSport) HidnSeek.gpsStandby();
    }
    if (HidnSeek.accelStatus()) { // device moved
      if (GPSactive) HidnSeek.NoflashRed(); else delay(50);
      detectMotion++;
      if (sleepDuration == SLEEP_4S) {
        sleepDuration = SLEEP_8S;
        i = i >> 1;
      }
      if (msgs == MSG_NO_MOTION || modeSport != forceSport) waitLoop = 0; // exit immediatly or stay in 5mn loop
    }
    i++;
    PORTD &= ~(1 << redLEDpin) & ~(1 << bluLEDpin);
  }
  detectMotion = (detectMotion > MOTION_MIN_NUMBER || forceSport) ? 1 : 0;
  if (msgs == MSG_NO_MOTION && waitLoop == 0) detectMotion = -1; // This mean a motion after a while
  if (detectMotion > 0 && !GPSactive) GPSactive = HidnSeek.initGPS();
  start = millis();
  loopGPS = syncSat = noSat = 0;
  alt = spd = 0;
  p.lat = p.lon = 0;
  return detectMotion;
}

int main(void)
{
  init();
  delay(100);

  HidnSeek.initGPIO();

  Serial.begin(9600);
  Serial.println();
  HidnSeek.serialString(PSTR("Firmware " FILE "\nRelease " __DATE__ " " __TIME__));
  Serial.println();

  HidnSeek.dumpEEprom();

  HidnSeek.initSense();
  HidnSeek.batterySense();
  HidnSeek.serialString(PSTR(" Battery: "));
  Serial.print(batteryPercent);
  HidnSeek.serialString(PSTR("% "));
  Serial.println(batteryValue);
  delay(100);

  if (GPSactive = HidnSeek.initGPS()) {
    HidnSeek.gpsCmd(PSTR(PMTK_VERSION));
    HidnSeek.flashRed(1);
  }

  if (accelPresent = HidnSeek.initMems()) {
    delay(500);
    if (HidnSeek.accelStatus()) HidnSeek.flashRed(2);
  }

  if (baromPresent = HidnSeek.initBaro()) {
    delay(500);
    HidnSeek.bmp180Measure(&Temp, &Press);
    HidnSeek.flashRed(3);
  }

  HidnSeek.bmp180Print();

  if (HidnSeek.initSigFox()) {
    delay(500);
    HidnSeek.flashRed(4);
  } else {
    PORTD |= (1 << bluLEDpin);
    Serial.flush();
    delay(500);
    digitalWrite(shdPin, LOW);
    delay(1000);
  }

  HidnSeek.serialString(PSTR("free Ram: "));
  Serial.println(HidnSeek.freeRam()); // Clears data from RAM

  HidnSeek.changeCurrent500mA(false);   // Changes charge current to 500mA when set to true - Not recommended if connected to a computer, use with wall adapter only.

  start = millis();

  while (1) {

    if ((uint16_t) (millis() - start) >= 4000) {
      blueLEDon;
      delay(100);
      HidnSeek.accelStatus();
      blueLEDoff;
      loopGPS++;
      start = millis();
    }

    // if a sentence is received, we can check the checksum, parse it...
    if (detectMotion == 1) {
      if (HidnSeek.gpsProcess()) LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
    }

    // Let 2mn to acquire GPS position otherwise go to sleep until accelerometer wake up.
    if ((( syncSat >= 64 && (spd < 11 || spd > 80)) || syncSat >= 128) && noSat == 0) {
      detectMotion = powerDownLoop(MSG_POSITION);
    }

    if ( loopGPS > 30 || noSat > 120) {
      detectMotion = powerDownLoop(MSG_NO_GPS);
    }

    if (detectMotion == 0) detectMotion = powerDownLoop(MSG_NO_MOTION);
    if (detectMotion == -1) detectMotion = powerDownLoop(MSG_MOTION_ALERT); // Alert for Motion detected after the blank time
  }
}
