/* Projekt: Sound-Clock
 * Part: Main
 * Hardware: Arduino Nano V3/Every, RTC DS3231, DFPlayer Mini, DCF77
 *
 * Description: With the Arduino, an DCF77, an RTC DS3221 and a DFPlayer Mini
 * I've created a small 'Box' that will play a sound every full hour from the 
 * SD-card of the DFPlayer. For a correct time and date this box has also 
 * a DCF77-Modul to receive a Time-Signal (in my case on startup and then 
 * every month). If the time and date was received corretly, the RTC and the
 * internal clock of the Arduino will synchronyzed.
 * If you have any questions, found an issue or have optimization tips
 * visit my GitHub-Repo @ github.com/dancervlt69
 *
 * Note:
 *  Yes, I know, thats not the best code, but hey, 
 *  I'm a beginner, and it works ;)
 *
 * Date: 2024/06/26
 * Location: Berlin/Germany
 * Autor: DancerVLT69
 * License: GPL3
 * 
 * Credits:
 *          Claude3.5 Sonnet: some code and optimization
 *          ChatGPT4.o/3.5: some optimization
 */
#include <TimeLib.h> 
#include <RTClib.h>
#include <DCF77.h>

#include <LowPower.h>
#include <avr/wdt.h>

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

#define DCF_PIN 2
#define DCF_POWER_PIN 4
#define DCF_INTERRUPT 0
#define DCF_StatusLed 5

#define FullDay_PIN 3     // Pin to toggle Full-Day-Play-Mode on/off
#define MP3_BUSY_PIN 10
#define RTC_ALARM_PIN 11
#define PWR_MEASURE_PIN 12 // Digital Pin to handle the Voltage-Divider

#define BATTERY_PIN A0
#define VIN_PIN A1        // Pin to measure the Input-Voltage / zur Überprüfung der Eingangsspannung

// #define SERIAL_TIMEOUT 1000
// #define DEBUG_PRINT(x)  if(DEBUG) Serial.print(x)
// #define DEBUG_PRINTLN(x)  if(DEBUG) Serial.println(x)
// #define DEBUG_PRINT_F(x)  if(DEBUG) Serial.print(F(x))
// #define DEBUG_PRINTLN_F(x)  if(DEBUG) Serial.println(F(x))

RTC_DS3231 rtc;
DCF77 DCF = DCF77(DCF_PIN, DCF_INTERRUPT);

SoftwareSerial mp3Serial (8, 9);  // RX / TX
DFRobotDFPlayerMini myDFPlayer;

String softwareVersion = " 0.0.37.0 (alpha)";

const float USB_THRESHOLD = 4.5;           // Schwellenwert zur Unterscheidung zwischen USB und Batterie
const float BATTERY_THRESHOLD = 3.3;       // Spannung in Volt, unter der BOD aktiv bleiben soll
const float VOLTAGE_DIVIDER_RATIO = 2.0;
const float ADC_REF_VOLTAGE = 1.1;

const uint8_t sleepMinutes = 3; // Temporarily value for the Sleep-Mode-Lenght in minutes
const uint8_t sysVolume = 18;

const uint8_t filesInFolder2 = 1;
const uint8_t filesInFolder3 = 1;
const uint8_t filesInFolder4 = 2;
const uint8_t filesInFolder5 = 2;
const uint8_t maxDCFSynchAtt = 4;

const unsigned long onDuration[] = {100, 250};
const unsigned long offDuration[] = {1500, 1500};
const unsigned long waitTime[] = {60000, 1500};
unsigned int runMinutes, runHours, runDays, runYears;
unsigned long lastMillis = 0;
bool ledState = LOW;
  
const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

int8_t lastExecutedHour = -1;
int8_t lastCheckedMinute = -1;
int8_t lastCheckedQuarter = -1;
uint8_t lastExecutedMonth = 0;
uint8_t lastCheckedWeek = 0;
uint8_t lastCheckedDay = 0;
uint8_t currentWeek = 0;
uint8_t specialDayProbability = 25;
uint8_t currentFileSummer = 1;
uint8_t currentFileXmas = 1;
uint8_t currentFileEven = 1;
uint8_t currentFileOdd = 1;
uint8_t totalFilesEven, totalFilesOdd, totalFilesSummer, totalFilesXmas;
unsigned int watchdogCount = 48; // Watchdog-Timer in hours!!!
unsigned long btnDelayTimer = 0;

bool fullDayPlay = false;
bool hourBellPlayed = false;
bool quarterBellPlayed = false;
bool initialMonthlyTasks = false;
bool initialWeeklyTasks = false;
bool initialHourlyTasks = false;
bool initialQuarterTasks = false;
bool dcfSynchSuccess = false;
bool fdButtonPin = false;
bool fdButtonPressed = false;
bool DEBUG = false;

struct HourInfo {
  String evenOddString;
  uint8_t currentFolder;
  uint8_t* currentFile;
};

void setup() {
  Serial.begin(115200);
  mp3Serial.begin (9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DCF_StatusLed, OUTPUT);
  pinMode(DCF_POWER_PIN, OUTPUT);
    digitalWrite(DCF_POWER_PIN, HIGH);  // sometimes needed to activate the DCF77-Modul
    digitalWrite(DCF_StatusLed, HIGH);  // turn DCF-Status LED on 
    // permanent ON = DCF-Sync failed / short flash = DCF-Sync in Progress / permanent OFF = DCF-Sync successful

  activateDebug();

  if(Serial) {
    Serial.println();
    Serial.print(F("SoundClock Version "));
    Serial.println(softwareVersion);
    Serial.println(F("Check and init Hardware, please wait..."));
    Serial.println();
  }

  randomSeed(analogRead(0));

  // bool debug = isUSBPowered();

  pinMode (FullDay_PIN, INPUT_PULLUP);  // Pin for a special function in the RTC-Module
  pinMode (MP3_BUSY_PIN, INPUT);        // Pin to check, if the DFPlayer is busy
  pinMode(PWR_MEASURE_PIN, OUTPUT);     // Pin to activate the voltage measurement
  digitalWrite(PWR_MEASURE_PIN, LOW);   // initially switched off
  analogReference(INTERNAL);            // activating ADC for the voltage measurement

  chkMP3Player();
    if(Serial) {
      delay(250);
      totalFilesSummer = myDFPlayer.readFileCountsInFolder(2);
      Serial.println("Count of Files in Folder 2: " + String(totalFilesSummer));
      delay(250);
      totalFilesXmas = myDFPlayer.readFileCountsInFolder(3);
      Serial.println("Count of Files in Folder 3: " + String(totalFilesXmas));
      delay(250);
      totalFilesEven = myDFPlayer.readFileCountsInFolder(4);
      Serial.println("Count of Files in Folder 4: " + String(totalFilesEven));
      delay(250);
      totalFilesOdd = myDFPlayer.readFileCountsInFolder(5);
      Serial.println("Count of Files in Folder 5: " + String(totalFilesOdd));
      Serial.println();
    }
    
    delay(150);
    
  chkRTC();
    delay(150);
}

void loop() {
  DateTime now = rtc.now();
  if(Serial) {
    Serial.println(F("Updating Date & Time..."));
    updateDateTime();
    digitalClockDisplay();
  } // delay(250);
  
  if (now.day() != lastCheckedDay) {
    checkMonthlyTasks(now);
    lastCheckedDay = now.day();
  }

  if (currentWeek != lastCheckedWeek) {
    checkWeeklyTasks(now);
    lastCheckedWeek = currentWeek;
  }

  if (now.minute() != lastCheckedMinute) {
    checkHourlyTasks(now);
    lastCheckedMinute = now.minute();
  }

  isBtnPressed();
   
  chkToSleep(sleepMinutes);

  waitTimeLED(13, onDuration[1], offDuration[1], waitTime[1]);  // to reduce the debug outputs

  // jumpToNextHour(); // only needed for testing some features/options
}

void activateDebug() {
  //  // Warte eine bestimmte Zeit auf die serielle Verbindung
  // unsigned long startTime = millis();
  // while (!Serial && millis() - startTime < SERIAL_TIMEOUT) {
  //   ; // Warte
  // }
  
  // DEBUG = Serial;  // Setze DEBUG auf true, wenn eine Verbindung besteht
  
  // DEBUG_PRINTLN_F("Debug-Mode activated...");
}

void checkHourlyTasks(const DateTime& now) {
  if(Serial) {
    Serial.println(F("Check/run hourly Tasks!"));
  }
  uint8_t minuteNow = now.minute();
  uint8_t hourNow = now.hour();
  
  if (hourNow != lastExecutedHour && minuteNow <= 4) {
    if (!initialHourlyTasks) {
      runHourlyTasks(now, lastExecutedHour);
      lastExecutedHour = hourNow;
      initialHourlyTasks = true;
    } else {
      runHourlyTasks(now, lastExecutedHour);
      lastExecutedHour = hourNow;
    }
  } else {
    if(Serial) {
      Serial.println(F("  No (other) hourly Tasks..."));
      Serial.println();
    }
  }
}

void runHourlyTasks(const DateTime& now, int8_t lastExecutedHour) {
  uint8_t hourNow = now.hour();

  if (hourNow != lastExecutedHour) {
    
    if (chkPlayHours(now) && chkFullHourToPlay()) {
      if(Serial) {
        Serial.println();
        Serial.print("* " + String(bellCount(hourNow)));
        Serial.println(F(" times Hour Bell (only between 6 and 22 a clock or 24h if FullDayPlay is activated)."));
        Serial.print(F("   PlayHours-Status: "));
        Serial.println(chkPlayHours(now));
      }
      // if (!hourBellPlayed) {
      
      initQuarterBell(now, sysVolume);
      initHourBell(now, sysVolume);
       
      // }
      delay(250);

      playSong(now, sysVolume);

      delay(250);
    }

    if(Serial) { Serial.println(F("* Deactivating FullDayPlay at 6 AM")); }
    if(hourNow >= 6 && hourNow <= 18) resetFullDayPlay();
    if(Serial) { Serial.println("   Status FullDayPlay: " + String(fullDayPlay)); }

    if(Serial) {
      Serial.println();
      Serial.println (F("* If DCF77-Sync was failed, repead hourly."));
    }
    if (!dcfSynchSuccess) {
      syncTimeWithDCF();
    } else {
      if(Serial) {
        Serial.println(F("   Not needed right now"));
        Serial.println();
      }
    }
    delay(250);
  }
  
  digitalClockDisplay();
}

void checkWeeklyTasks(const DateTime& now) {
  if(Serial) {
    Serial.println(F("Check/run weekly Tasks!"));
  }

  if ((!initialWeeklyTasks || currentWeek != lastCheckedWeek) && !dcfSynchSuccess && !initialMonthlyTasks) {
    currentWeek = getCurrentWeek(now);
    dcfSynchSuccess = false;
    runWeeklyTasks(now, lastCheckedWeek);
    lastCheckedWeek = currentWeek;
    initialMonthlyTasks = true;
  } else {
    if(Serial) {
      Serial.println(F("  No (more) weekly Tasks..."));
      Serial.println();
    }
  }
}

void runWeeklyTasks(const DateTime& now, uint8_t lastCheckedWeek) {
  if (currentWeek != lastCheckedWeek) {
    dcfSynchSuccess = false;
    syncTimeWithDCF();    
  }
}

void checkMonthlyTasks(const DateTime& now) {
  if(Serial) {
    Serial.println(F("Check/run monthly Tasks!"));
  }
  uint8_t monthNow = now.month();
  
  if (!initialMonthlyTasks || (monthNow != lastExecutedMonth && now.day() == 1) && !initialWeeklyTasks) {
    dcfSynchSuccess = false;
    runMonthlyTasks(now, lastExecutedMonth);
    lastExecutedMonth = monthNow;
    initialMonthlyTasks = true;
  }
  else {
    if(Serial) {
      Serial.println(F("  No (more) monthly Tasks..."));
      Serial.println();
    }
  }
}

void runMonthlyTasks(const DateTime& now, int8_t lastExecutedMonth) {

  if (now.month() != lastExecutedMonth || rtc.lostPower() || now.year() < 2024) {
      dcfSynchSuccess = false;
      syncTimeWithDCF();
      currentWeek = getCurrentWeek(now);
  }
  else {
    if(Serial) { Serial.println(F("* DCF77-Sync not needed right now.")); }
    dcfSynchSuccess = true;
    setTime(now.unixtime());
  }
}

void syncTimeWithDCF() {
  unsigned long waitMillis = millis();
  digitalWrite(DCF_POWER_PIN, LOW);
  digitalWrite(DCF_StatusLed, HIGH);
  if(Serial) {
    Serial.println(F("* Synching Time & Date with DCF77..."));
    Serial.println();
  }
  // dcfSynchSuccess = false;
  DCF.Start();
  time_t DCFtime = 0;
  uint8_t attempts = 0;
  while (DCFtime == 0 && attempts < maxDCFSynchAtt) {
    DCFtime = DCF.getTime();
    if (DCFtime != 0) {
      handleSyncSuccess(DCFtime);
    }
    
    attempts++;
    if (DCFtime == 0) {
      if(Serial) {
        Serial.print(F("  DCF77-Sync Attempt "));
        Serial.print(attempts);
        Serial.print(F(" of "));
        Serial.println(maxDCFSynchAtt);
      }

      waitTimeLED(DCF_StatusLed, onDuration[0], offDuration[0], waitTime[0]);
      // delay(60000); // wait a minute befor start the next attempt / Warte eine Minute vor dem nächsten Versuch
    }
  }
  
  DCF.Stop();
  if (DCFtime == 0) {
    handleSyncFailed();
  }
}

void handleSyncSuccess(time_t DCFtime) {
  
    if(Serial) {
      Serial.println(F("  DFC-Time-Sync successful!"));
      Serial.println();
    }
  setPlayVolume(sysVolume);
  myDFPlayer.playFolder(1,10);
  delay(250);
  setPlayVolume(sysVolume);
  
  rtc.adjust(DateTime(DCFtime));
  setTime(DCFtime);
  dcfSynchSuccess = true;
  
  digitalWrite(DCF_StatusLed, LOW);
  digitalWrite(DCF_POWER_PIN, HIGH);
  digitalClockDisplay();
}

void handleSyncFailed() {
  if(Serial) {
    Serial.print(F("  DCF-Time-Sync failed! Next try in "));
    Serial.print( 60 - rtc.now().minute() + 2);
    Serial.println(F(" minute/s."));
    Serial.println();
  }
  setPlayVolume(sysVolume);
  myDFPlayer.playFolder(1,11);
  delay(250);
  setPlayVolume(sysVolume);
  digitalWrite(DCF_POWER_PIN, HIGH);
  digitalWrite(DCF_StatusLed, HIGH);
  digitalClockDisplay();
}

void chkMP3Player() {
 if (!myDFPlayer.begin(mp3Serial)) {  // using SoftwareSerial to communicate with the MP3-Player-Modul.

    delay(250);
    if(Serial) {
      Serial.println(F("Unable to find/connect to the DFPlayer Mini!"));
      Serial.println(F("1. Please recheck the connection!"));
      Serial.println(F("2. Please insert the SD card!"));
    }
    while(true) delay(15);
  }

  myDFPlayer.setTimeOut(500);
  setPlayVolume(0);
  myDFPlayer.stop();
  myDFPlayer.disableDAC();
  setPlayVolume(sysVolume);
  myDFPlayer.playFolder(1,1);
  delay(4800);
  setPlayVolume(0);
  myDFPlayer.stop();
  
  if(Serial) {
    Serial.print(F("DFPlayer Mini online. Status: "));
    Serial.println(myDFPlayer.readState()); //read mp3 state
    delay(200);
    Serial.print(F("DFPlayer Volume: "));
    Serial.println(myDFPlayer.readVolume()); //read current volume
    delay(200);
    Serial.print(F("DFPlayer EQ-Settings: "));
    Serial.println(myDFPlayer.readEQ()); //read EQ setting
    delay(200);
    Serial.print(F("DFPlayer Total Files: "));
    Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
    delay(200);
    Serial.print(F("DFPlayer Current Play File: "));
    Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
    delay(200);
    Serial.print(F("DFPlayer Count of Folder: "));
    Serial.println(myDFPlayer.readFolderCounts());
    delay(200);
    Serial.print(F("DFPlayer File Count in Folder 1: "));
    Serial.println(myDFPlayer.readFileCountsInFolder(1));
    delay(200);
    Serial.print(F("DFPlayer File Count in Folder 2: "));
    Serial.println(myDFPlayer.readFileCountsInFolder(2));
    delay(200);
    Serial.print(F("DFPlayer File Count in Folder 3: "));
    Serial.println(myDFPlayer.readFileCountsInFolder(3));
    delay(200);
    Serial.print(F("DFPlayer File Count in Folder 4: "));
    Serial.println(myDFPlayer.readFileCountsInFolder(4));
    delay(200);
    Serial.print(F("DFPlayer File Count in Folder 5: "));
    Serial.println(myDFPlayer.readFileCountsInFolder(5));
    Serial.println();
    
    delay(1000); // wait until all serial monitor outputs have been displayed
  }
}

void chkRTC () {
  if (!rtc.begin()) {
    if(Serial) {
      Serial.println(F("Couldn't find/connect to RTC"));
      Serial.flush();
    }
    while(true) delay(15);
  }
  updateDateTime();
}

void updateDateTime() {
  DateTime now = rtc.now();
  
  if (Serial && !dcfSynchSuccess && now.year() < 2020){
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  setTime(now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2024, 7, 11, 23, 58, 0)); // temporarily to test some feature
}

bool chkPlayHours(const DateTime& now) {
  uint8_t hourNow = now.hour();

  if (!fullDayPlay) {
    return (hourNow >= 6 && hourNow <= 22);
  }
  return true;
}

void chkToSleep(uint8_t sleepMinutes) {
  DateTime now = rtc.now();
  uint8_t hourNow = now.hour();
    
  if (hourNow < 20 || hourNow > 21) {
    if (chkMinutesToSleep()) {
      digitalWrite(LED_BUILTIN, LOW);
      goToSleep(now, sleepMinutes);
    } else {
      if(digitalRead(MP3_BUSY_PIN)) {
      }
    }
  }
}

bool chkMinutesToSleep() {
  DateTime now = rtc.now();
  uint8_t minuteNow = now.minute();
  return (minuteNow > 3 && minuteNow < 13) ||
         (minuteNow > 17 && minuteNow < 27) ||
         (minuteNow > 33 && minuteNow < 43) ||
         (minuteNow > 47 && minuteNow < 57);
}

bool chkFullHourToPlay() {
  DateTime now = rtc.now();
  uint8_t minuteNow = now.minute();
  return (minuteNow <= 2);
}

void goToSleep(const DateTime& now, uint8_t sleepMinutes) {
  // int minutesToNextHour = 60 - now.minute();
  // int secondsToNextHour = 3600 - ((now.minute() + 2) * 60 + now.second());
  uint8_t secondsToNextWakeUp = sleepMinutes * 60 - now.second();
  
  float batteryVoltage = getBatteryVoltage();
  if(Serial) {
    Serial.print(F("Battery Voltage: "));
    Serial.print(batteryVoltage);
    Serial.println(F(" Volt"));

    Serial.print(F("Go to sleep for "));
    Serial.print(secondsToNextWakeUp / 60 + 1);
    Serial.print(F(" Minutes... until "));
  }
  
  uint8_t addDigitMinute = secondsToNextWakeUp / 60 + now.minute() + 1;
  uint8_t addDigitHour = now.hour();
  if (addDigitMinute >= 60) {
    addDigitMinute = addDigitMinute - 60;
    addDigitHour = addDigitHour + 1;
    if (addDigitHour >= 24) {
      addDigitHour = addDigitHour - 24;
    }
  }
  String addedDigits = addDigits(addDigitHour) + ":" + addDigits(addDigitMinute) + ":" + addDigits(now.second());
  if(Serial) { Serial.println(addedDigits); }

  delay(250);
  
  bool useBOD = batteryVoltage < BATTERY_THRESHOLD;

  while (secondsToNextWakeUp > 0) {
    if (secondsToNextWakeUp >= 8) {
      if (useBOD) {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
      } else {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      }
      secondsToNextWakeUp -= 8;
    } else {
      if (useBOD) {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
      } else {
        LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      }
      secondsToNextWakeUp -= 1;
    }
  }
  
  if(Serial) {
    Serial.println(F("\n--------------------------------------"));
    Serial.println(F("-- >> Wake up to check ToDo list << --"));
    Serial.println(F("--------------------------------------"));
  }

  delay(30);
}

float getBatteryVoltage() {
  // Aktiviere den Spannungsteiler
  digitalWrite(PWR_MEASURE_PIN, HIGH);
  delay(35);  // Short delay for a stibilization of the power on the measure pin/s
  
  // Reading (battery) power on pin A0
  int sensorValue = analogRead(BATTERY_PIN);
  
  // Deaktiviere den Spannungsteiler
  digitalWrite(PWR_MEASURE_PIN, LOW);
  
  /* Umrechnung in Volt
  // Wir multiplizieren mit 2, weil der Spannungsteiler die Spannung halbiert
  // und mit 1.1, weil wir die interne 1.1V Referenz verwenden */
  return sensorValue * (ADC_REF_VOLTAGE / 1023.0) * VOLTAGE_DIVIDER_RATIO * VOLTAGE_DIVIDER_RATIO;
  // falls mit LM4040 mit konstanter Referenzspannung
  // return sensorValue * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO; statt V_D_R auf 2.0 setzen, falls V_D_R <> 2
}

bool isUSBPowered() {
  // Aktiviere den Spannungsteiler
  digitalWrite(PWR_MEASURE_PIN, HIGH);
  delay(30);  // short delay until voltage on the pins is stabilized

  pinMode(VIN_PIN, INPUT);
  int sensorValue = analogRead(VIN_PIN);

  // Deaktiviere den Spannungsteiler wieder
  digitalWrite(PWR_MEASURE_PIN, LOW);

  // Berechne die Eingangsspannung
  float vinVoltage = sensorValue * (ADC_REF_VOLTAGE / 1023.0) * VOLTAGE_DIVIDER_RATIO;

  if (vinVoltage > USB_THRESHOLD) {
    if(Serial) { Serial.println(F("USB-Betrieb")); }
    return true;  // USB-Betrieb
  } else {
    if(Serial) { Serial.println(F("Batteriebetrieb")); }
    return false;  // Batteriebetrieb
  }
}

void resetFullDayPlay() {
  fullDayPlay = false;
  if(Serial) {
    Serial.print(F("   FullDayPlay-Reset successful. Status: "));
    Serial.println(fullDayPlay);
  }
}

bool isSpecialDay(const DateTime& now) {
  if (now.month() == 12 && (now.day() >= 24 && now.day() <= 26)) {
    return true;
  }

  if (now.month() == 7 && now.day() == 12) {
    return true;
  }

  return false;
}

HourInfo checkEvenOddHour(const DateTime& now, uint8_t& currentFileEven, uint8_t& currentFileOdd) {
  HourInfo result;
  uint8_t currentHour = now.hour() % 12; // Umwandlung in 12-Stunden-Format
  if (currentHour == 0) {
    currentHour = 12; // to set midnight to 12 / Um Mitternacht auf 12 zu setzen
  }

  bool isEven = (currentHour % 2 == 0);

  if (isEven) {
    result.evenOddString = "even (gerade)";
    result.currentFolder = 4;
    result.currentFile = &currentFileEven;
  } else {
    result.evenOddString = "odd (ungerade)";
    result.currentFolder = 5;
    result.currentFile = &currentFileOdd;
  }

  if (isSpecialDay(now) && random(100) < specialDayProbability) {
    if (now.month() == 7) {
      result.evenOddString += " (Summer)";
      result.currentFolder = 2;
      result.currentFile = &currentFileSummer;
    } else {
      result.evenOddString += " (Xmas)";
      result.currentFolder = 3;
      result.currentFile = &currentFileXmas;
    }
  }

  return result;
}

void playSong(const DateTime& now, uint8_t sysVolume) {
  HourInfo hourInfo = checkEvenOddHour(now, currentFileEven, currentFileOdd);
  setPlayVolume(sysVolume);

  int fileCount; // = myDFPlayer.readFileCountsInFolder(hourInfo.currentFolder);
  // if (fileCount <= 0) fileCount = 1;

  switch (hourInfo.currentFolder) {
    case 2:  // Summer
      fileCount = filesInFolder2;
      break;
    case 3:  // Winter
      fileCount = filesInFolder3;
      break;
    case 4:  // Even
      fileCount = filesInFolder4;
      break;
    case 5:  // Odd
      fileCount = filesInFolder5;
      break;
    default:
      fileCount = 1;  // Default to 1 if folder is not recognized
  }

  if (*hourInfo.currentFile > fileCount) {
    *hourInfo.currentFile = 1;
  }

  if(Serial) {
    Serial.println();
    Serial.print(F("Playing Track "));
    Serial.println(*hourInfo.currentFile);
    Serial.print(F("for the "));
    Serial.print(hourInfo.evenOddString);
    Serial.println(F(" Hour or special Day"));
    Serial.print(F("from Folder "));
    Serial.println(hourInfo.currentFolder);
    Serial.println();
  }

  if (fileCount > 0) {
      myDFPlayer.playFolder(hourInfo.currentFolder, *hourInfo.currentFile);
      (*hourInfo.currentFile)++;
  } else {
    if(Serial) {
      Serial.println();
      Serial.print(F("No files found in Folder "));
      Serial.println(hourInfo.currentFolder);
      Serial.println();
    }
  }
  delay(570);
  
  while (!digitalRead(MP3_BUSY_PIN)) {
    delay(50);
  }
}

void initQuarterBell(DateTime now, uint8_t playVolume) {
  uint8_t currentMinute = now.minute();

  switch (currentMinute) {
    case 0:
      playBell(4, 2, playVolume);
      break;
    case 15:
      playBell(1, 2, playVolume);
      break;
    case 30:
      playBell(2, 2, playVolume);
      break;
    case 45:
      playBell(3, 2, playVolume);
      break;
    default:
      return;
  }
}

void initHourBell(const DateTime& now, uint8_t playVolume) {
   uint8_t countOfBell = now.hour();
   playBell(countOfBell, 3, playVolume);
}

void playBell(uint8_t countOfBell, uint8_t typeOfBell, uint8_t playVolume) {
    if (Serial) { Serial.println("Count of Bell = " + String(bellCount(countOfBell))); }
  for (int i = 0; i < bellCount(countOfBell); ++i) {
    if (Serial) { Serial.println("Bell Count: " + String(i + 1)); }
    setPlayVolume(playVolume);
    myDFPlayer.playFolder(1, typeOfBell);

    delay(2400);
  }
}

int bellCount(uint8_t countOfBell) {
  if (countOfBell == 0) { countOfBell = 12; }
  
  return countOfBell > 12 ? countOfBell - 12 : countOfBell;
}

void setPlayVolume(uint8_t volLvl) {
  myDFPlayer.volume(volLvl);
}

void digitalClockDisplay() {
  if(Serial) {
    Serial.println(F("* Current Date and Time: "));
    Serial.print(F("  RTC-Modul: "));
    showRTC();
    Serial.println();
    Serial.print(F("  System-Clock: "));
    printTime(now());
    Serial.println();
  }
}

void printTime(time_t t) {
  char timeDateString[23];

  snprintf( timeDateString,
            sizeof(timeDateString),
            "%02d.%02d.%04d | %02d:%02d:%02d",
            day(t), month(t), year(t),
            hour(t), minute(t), second(t));
            
  Serial.println(timeDateString);
}

void showRTC() {
  DateTime now = rtc.now();
  char dateTimeString[23];
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(F(", "));

    snprintf(dateTimeString,
             sizeof(dateTimeString),
             "%02d.%02d.%04d | %02d:%02d:%02d",
             now.day(), now.month(), now.year(),
             now.hour(), now.minute(), now.second());

    Serial.print(dateTimeString);

    Serial.print(F(" | Temperature: "));
    Serial.print(rtc.getTemperature()-2, 1);
    Serial.print(F(" °C"));
}

String addDigits(uint8_t timeDigits) {
  if (timeDigits < 10) {
     return String("0" + String(timeDigits));
  }
  return String(timeDigits);
}

void isBtnPressed() {
  fdButtonPin = digitalRead(FullDay_PIN);
  
  if (fdButtonPin) btnDelayTimer = millis();
  if (btnDelayTimer > millis()) btnDelayTimer = 0;
   
  if (!fdButtonPin && !fdButtonPressed && (millis() - btnDelayTimer > 25 )){
    fdButtonPressed = true;
  }

  if (fdButtonPin && fdButtonPressed) {

    switchFullDayPlay();
    
    fdButtonPressed = false;    
  }
}

void switchFullDayPlay() {
  if(Serial) {
    Serial.print(F("Status FullDayPlay: "));
    Serial.println(!fullDayPlay);
  }
  
  fullDayPlay = !fullDayPlay;
    setPlayVolume(23);
    if (fullDayPlay) {
      myDFPlayer.playFolder(1,10); 
      if(Serial) { Serial.println(F("FullDayPlay = ON")); }
    } else {
      myDFPlayer.playFolder(1,11);
      if(Serial) { Serial.println(F("FullDayPlay = OFF")); }
    }
    delay(250);
    setPlayVolume(sysVolume);
}

void jumpToNextHour() {
  DateTime now = rtc.now();
  if (now.minute() >= 3 && now.minute() < 53) {
    if(Serial) {
      Serial.println();
      Serial.print(F("Jumping to (next Hour) "));
    }
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), 53, 0));
    if(Serial) {
      showRTC();
      Serial.println(F(" to test some features."));
    }
  }
}

void waitTimeLED(uint8_t ledPin,  unsigned long ledOn, unsigned long ledOff, unsigned long waitingTime) {
  unsigned long startTime = millis();
  if (startTime > millis()) startTime = 0;
  while (millis() - startTime < waitingTime) { // waitTime in ms to bridge a waiting period
    blinkLED(ledPin, ledOn, ledOff);
    delay(50);                              // short pause, to save CPU-Time
  }

}

void blinkLED(uint8_t ledPin, unsigned long ledOn, unsigned long ledOff){
  if ((ledState == HIGH && millis() - lastMillis >= ledOn) ||
      (ledState == LOW && millis() - lastMillis >= ledOff)) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    lastMillis = millis();
    if (lastMillis > millis()) lastMillis = 0;
  }
}

void turnLEDsOff(uint8_t ledPIN) {
  digitalWrite(ledPIN, LOW);
}

uint8_t getCurrentWeek(const DateTime& now) {
  uint16_t yearNow = now.year();
  uint8_t monthNow = now.month();
  uint8_t dayNow = now.day();
    
  uint16_t dayOfYear = getDayOfYear(yearNow, monthNow, dayNow);

  // Berechne Wochentag des 1. Januar
  uint8_t y = yearNow % 100;
  uint8_t c = yearNow / 100;
  uint8_t w = (1 + 5 * ((y + y / 4) % 7) + 4 * ((c + c / 4) % 7)) % 7;
  if (w == 0) w = 7;

  // Berechne Kalenderwoche
  int8_t currentWeek = (dayOfYear + 6 - (dayNow % 7 + w - 1) % 7) / 7;
  if (w <= 4) {
    currentWeek++;
  } else if (currentWeek == 0) {
    // Woche gehört zum Vorjahr
    y = (yearNow - 1) % 100;
    c = (yearNow - 1) / 100;
    w = (1 + 5 * ((y + y / 4) % 7) + 4 * ((c + c / 4) % 7)) % 7;
    if (w == 0) w = 7;
    currentWeek = (dayOfYear + 6 - (dayNow % 7 + w - 1) % 7) / 7;
    if (w <= 4) currentWeek++;
  }

  return currentWeek;
}

uint16_t getDayOfYear(uint16_t yearNow, uint8_t monthNow, uint8_t dayNow) {
  // Schaltjahresberechnung
  if (yearNow % 4 == 0) {
    if (yearNow % 100 != 0) {
      daysInMonth[1] = 29;
    } else {
      if (yearNow % 400 == 0) {
        daysInMonth[1] = 29;
      }
    }
  } else {
    daysInMonth[1] = 28;  // Sicherstellen, dass Februar in Nicht-Schaltjahren 28 Tage hat
  }
  
  uint16_t dayOfYear = dayNow;
  for (uint8_t i = 1; i < monthNow; ++i) {
    dayOfYear += daysInMonth[i - 1];
  }
  
  return dayOfYear;
}

void addMinsHrsDaysYrs() {
  runMinutes++;
  if (runMinutes >= 60) {
    runMinutes = 0;
    runHours++;
    if (runHours >= 24) {
      runHours = 0;
      runDays++;
      if (runDays >= 365) {
        runDays = 0;
        runYears++;
      }
    }
  }
}

void printRunTime(unsigned int runSeconds) {
    Serial.print(F("Laufzeit: "));
    Serial.print(runYears);
    Serial.print(F(" Jahre, "));
    Serial.print(runDays);
    Serial.print(F(" Tag/e und "));
    Serial.print(addDigits(runHours));
    Serial.print(F(":"));
    Serial.print(addDigits(runMinutes));
    Serial.print(F(":"));
    Serial.print(addDigits(runSeconds));
    Serial.println();
}

// void calcWatchdogCountMS (unsigned int watchdogCount) {
//   unsigned long watchdogCountMS = watchdogCount * 24 * 60 * 60 * 1000 * 1000;
//   Watchdog.enable(watchdogCountMS);
// }
