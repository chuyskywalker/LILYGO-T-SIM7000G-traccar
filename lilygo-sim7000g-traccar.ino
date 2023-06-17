/* 
Based on many sources, including:
 - https://RandomNerdTutorials.com/lilygo-t-sim7000g-esp32-lte-gprs-gps/
 - https://randomnerdtutorials.com/lilygo-t-sim7000g-esp32-gps-data/
 - https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G
 - https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/Arduino_TinyGSM/AllFunctions/AllFunctions.ino
 - https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/Arduino_NetworkTest/Arduino_NetworkTest.ino
 - https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/Historical/SIM7000G_20200415/README.MD
 - https://cdn.geekfactory.mx/sim7000g/SIM7000%20Series_AT%20Command%20Manual_V1.06.pdf
 - https://www.amazon.com/dp/B07JCTZ3BF
 - https://www.amazon.com/gp/product/B099RQ7BSR
 - https://github.com/onlinegill/LILYGO-TTGO-T-SIM7000G-ESP32-Traccar-GPS-tracker/blob/main/traccar.ino
 - https://github.com/htotoo/TCarTrackerSim/blob/main/esp32_car_github.ino
 - https://github.com/traccar/traccar/blob/master/src/main/java/org/traccar/protocol/OsmAndProtocolDecoder.java
 - https://github.com/traccar/traccar/blob/master/src/main/java/org/traccar/model/Position.java#L27
 - https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/issues/81
 */

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// Your GPRS credentials, if any
const char apn[]  = "XXXXXX";     // SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <UrlEncode.h>
#include <TimeLib.h>

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);

const char server[] = "XXXXXXX"; // traccar installation ip address
const int port = 5055;

HttpClient http(client, server, port);

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define LED_PIN     12
#define BAT_ADC     35

int counter, lastIndex, numberOfPieces = 24;
String pieces[24], input;

// LILYGO T-SIM7000G
// https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G/blob/master/examples/readBattery/readBattery.ino
float ReadBattery()
{
  int vref = 1100;
  uint16_t volt = analogRead(BAT_ADC);
  return ((float)volt / 4095.0) * 2.0 * 3.3 * (vref/1000);
}

void setup(){
  // Set console baud rate
  Serial.begin(115200);
  delay(100);

  Serial.println("--- Starting up");

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("--- Toggle modem pin");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  // Starting the machine requires at least 1 second of low level, and with a level conversion, the levels are opposite
  delay(1000);
  digitalWrite(PWR_PIN, LOW);

  Serial.println("--- Give modem boot time");
  delay(1000);

  Serial.println("--- Open modem serial connection");
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("--- Initializing modem...");
  if (!modem.restart()) {
    Serial.println("--- Failed to restart modem, continue anyway");
  }
}

void loop(){

  Serial.println("--- Initializing modem...");
  if (!modem.init()) {
    Serial.println("--- Failed to initialize modem, looping");
    delay(1000);
    return;
  }

  // CFUN=0 == reset to basic functionality
  Serial.println("--- CFUN=0; Reset modem to basic functionality");
  modem.sendAT("+CFUN=0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" +CFUN=0  false ");
  }
  delay(200);

  Serial.println("--- Set networkmode");
  /*
    2 Automatic
    13 GSM only
    38 LTE only
    51 GSM and LTE only
  * * * */
  String res;
  // CHANGE NETWORK MODE, IF NEEDED
  res = modem.setNetworkMode(51);  // customize as needed
  if (res != "1") {
    DBG("setNetworkMode  false ");
    return ;
  }
  delay(200);

  Serial.println("--- Set preferredmode");
  /*
    1 CAT-M
    2 NB-Iot
    3 CAT-M and NB-IoT
  * * */
  // CHANGE PREFERRED MODE, IF NEEDED
  res = modem.setPreferredMode(2);  // customize as needed
  if (res != "1") {
    DBG("setPreferredMode  false ");
    return ;
  }
  delay(200);

  Serial.println("--- CFUN=1; all functionality back on");
  modem.sendAT("+CFUN=1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" +CFUN=1  false ");
  }
  delay(200);

  // todo: what in the blazes is this section doing?
  Serial.println("--- CGDCONT PDP...thing");
  SerialAT.println("AT+CGDCONT?");
  delay(500);
  if (SerialAT.available()) {
    input = SerialAT.readString();
    for (int i = 0; i < input.length(); i++) {
      if (input.substring(i, i + 1) == "\n") {
        pieces[counter] = input.substring(lastIndex, i);
        lastIndex = i + 1;
        counter++;
       }
        if (i == input.length() - 1) {
          pieces[counter] = input.substring(lastIndex, i);
        }
      }
      // Reset for reuse
      input = "";
      counter = 0;
      lastIndex = 0;

      for ( int y = 0; y < numberOfPieces; y++) {
        for ( int x = 0; x < pieces[y].length(); x++) {
          char c = pieces[y][x];  //gets one byte from buffer
          if (c == ',') {
            if (input.indexOf(": ") >= 0) {
              String data = input.substring((input.indexOf(": ") + 1));
              if ( data.toInt() > 0 && data.toInt() < 25) {
                modem.sendAT("+CGDCONT=" + String(data.toInt()) + ",\"IP\",\"" + String(apn) + "\",\"0.0.0.0\",0,0,0,0");
              }
              input = "";
              break;
            }
          // Reset for reuse
          input = "";
         } else {
          input += c;
         }
      }
    }
  } else {
    Serial.println("Failed to get PDP!");
  }

  Serial.println("--- Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println("--- Wait failed, restart loop");
    delay(1000);
    return;
  }

  if (modem.isNetworkConnected()) {
    Serial.println("--- Network connected");
  }
  
  Serial.println("--- Connecting to GPRS with APN: " + String(apn));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("--- GPRS connect failed, looping");
    delay(1000);
    return;
  }

  Serial.print("--- GPRS status: ");
  if (modem.isGprsConnected()) {
    Serial.println("connected");
  } else {
    Serial.println("not connected");
    delay(1000);
    return;
  }

  Serial.println("--- Power on the GPS");
  // Set SIM7000G GPIO4 HIGH ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
  
  int nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond, usat = 0, vsat = 0;
  float gpsLatitude = 0, gpsLongitude = 0, gpsSpeed = 0,  gpsAltitude = 0, gpsAccuracy = 0, battery = 0;

  Serial.println("--- Fetching GPS location after lock");
  while (1) {
    if (modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      Serial.printf("--- got gps data: lat:%f lon:%f\n", gpsLatitude, gpsLongitude);
      break;
    } else {
      Serial.print("--- awaiting lock, ms:");
      Serial.println(millis());
    }
    delay(2000);
  }
  modem.disableGPS();

  Serial.println("--- Turning off GPS power");
  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  Serial.println("--- GPS turned off");


  Serial.println("--- Send Coords");

  tmElements_t my_time;  // time elements structure
  time_t unix_timestamp; // a timestamp

  // convert the GPS date and time (which is in UTC) into unix time, offset 1970
  my_time.Second = nowSecond;
  my_time.Hour   = nowHour;
  my_time.Minute = nowMinute;
  my_time.Day    = nowDay;
  my_time.Month  = nowMonth;                // sometimes (months start from 0, so deduct 1) but in this case, nope
  my_time.Year   = CalendarYrToTm(nowYear); // years since 1970, so deduct 1970

  unix_timestamp = makeTime(my_time);

  // get the battery voltage, to report back
  battery = ReadBattery();

  // experimental version (i'm pretty sure this one doesn't really work)
  uint8_t  chargeState = -99;
  int8_t   percent     = -99;
  uint16_t milliVolts  = -9999;
  modem.getBattStats(chargeState, percent, milliVolts);

  String GpsData = "id=" + urlEncode(modem.getIMEI())
                 + "&lat=" + urlEncode(String(gpsLatitude, 6))
                 + "&lon=" + urlEncode(String(gpsLongitude, 6))
                 + "&speed=" + urlEncode(String(gpsSpeed))
                 + "&altitude=" + urlEncode(String(gpsAltitude))
                 + "&accuracy=" + urlEncode(String(gpsAccuracy))
                 + "&satVisible=" + urlEncode(String(vsat)) // Visible Satellites
                 + "&sat=" + urlEncode(String(usat)) // Used Satellites
                 + "&timestamp=" + urlEncode((String)unix_timestamp)
                 + "&battery=" + urlEncode(String(battery, 4))
                 + "&xbatstate=" + urlEncode(String(chargeState))
                 + "&xbatpercent=" + urlEncode(String(percent))
                 + "&xbatvolts=" + urlEncode(String(milliVolts, 4))
                 ;
  
  Serial.write("--- GPS Data: ");
  Serial.println(GpsData);

  Serial.println("--- POST data");
  int err = http.post("/?" + GpsData);
  if (err != 0) {
    Serial.println("--- failed to post data");
    Serial.println(err);
  }

  int status = http.responseStatusCode();
  if (!status) {
    Serial.println("--- Bogus response code:");
    Serial.println(status);
  }

  String body = http.responseBody();
  Serial.println("--- Response:");
  Serial.println(body);

  // close the connection
  http.stop();
  Serial.println("--- GPS Data submitted");


  Serial.println("--- GPRS disconnecting");
  modem.gprsDisconnect();
  if (!modem.isGprsConnected()) {
    Serial.println("--- GPRS disconnected");
  } else {
    Serial.println("--- GPRS disconnect: Failed.");
  }

  // --------TESTING POWER DONW--------
  Serial.println("--- Entering power down state");

  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.sendAT("+CPOWD=1");
  if (modem.waitResponse(10000L) != 1) {
    DBG("+CPOWD=1");
  }
  // The following command does the same as the previous lines
  modem.poweroff();

  // GO TO SLEEP
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(200);
  esp_deep_sleep_start();

  // Do nothing forevermore
  while (true) {
      modem.maintain();
  }
}
