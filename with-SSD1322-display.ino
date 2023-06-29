// added support for wiring up a 3" oled 256x64 screen
// I used https://www.amazon.com/dp/B0B8126NW5 but any SSD1322 should work
// or just adapt this to one of the dozens and dozens of screens the U8x8 
// library supports (see it's examples)
//
// The hardest part was resoldering that TINY 0ohm resistor to the right pins to put it in SPI mode.
// 
// For my layout I connected the pins as below:
//
//   gnd =  1 -> GND
//   3v3 =  2 -> 3v3
// clock =  4 -> 18 (SCLK/SCK)
//  data =  5 -> 19 (MISO)
//    cs = 16 ->  5 (CS)
//    dc = 14 -> 22 (SCL)
// reset = 15 -> 21 (SDA)
// 
// Some folks recommend puting most of the other pins to ground, but I didn't find I needed to do that.
//
// keyword stuffing for search results:
//  - Synpinya OLED Display 3.12Inch 256X64 25664 Dots Graphic LCD Module Display LCM Screen SSD1322 Controller Support SPI (Yellow)
//  - Spacesea OLED Display 3.12Inch 256X64 25664 Dots Graphic LCD Module Display LCM Screen SSD1322 Controller Support SPI (White), blue (500131408A1)

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include "config.h"

#include <TinyGsmClient.h>

#include <ArduinoHttpClient.h>

#include <UrlEncode.h>
#include <TimeLib.h>

#include <Arduino.h>
#include <WiFi.h>
#include <U8x8lib.h>

U8X8_SSD1322_NHD_256X64_4W_SW_SPI u8x8(/* clock=*/ 18, /* data=*/ 19, /* cs=*/ 5, /* dc=*/ 22, /* reset=*/ 21);

#define U8LOG_WIDTH 32  // 256/8
#define U8LOG_HEIGHT 8  // 64/8
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
U8X8LOG u8x8log;

bool resetRequest = false;

#define SerialMon u8x8log
// #define SerialMon Serial

TinyGsm modem(Serial1);
TinyGsmClient client(modem);

#define UART_BAUD 115200
#define PIN_DTR 25 // reset pin, maybe? Not sure
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define LED_PIN 12

uint postFailures = 0;
uint modemFailures = 0;
uint maxPostFailures = 5;
uint maxModemFailures = 5;

uint loopRate = 15000; // 15 seconds

void modemPowerOn() {
  SerialMon.println("turn on modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000); // Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, HIGH);
  delay(10000); // give the modem 10 seconds to have its coffee
  SerialMon.println("> on");
}

void modemPowerOff() {
  SerialMon.println("turn off modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500); // Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
  SerialMon.println("> off");
}

void modemRestart() {
  SerialMon.println("Restart modem");
  modemPowerOff();
  modemPowerOn();
  SerialMon.println("> restarted");
}

void enableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1)
  {
    SerialMon.println(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1)
  {
    SerialMon.println(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

int nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond, usat = 0, vsat = 0;
float gpsLatitude = 0, gpsLongitude = 0, gpsSpeed = 0,  gpsAltitude = 0, gpsAccuracy = 0;

void send_data() {

  unsigned long millistart = millis();

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

  // compile the urldata
  String url = "id=" + urlEncode(modem.getIMEI())
             + "&lat=" + urlEncode(String(gpsLatitude, 6))
             + "&lon=" + urlEncode(String(gpsLongitude, 6))
             + "&speed=" + urlEncode(String(gpsSpeed))
             + "&altitude=" + urlEncode(String(gpsAltitude))
             + "&accuracy=" + urlEncode(String(gpsAccuracy))
             + "&satVisible=" + urlEncode(String(vsat)) // Visible Satellites
             + "&sat=" + urlEncode(String(usat)) // Used Satellites
             + "&timestamp=" + urlEncode(String(unix_timestamp))
             + "&batteryLevel=100&charge=true" // I desoldered the battery, sooo
             + "";

  Serial.print(F("GPS POST: "));
  Serial.println(url);
  SerialMon.println("gps: " + String(gpsLatitude, 6) + ":" + String(gpsLongitude, 6));

  // new client each time
  HttpClient http(client, traccar_server, traccar_port);

  // start for the / url, and set some timeouts
  http.beginRequest();
  http.setTimeout(5000);
  http.setHttpResponseTimeout(5000);  
  int err = http.post("/", "application/x-www-form-urlencoded", url);
  http.endRequest();

  // SEND IT
  // int err = http.post("/");
  if (err != 0) {
    switch (err) {
      case HTTP_ERROR_CONNECTION_FAILED:
        SerialMon.println("> post fail, conn fail (" + String((millis() - millistart)/1000.0,2) + "s)");
        break;
      case HTTP_ERROR_API:
        SerialMon.println("> post fail, api error (" + String((millis() - millistart)/1000.0,2) + "s)");
        break;
      case HTTP_ERROR_TIMED_OUT:
        SerialMon.println("> post fail, timeout (" + String((millis() - millistart)/1000.0,2) + "s)");
        break;
      case HTTP_ERROR_INVALID_RESPONSE:
        SerialMon.println("> post fail, invalid (" + String((millis() - millistart)/1000.0,2) + "s)");
        break;
      default:
        SerialMon.println("> post fail, ["+String(err)+"] (" + String((millis() - millistart)/1000.0,2) + "s)");
        break;
    }
    postFailures++;
    http.stop();
    return;
  }

  int responseCode = http.responseStatusCode();
  switch (responseCode) {
    case HTTP_ERROR_CONNECTION_FAILED:
      SerialMon.println("> bad resp, conn fail (" + String((millis() - millistart)/1000.0,2) + "s)");
      postFailures++;
      break;
    case HTTP_ERROR_API:
      SerialMon.println("> bad resp, api error (" + String((millis() - millistart)/1000.0,2) + "s)");
      postFailures++;
      break;
    case HTTP_ERROR_TIMED_OUT:
      SerialMon.println("> bad resp, timeout (" + String((millis() - millistart)/1000.0,2) + "s)");
      postFailures++;
      break;
    case HTTP_ERROR_INVALID_RESPONSE:
      SerialMon.println("> bad resp, invalid (" + String((millis() - millistart)/1000.0,2) + "s)");
      postFailures++;
      break;
    default:
      postFailures = 0;
      SerialMon.println("> posted! [" + String(responseCode) + "] (" + String((millis() - millistart)/1000.0,2) + "s)");
      break;
  }
  
  // // grab any response content; may be a command present
  // String body = http.responseBody();
  // if (body != "") {
  //   SerialMon.println(F("Response:"));
  //   SerialMon.println(body);
  //   SerialMon.println(F("/EOF Response"));

  //   // if (body == "boost") {
  //   //   SerialMon.println(F("CMD 'boost' -- Boosting reporting rate"));
  //   //   loopRate = loopRateBoost;
  //   // } else if (body == "endboost") {
  //   //   SerialMon.println(F("CMD 'endboost' -- Returning to normal reporting rate"));
  //   //   loopRate = loopRateNormal;
  //   // } else if (body == "reset") {
  //   //   SerialMon.println(F("CMD 'reset' -- setting"));
  //   //   resetRequest = true;
  //   // } else {
  //   //   SerialMon.println(F("No such command; no effect"));
  //   // }

  //   // String loopRateCommand = "loopRate=";
  //   // uint cmdLen = loopRateCommand.length();
  //   // if (body.startsWith(loopRateCommand) && body.length() > cmdLen) {
  //   //   String nextRate = body.substring(cmdLen);
  //   //   int nextRateVal = nextRate.toInt();
  //   //   if (nextRateVal != 0) {
  //   //     SerialMon.println(F("Change loop rate to:"));
  //   //     SerialMon.println(nextRate);
  //   //     loopRate = nextRateVal;
  //   //   } else {
  //   //     SerialMon.println(F("Got loopRate with bogus data:"));
  //   //     SerialMon.println(nextRate);
  //   //   }
  //   // }

  // } else {
  //   // SerialMon.println(F("Response empty"));
  // }

  // Shutdown
  http.stop();
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  // Set console baud rate
  Serial.begin(115200);
  delay(100);
  
  // setup the screen logger with 8x8 font, log area, and newline (not per character) updates
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8x8log.setRedrawMode(0);

  u8x8log.println(F("Welcome!"));
  u8x8log.println(F("GPS Tracker Online..."));

  delay(1000);
  
  SerialMon.println("setup()");
  Serial.println("setup()");

  // turn off wifi & bluetooth to save power, I guess...
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  // set modem baud rate and connection details
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  modemPowerOn();

  SerialMon.println("> end setup()");

}

void loop()
{
  SerialMon.println(F("loop()"));
  Serial.println(F("loop()"));
  
  // if the modem failure is too high OR we've been through this loop a few times 
  // and STILL have no post success, let's do a full reset of the modem
  if (modemFailures >= maxModemFailures || postFailures > maxPostFailures * 2) {
    SerialMon.println(F("Failures high, resetting"));
    modem.gprsDisconnect();
    disableGPS();
    modemRestart();
    modemFailures = 0;
    postFailures = 0;
    SerialMon.println(F("> reset complete"));
  }

  if (!modem.isNetworkConnected()) {
    SerialMon.println(F("Not connected, restarting modem"));
    if (!modem.restart()) {
      SerialMon.println(F("> failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    SerialMon.println(F("> restarted"));
  }
  
  SerialMon.println(F("Awaiting network"));
  if (!modem.waitForNetwork(600000L)) {
    SerialMon.println(F("> failed"));
    modemFailures++;
    delay(1000);
    return;
  }
  SerialMon.println(F("> network ready"));

  // Establish GPRS (data/tcp/etc) connection, as needed
  if (!modem.isGprsConnected()) {
    SerialMon.print(F("Connecting to APN: "));
    SerialMon.println(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(F("> failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    SerialMon.println(F("> success"));
  }
  SerialMon.println(F("GPRS Ready"));

  // turn on gps now that modem is running and GPRS is up (which needs to happen first, apparently?)
  enableGPS();

  SerialMon.println(F("Lock and fetch GPS location"));
  // todo: perhaps wait till accuracy is high enough? enough usat's?
  while (1) {
    if (modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      if (gpsLatitude == 0 && gpsLongitude == 0) {
        SerialMon.print(F("err: lat/long 0/0 -- reset"));
        modemFailures = maxModemFailures; // will cause a modem power cycle on next loop()
        break; // breaks the while, allowing for the next loop()
      } else {
        send_data();
      }
      delay(loopRate);
    } else {
      SerialMon.print(F("awaiting lock, ms:"));
      SerialMon.println(millis());
      delay(2000);
    }
    // if (resetRequest) {
    //   SerialMon.print(F("reset request initiated; resetting/restarting modem & gps"));
    //   resetRequest = false; // handled, set back to normal
    //   modemFailures = maxModemFailures; // will cause a modem power cycle on next loop()
    //   break;
    // }
    if (postFailures >= maxPostFailures) {
      SerialMon.println(F("too many post fail, restart"));
      break;
    }
  }

}
