/* 

An absurdly simple GPS tracker based on the LILYGO T-SIM7000G

Reports very quickly (10s) and if you send the custom command "boost" 
will report, basically, as fast as possible.

I desoldered the battery tray; I simply could not find a reason to 
keep it on, plus the way power worked on the board between usb vs 
solar charge I just wasn't happy with any of the setups available. 

So I decided to just drop it and only have GPS reporting when the 
unit has power, which is fine for me. Slims up the overall profile
quite a bit.

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

#include "config.h"

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <UrlEncode.h>
#include <TimeLib.h>

TinyGsm modem(Serial1);

TinyGsmClient client(modem);
HttpClient http(client, traccar_server, traccar_port);

#define UART_BAUD 115200
#define PIN_DTR 25 // reset pin, maybe? Not sure
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#define LED_PIN 12

uint loopRateNormal = 15000; // 10 seconds
uint loopRateBoost  =  1000; // very fast, every 1 seconds
uint loopRate = loopRateNormal;

uint postFailures = 0;
uint modemFailures = 0;
uint maxPostFailures = 5;
uint maxModemFailures = 5;

void modemPowerOn() {
  Serial.println("turn on modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000); // Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, HIGH);
  delay(10000); // give the modem 10 seconds to have its coffee
  Serial.println("...on");
}

void modemPowerOff() {
  Serial.println("turn off modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500); // Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
  Serial.println("...off");
}

void modemRestart() {
  Serial.println("Restart modem");
  modemPowerOff();
  modemPowerOn();
  Serial.println("...restarted");
}

void enableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void) {
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

int nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond, usat = 0, vsat = 0;
float gpsLatitude = 0, gpsLongitude = 0, gpsSpeed = 0,  gpsAltitude = 0, gpsAccuracy = 0;

void send_data() {

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

  // compile the url
  String url = "/?id=" + urlEncode(modem.getIMEI())
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

  int err = http.post(url);
  if (err != 0) {
    Serial.println(F("...failed to connect: "));
    Serial.println(err);
    postFailures++;
    return;
  }

  int status = http.responseStatusCode();
  if (!status) {
    Serial.println(F("...failed to submit: "));
    Serial.println(status);
    postFailures++;
    return;
  }
  
  // grab any response content; may be a command present
  String body = http.responseBody();
  if (body != "") {
    Serial.println(F("Response:"));
    Serial.println(body);
    Serial.println(F("/EOF Response"));

    if (body == "boost") {
      Serial.println(F("CMD 'boost' -- Boosting reporting rate"));
      loopRate = loopRateBoost;
    } else if (body == "endboost") {
      Serial.println(F("CMD 'endboost' -- Returning to normal reporting rate"));
      loopRate = loopRateNormal;
    } else {
      Serial.println(F("No such command; no effect"));
    }

    // String loopRateCommand = "loopRate=";
    // uint cmdLen = loopRateCommand.length();
    // if (body.startsWith(loopRateCommand) && body.length() > cmdLen) {
    //   String nextRate = body.substring(cmdLen);
    //   int nextRateVal = nextRate.toInt();
    //   if (nextRateVal != 0) {
    //     Serial.println(F("Change loop rate to:"));
    //     Serial.println(nextRate);
    //     loopRate = nextRateVal;
    //   } else {
    //     Serial.println(F("Got loopRate with bogus data:"));
    //     Serial.println(nextRate);
    //   }
    // }

  } else {
    Serial.println(F("Response empty"));
  }

  // Shutdown
  http.stop();
  postFailures = 0;
  Serial.println(F("...posted"));
}

void setup()
{
  // Set console baud rate
  Serial.begin(115200);
  delay(100);
  
  Serial.println("setup()");

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // set modem bause rate and connection details
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  modemPowerOn();

  Serial.println("...end setup()");

}

void loop()
{
  Serial.println(F("loop()"));

  if (modemFailures >= maxModemFailures) {
    Serial.println(F("too many modem failures, resetting"));
    disableGPS();
    modemRestart();
    modemFailures = 0;
    Serial.println(F("...reset complete"));
  }

  if (!modem.isNetworkConnected()) {
    Serial.println(F("Not connected, running a modem restart"));
    if (!modem.restart()) {
      Serial.println(F("...failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    Serial.println(F("...restarted"));
  }
  
  Serial.println(F("Awaiting network"));
  if (!modem.waitForNetwork(600000L)) {
    Serial.println(F("...failed"));
    modemFailures++;
    delay(1000);
    return;
  }
  Serial.println(F("...network ready"));

  // Establish GPRS (data/tcp/etc) connection, as needed
  if (!modem.isGprsConnected()) {
    Serial.print(F("Connecting to APN: "));
    Serial.println(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      Serial.println(F("...failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    Serial.println(F("...success"));
  }
  Serial.println(F("GPRS Ready"));

  // turn on gps now that modem is running and GPRS is up (which needs to happen first, apparently?)
  enableGPS();

  Serial.println("Lock and fetch GPS location");
  // todo: perhaps wait till accuracy is high enough? enough usat's?
  while (1) {
    if (modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      if (gpsLatitude == 0 && gpsLongitude == 0) {
        Serial.print("what the hell lat/long @ 0? No. Bogus data; do not submit; full reset");
        modemFailures = maxModemFailures; // will cause a modem power cycle on next loop()
        break; // breaks the while, allowing for the next loop()
      } else {
        send_data();
      }
      delay(loopRate);
    } else {
      Serial.print("awaiting lock, ms:");
      Serial.println(millis());
      delay(2000);
    }
    if (postFailures >= maxPostFailures) {
      Serial.println("post failures high, restarting loop");
      break;
    }
  }

}
