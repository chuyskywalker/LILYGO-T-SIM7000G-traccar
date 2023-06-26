/* 

An absurdly simple GPS tracker based on the LILYGO T-SIM7000G

Reports very quickly (10s) -- if you send the custom command "boost" 
it will report, basically, as fast as possible.

I desoldered the battery tray; I simply could not find a reason to 
keep it on, plus the way power worked on the board between usb vs 
solar charge I just wasn't happy with any of the setups available. 

On USB, no battery:
- get power, online
- no power, no online
On USB, w/ battery:
- no way to read voltage of battery
- charges battery (slowly)
- when USB power goes out, battery takes over (may or may not reboot)
- when battery runs dead, if USB receives power, system reboots
w/ solar, w/ battery:
- can read voltage at all times (charge or discharge)
- no way to determine if charging or discharging
- when battery runs too low, the solar port *can* recharge the battery, 
  but the device requires a physical off/on cycle to come back online

The "on usb, w/battery" appreach was the closest reasonable setup, but
wasn't really worth the hassle. If placed in a car, the battery would,
at full charge, probably only last 12-24 hours. And given the slow charge
rate, even with 2h of car driving, you still would always be down on power.

Thus, you might as well skip the battery since it'll almost always be without
juice anyway.

You COULD try to work around this by, if when on battery power, going into
esp deep sleeps for longer periods of time to really stretch the battery
life out, but all that does is keep telling me the car is parked. I suppose,
if someone stole the car by towing it away, this could be useful, but...

Ultimatley, I decided to just drop it and only have GPS reporting when the 
unit has power, which is fine for me. Slims up the overall profile of the 
mounting box quite a bit.

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

The intent of the code is to be robust and reliable without too 
much concern to power usage or data caps. It has multiple fallback
counters and will, in dire circumstances, restart all the critical
components from scratch (it literally turns the modem off and on again).

This, however, is my first ESP32/T-SIM project as, as such, I've
likely made a dog's breakfast of this in some critical fashion
and I just don't know it. :D

*/

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include "config.h"

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <UrlEncode.h>
#include <TimeLib.h>


#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>

AsyncWebServer server(80);

const char* ssid = ""; // Your WiFi AP SSID 
const char* password = ""; // Your WiFi Password

bool resetRequest = false;

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "reset") {
    resetRequest = true;
  }
}

// #define SerialMon WebSerial
#define SerialMon Serial

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
  SerialMon.println("turn on modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000); // Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, HIGH);
  delay(10000); // give the modem 10 seconds to have its coffee
  SerialMon.println("...on");
}

void modemPowerOff() {
  SerialMon.println("turn off modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500); // Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
  SerialMon.println("...off");
}

void modemRestart() {
  SerialMon.println("Restart modem");
  modemPowerOff();
  modemPowerOn();
  SerialMon.println("...restarted");
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

void blink(uint c1, uint c2) {
  return;
  // digitalWrite(LED_PIN, HIGH);
  // uint done = 0;
  // while (done < (c1 * 2)) {
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   delay(250);
  //   done++;
  // }
  // delay(750);

  // done = 0;
  // while (done < (c2 * 2)) {
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   delay(250);
  //   done++;
  // }

  // digitalWrite(LED_PIN, HIGH);
  // delay(1000);
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

  SerialMon.print(F("GPS POST: "));
  SerialMon.println(url);
  
  // set a 2 second timeout
  http.setHttpResponseTimeout(5000);

  int err = http.post(url);
  if (err != 0) {
    SerialMon.println(F("...failed to connect: "));
    SerialMon.println(err);
    postFailures++;
    return;
  }

  int status = http.responseStatusCode();
  if (!status) {
    SerialMon.println(F("...failed to submit: "));
    SerialMon.println(status);
    postFailures++;
    return;
  }
  
  // grab any response content; may be a command present
  String body = http.responseBody();
  if (body != "") {
    SerialMon.println(F("Response:"));
    SerialMon.println(body);
    SerialMon.println(F("/EOF Response"));

    if (body == "boost") {
      SerialMon.println(F("CMD 'boost' -- Boosting reporting rate"));
      loopRate = loopRateBoost;
    } else if (body == "endboost") {
      SerialMon.println(F("CMD 'endboost' -- Returning to normal reporting rate"));
      loopRate = loopRateNormal;
    } else {
      SerialMon.println(F("No such command; no effect"));
    }

    // String loopRateCommand = "loopRate=";
    // uint cmdLen = loopRateCommand.length();
    // if (body.startsWith(loopRateCommand) && body.length() > cmdLen) {
    //   String nextRate = body.substring(cmdLen);
    //   int nextRateVal = nextRate.toInt();
    //   if (nextRateVal != 0) {
    //     SerialMon.println(F("Change loop rate to:"));
    //     SerialMon.println(nextRate);
    //     loopRate = nextRateVal;
    //   } else {
    //     SerialMon.println(F("Got loopRate with bogus data:"));
    //     SerialMon.println(nextRate);
    //   }
    // }

  } else {
    SerialMon.println(F("Response empty"));
  }

  // Shutdown
  http.stop();
  postFailures = 0;
  SerialMon.println(F("...posted"));
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  // Set console baud rate
  Serial.begin(115200);
  delay(100);
  
  SerialMon.println("setup()");
  
  // turn on wifi ap
  WiFi.softAP("lgo-ap", "obscurepsk");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  // end wifi ap

  // // connect to known wifi station
  // WiFi.mode(WIFI_STA);
  // WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // String hostname = "lgo";
  // WiFi.setHostname(hostname.c_str());
  // WiFi.begin("TUBES", "obscurepsk");
  // if (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //     Serial.printf("WiFi Failed!\n");
  //     return;
  // }
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());
  // // end wifi station

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

  // // Set LED OFF
  // digitalWrite(LED_PIN, HIGH);

  // set modem bause rate and connection details
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  modemPowerOn();

  SerialMon.println("...end setup()");

}

void loop()
{
  SerialMon.println(F("loop()"));
  
  // if the modem failure is too high OR we've been through this loop a few times 
  // and STILL have no post success, let's do a full reset of the modem
  if (modemFailures >= maxModemFailures || postFailures > maxPostFailures * 2) {
    SerialMon.println(F("too many failures, resetting modem/gps/etc"));
    modem.gprsDisconnect();
    disableGPS();
    modemRestart();
    modemFailures = 0;
    postFailures = 0;
    SerialMon.println(F("...reset complete"));
  }

  if (!modem.isNetworkConnected()) {
    SerialMon.println(F("Not connected, running a modem restart"));
    if (!modem.restart()) {
      SerialMon.println(F("...failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    SerialMon.println(F("...restarted"));
  }
  
  SerialMon.println(F("Awaiting network"));
  if (!modem.waitForNetwork(600000L)) {
    SerialMon.println(F("...failed"));
    modemFailures++;
    delay(1000);
    return;
  }
  SerialMon.println(F("...network ready"));

  // Establish GPRS (data/tcp/etc) connection, as needed
  if (!modem.isGprsConnected()) {
    SerialMon.print(F("Connecting to APN: "));
    SerialMon.println(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(F("...failed"));
      modemFailures++;
      delay(1000);
      return;
    }
    SerialMon.println(F("...success"));
  }
  SerialMon.println(F("GPRS Ready"));

  // turn on gps now that modem is running and GPRS is up (which needs to happen first, apparently?)
  enableGPS();

  SerialMon.println(F("Lock and fetch GPS location"));
  // todo: perhaps wait till accuracy is high enough? enough usat's?
  while (1) {
    if (modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      if (gpsLatitude == 0 && gpsLongitude == 0) {
        SerialMon.print(F("what the hell lat/long @ 0? No. Bogus data; do not submit; full reset"));
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
    if (resetRequest) {
      SerialMon.print(F("reset request initiated; resetting/restarting modem & gps"));
      resetRequest = false; // handled, set back to normal
      modemFailures = maxModemFailures; // will cause a modem power cycle on next loop()
      break;
    }
    if (postFailures >= maxPostFailures) {
      SerialMon.println(F("post failures high, restarting loop"));
      break;
    }
  }

}
