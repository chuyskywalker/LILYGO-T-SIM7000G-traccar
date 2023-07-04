/*

Tracker - ESP32 GPS Location Tracker w/ Traccar Integration

Setup a Traccar instance, populate the `config.h` file,
pop in your sim, and you're off to the races.

This project is based on a LILYGO T-SIM7000G. While it's a
nice all-in-one solution, as I intended to put this in a
vehicle, I didn't want to put in the effort to make the
battery sub-system work. Frankly, it has some off behaviors
anyway, which make it troublesome to work with.

As such, this is a pretty basic "turn on, report rather fast"
type approach. The idea is that you plug it into a cars 12v
active system (with a step down converter) and when the car
turns on, this starts tracking.

This sketch also includes support for outputting to an oled
screen so you can get an idea of the operational state of
the device. I used the SSD1322 type screen.

I originally built this with the OSMAND protocol and
ArduinoHttpClient, but I had enormous trouble getting that
to work without timeouts measured in minutes and > 10%
failure-to-report rates. Swapping to a TCP protocol has
proven far more reliable.

*/

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <Arduino.h>
#include <WiFi.h>

// #include <StreamDebugger.h>
// StreamDebugger debugger(Serial1, Serial);
// TinyGsm modem(debugger);

TinyGsm modem(Serial1);
TinyGsmClient client(modem);

#define UART_BAUD 115200
#define PIN_DTR 25 // reset pin, maybe? Not sure
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

#include "config.h";

#include <U8x8lib.h>

U8X8_SSD1322_NHD_256X64_4W_SW_SPI u8x8(/* clock=*/ 18, /* data=*/ 19, /* cs=*/ 5, /* dc=*/ 22, /* reset=*/ 21);

// screen properties
struct screen {
  String imei = "";
  bool hasNetwork = false;
  bool hasGPRS = false;
  bool hasGPS = false;
  bool hasTCP = false;
  String lastTime = "????/??/?? ??:??:??";
  float lastLat = 0;
  float lastLon = 0;
  int submissionCount = 0;
};

screen screenInfo;

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_ / SECS_PER_HOUR) % SECS_PER_HOUR) 

void screenDraw(void * pvParameters) {
  bool first = false;
  while(1) {

    if (!first) {
      // turn on the screen
      u8x8.begin();
      delay(100);

      u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
      u8x8.drawString(0, 0, "Tracker");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(32-6, 1, "LTE: ");
      u8x8.drawString(32-7, 2, "GPRS: ");
      u8x8.drawString(32-6, 3, "GPS: ");
      u8x8.drawString(32-6, 4, "TCP: ");
      u8x8.drawString(0, 2, "IMEI: ");
      u8x8.drawString(0, 4, "Last Report:");
      first = true;
    }

    long val = millis() / 1000;
    
    int hours   = numberOfHours(val);
    int minutes = numberOfMinutes(val);
    int seconds = numberOfSeconds(val);

    String dhms = pad(hours) + ":" + pad(minutes) + ":" + pad(seconds);
    u8x8.drawString(15, 1, dhms.c_str());

    String subs_count = "Subs: " + String(screenInfo.submissionCount);
    u8x8.drawString(32-subs_count.length(), 0, subs_count.c_str());
    
    u8x8.drawString(32-1, 1, screenInfo.hasNetwork ? "Y" : "N");
    u8x8.drawString(32-1, 2, screenInfo.hasGPRS ? "Y" : "N");
    u8x8.drawString(32-1, 3, screenInfo.hasGPS ? "Y" : "N");
    u8x8.drawString(32-1, 4, screenInfo.hasTCP ? "Y" : "N"); 

    u8x8.drawString(2, 5, screenInfo.lastTime.c_str());

    String lastlatlon = String(screenInfo.lastLat, 6) + " / " + String(screenInfo.lastLon, 6);
    u8x8.drawString(2, 6, lastlatlon.c_str());
    
    u8x8.drawString(6, 2, screenInfo.imei == "" ? "Unknown" : screenInfo.imei.c_str());

    delay(200);
  }
}

void modemPowerOn() {
  Serial.println("turn on modem");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000); // Datasheet Ton mintues = 1S
  digitalWrite(PWR_PIN, LOW);
  Serial.println("> on");
  delay(10000); // give the modem 10 seconds to have its coffee
  Serial.println("> ready");
}

void enableGPS(void) {
  modem.sendAT("+CGPIO=0,48,1,1");
  if (modem.waitResponse(10000L) == 1) {
    Serial.println("Set GPS Power pin HIGH");
  } else {
    Serial.println("Set GPS Power pin HIGH FAILED");
  }
  if (modem.enableGPS()) {
    Serial.println("GPS turned on via AT");
  } else {
    Serial.println("GPS turned on via AT FAILED");
  }
  modem.sendAT("+CGNSHOR=10 ");
  if (modem.waitResponse(10000L) == 1) {
    Serial.println("GPS set to 10m resolution");
  } else {
    Serial.println("GPS set to 10m resolution FAILED");
  }
  Serial.println("> GPS Enabled");
}

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(1000);

  Serial.println("setup()");

  // turn off wifi & bluetooth to save power, I guess...
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  // set modem bause rate and connection details
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(100);

  // start screen drawing thread
  // NOTE: Do all screen operations in thread; I'm pretty sure 
  //       the screen goes haywire if the two threads try to 
  //       write at the same time.
  xTaskCreatePinnedToCore(
    screenDraw,         /* Task function. */
    "screen",           /* name of task. */
    10000,              /* Stack size of task */
    NULL,               /* parameter of the task */
    1,                  /* priority of the task */
    NULL,               /* Task handle to keep track of created task */
    0);                 /* pin task to core 0 */

  // give it the juice
  modemPowerOn();

  // Always run a restart; sometimes modem connection from last build gets stuck
  modem.restart();

  Serial.println("> end setup()");

}

// sprintf? never heard of it...
String pad(int val) {
  if (val < 10) {
    return "0" + String(val);
  }
  return String(val);
}

int nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond, usat = 0, vsat = 0;
float gpsLatitude = 0, gpsLongitude = 0, gpsSpeed = 0,  gpsAltitude = 0, gpsAccuracy = 0;

void loop() {

  if (!modem.isNetworkConnected()) {
    screenInfo.hasNetwork = false;
    Serial.println(F("Not connected, restarting modem"));
    if (!modem.restart()) {
      Serial.println(F("> failed"));
      delay(1000);
      return;
    }
    Serial.println(F("> restarted"));
  }

  // wait for, like, useful shit
  Serial.println(F("Awaiting network"));
  if (!modem.waitForNetwork(600000L)) {
    Serial.println(F("> failed"));
    screenInfo.hasNetwork = false;
    delay(1000);
    return;
  }
  screenInfo.hasNetwork = true;
  Serial.println(F("> network ready"));

  // Establish GPRS (data/tcp/etc) connection, as needed
  if (!modem.isGprsConnected()) {
    screenInfo.hasGPRS = false;
    Serial.println(F("Connecting to APN"));
    if (!modem.gprsConnect("wholesale", "", "")) {
      Serial.println(F("> failed"));
      delay(1000);
      return;
    }
    Serial.println(F("> success"));
  }
  screenInfo.hasGPRS = true;
  Serial.println(F("GPRS Ready"));

  // locate a meeee
  enableGPS();

  // lets gooooo
  Serial.println(F("Lock and fetch GPS location"));
  
  while(1) {
    delay(15000);
    
    if (screenInfo.imei == "") {
      Serial.print(F("Updating IMEI, now:"));
      screenInfo.imei = modem.getIMEI();
      Serial.println(screenInfo.imei);
    }

    if (!modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, 
                      &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      Serial.print(F("awaiting lock, ms:"));
      Serial.println(millis());
      screenInfo.hasGPS = false;
      continue;
    }

    if (gpsLatitude == 0 && gpsLongitude == 0) {
      Serial.println(F("lat/long 0/0, that's some bulls..."));
      continue;
    }
    
    screenInfo.hasGPS = true;
    Serial.println("lat/long = " + String(gpsLatitude, 6) + "/" + String(gpsLongitude, 6));

    // ex:
    // $$357207059646786,4003,2015/05/19,15:55:27,-20.21421,-70.14920  , 33.6, 0.4,  0.0, 11, 0.8, 12.9, 31,297, 1, 0, 0.0 ,0.0,0,1,1,1##
    
    // from:
    //    https://github.com/traccar/traccar/blob/a8a06ffd494fc7161ca0edca39ae35be865a383f/src/main/java/org/traccar/protocol/XirgoProtocolDecoder.java#L53
    //
    // private static final Pattern PATTERN_OLD = new PatternBuilder()
    //         .text("$$")
    //         .number("(d+),")                     // imei
    //         .number("(d+),")                     // event
    //         .number("(dddd)/(dd)/(dd),")         // date (yyyy/mm/dd)
    //         .number("(dd):(dd):(dd),")           // time (hh:mm:ss)
    //         .number("(-?d+.?d*),")               // latitude
    //         .number("(-?d+.?d*),")               // longitude
    //         .number("(-?d+.?d*),")               // altitude
    //         .number("(d+.?d*),")                 // speed
    //         .number("(d+.?d*),")                 // course
    //         .number("(d+),")                     // satellites
    //         .number("(d+.?d*),")                 // hdop
    //         .number("(d+.d+),")                  // battery
    //         .number("(d+),")                     // gsm
    //         .number("(d+.?d*),")                 // odometer
    //         .number("(d+),")                     // gps
    //         .any()
    //         .compile();

    // $$123456789012345,4002,2023/07/03,03:10:37,37.534069,-122.285416,5.80,0.00,0,9,0,0,0,0,1##

    String data = "$$" + String(screenInfo.imei)
                + "," + String("4001")
                + "," + pad(nowYear) + "/"+ pad(nowMonth) + "/" + pad(nowDay)
                + "," + pad(nowHour) + ":"+ pad(nowMinute) + ":" + pad(nowSecond)
                + "," + String(gpsLatitude, 6)
                + "," + String(gpsLongitude, 6)
                + "," + String(gpsAltitude, 2)
                + "," + String(gpsSpeed, 2)
                + "," + String("0.0") // course?!?
                + "," + String(usat) // sats
                + "," + String("0.0") // hdop
                + "," + String("0.0") // battery (NOTE: *must* have decimal place)
                + "," + String("1") // gsm/rssi
                + "," + String("0") // odometer (mi)
                + "," + String("1") // gps (isvalid == 1)
                + ",0##"; // the pattern match will junk anything after the GPS valid flag, but it REQUIRES the comma to be there

    if (!client.connected()) {
      if (!client.connect(traccar_server, traccar_port, 5)) {
        Serial.println("Failed to TCP connect");
        screenInfo.hasTCP = false;
        continue; // try again in a moment 
      } else {
        Serial.println("TCP connected");
      }
    } else {
      Serial.println("TCP still connected");
    }
    screenInfo.hasTCP = true;

    Serial.println("Sending TCP data:");
    Serial.println(data);
    client.print(data);
    Serial.println("...sent");

    screenInfo.submissionCount++; 
    screenInfo.lastLat = gpsLatitude;
    screenInfo.lastLon = gpsLongitude;
    screenInfo.lastTime = pad(nowYear) + "/"+ pad(nowMonth) + "/" + pad(nowDay)
                  + " " + pad(nowHour) + ":"+ pad(nowMinute) + ":" + pad(nowSecond);

  }
  
}
