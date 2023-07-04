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

  // give it the juice
  modemPowerOn();

  // Always run a restart; sometimes modem connection from last build gets stuck
  modem.restart();

  Serial.println("> end setup()");

}

String pad(int val) {
  if (val < 10) {
    return "0" + String(val);
  }
  return String(val);
}

int nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond, usat = 0, vsat = 0;
float gpsLatitude = 0, gpsLongitude = 0, gpsSpeed = 0,  gpsAltitude = 0, gpsAccuracy = 0;

// xirgo -- works as a persistent connection, start it up and keep using it?!?!
byte server[]  = { /* 1,2,3,4 your ip here*/ };
const int port = 5081;

uint odo = 10;

void loop() {

  if (!modem.isNetworkConnected()) {
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
    delay(1000);
    return;
  }
  Serial.println(F("> network ready"));

  // Establish GPRS (data/tcp/etc) connection, as needed
  if (!modem.isGprsConnected()) {
    Serial.println(F("Connecting to APN"));
    if (!modem.gprsConnect("YOUR_APN_HERE", "", "")) {
      Serial.println(F("> failed"));
      delay(1000);
      return;
    }
    Serial.println(F("> success"));
  }
  Serial.println(F("GPRS Ready"));

  // locate a meeee
  enableGPS();

  // lets gooooo
  Serial.println(F("Lock and fetch GPS location"));

  if (!client.connect(server, port, 60)) {
    Serial.println("Failed to TCP connect");
    return;
  } else {
    Serial.println("TCP connected");
  }
  
  while(1) {
    delay(5000);

    if (!client.connected()) {
      Serial.println("Lost the connection; restart the loop");
      client.stop(); // probably not needed but...
      return;
    } else {
      Serial.println("TCP still connected");
    }

    if (!modem.getGPS(&gpsLatitude, &gpsLongitude, &gpsSpeed, &gpsAltitude, &vsat, &usat, &gpsAccuracy, &nowYear, &nowMonth, &nowDay, &nowHour, &nowMinute, &nowSecond)) {
      Serial.print(F("awaiting lock, ms:"));
      Serial.println(millis());
      continue;
    }

    if (gpsLatitude == 0 && gpsLongitude == 0) {
      Serial.println(F("lat/long 0/0, that's some bulls..."));
      continue;
    }
    
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

    odo++;

    String data = "$$" + String("123456789012345") // todo: fix imie later 
                + "," + String("4001")
                + "," + pad(nowYear) + "/"+ pad(nowMonth) + "/" + pad(nowDay)
                + "," + pad(nowHour) + ":"+ pad(nowMinute) + ":" + pad(nowSecond)
                + "," + String(gpsLatitude + (odo/100.0), 6)
                + "," + String(gpsLongitude, 6)
                + "," + String(gpsAltitude, 2)
                + "," + String(gpsSpeed, 2)
                + "," + String("0.0") // course?!?
                + "," + String(usat) // sats
                + "," + String("0.0") // hdop
                + "," + String("0.0") // battery NOTE: *must* have decimal place
                + "," + String("1") // gsm/rssi
                + "," + String(odo) // odometer (mi)
                + "," + String("1") // gps (isvalid == 1)
                + ",0##"; // the pattern match junks anything after the GPS valid flag, but it REQUIRES the comma to be there

    // if (!client.connect(server, port, 60)) {
    //   Serial.println("Failed to TCP connect");
    //   continue;
    // } else {
    //   Serial.println("TCP connected");
    // }

    Serial.println("Sending TCP data:");
    Serial.println(data);

    client.print(data + "\r\n"); // report the data and crlf in a single message, not multiple sends like println() does behind the scenes
    Serial.println("...sent");
    // delay(250);
    // client.flush();
    // client.stop();

    // Serial.println("--- reply ---");
    // uint32_t timeout = millis();
    // while (client.connected() && millis() - timeout < 5000L) { // there is no response ever, but just in case...
    //   // Print available data
    //   while (client.available()) {
    //     char c = client.read();
    //     Serial.print(c);
    //     timeout = millis(); // reset timeout on each read
    //     delay(10);
    //   }
    // }
    // Serial.println();
    // Serial.println("--- /reply ---");

  }
  
}
