/*  refactoring simple care
     http://download.labs.mediatek.com/package_mtk_linkit_7697_index.json
   https://paper.dropbox.com/doc/Simple-Care-Server-Docs-DUxlaVIZeUCDF6gVN4JXQ
   https://docs.google.com/spreadsheets/d/1vbx1XJ_iKMXrqE0wVKvDYtStpL_afLJsG3Qn_Tgm8OI/edit#gid=0
   https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/247015
   https://makerpro.cc/author/sco-lin/
   for linkit 7697

   TODO:
   EEPROM to save ssid, password... with crc8
   https://github.com/dhylands/projects/blob/master/common/Crc8.c
   AP mode to configure ssid, password...
   receive switch input signal
   sync NTP time
   http://yhhuang1966.blogspot.tw/2016/07/ntp-arduino.html
   http://forum.arduino.cc/index.php?topic=58915.0
   Linkit remote to configure ssid, password...
  https://docs.labs.mediatek.com/resource/linkit7697-arduino/en/developer-guide/using-linkit-remote
  serial.read() for command line reader
  software serial
  heater

  indoor location in nodejs
*/


#include "limits.h"
#include "Arduino.h"
#include "LTimer.h"
#include "LWiFi.h"
#include <LWatchDog.h>
#include <PubSubClient.h>
#include <LBLE.h>
#include <LBLECentral.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <hal_wdt.h>
#include <stdio.h>
#include <time.h>
#include <Regexp.h> //https://forum.arduino.cc/index.php?topic=59917.0



const short ultrasonic_trig = 5;
const short ultrasonic_echo = 6;



const short usr_led_pin = 7; // the USR led on linkit7697 board
const short usr_button_pin = 6; // the USR button on linkit7697 board

const char* wifi_ssid = ""; //your wifi ssid
const char* wifi_password = ""; //your wifi password
const char* mqtt_server = ""; //your cloud server IP, if using localhost, please check CMD ipconfig.
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_topic = "simplecare/upload";
const char* mqtt_subtopic = "simplecare/download";
const char* this_ap_name = "Simple Care"; //identify of this linkit7697 board
const char* this_ap_password = "12345678";

unsigned long usr_led_status = 0;
unsigned long loop_counter = 0;
unsigned long timer_counter = 0;

char *array[5];

// instantiation
LTimer timer0(LTIMER_0);

WiFiClient wifi_client;
PubSubClient pubsub_client(wifi_client);

WiFiServer wifi_server(80);

void reset_linkit7697() {
  Serial.println("Restarting linkit7697");
  hal_wdt_config_t wdt_config;
  wdt_config.mode = HAL_WDT_MODE_RESET;
  wdt_config.seconds = 0;
  hal_wdt_init(&wdt_config); //set WDT as t0 mode.
  hal_wdt_enable(HAL_WDT_ENABLE_MAGIC);
  while (1) {
    Serial.println("Restarting");
    hal_wdt_software_reset();
    delay(100);
  }
}

// 讀取資料，1頁 50 bytes
String EEPROM_read(int _page = 0, int _length = 50) {
  int _address = _page * 52;
  char _str;
  String read_buffer = "";

  // 超出頁面
  if (_length > 51) {
    Serial.println("Out Of Pages");
  } else {
    for ( int _i = 0; _i < _length; _i++ ) {
      _str = EEPROM.read(_address + _i);
      read_buffer += (String)_str;
    }
  }
  return read_buffer;
} // end of EEPROM_read()

// 寫入資料，1頁 50 bytes
bool EEPROM_write(char* _str, int _page, int _length) {
  int _address = _page * 52;
  if (_length > 51) {                // 超出頁面
    Serial.println("Out Of Pages");
    return false;
  } else {
    // Serial.print("Writing data：");
    for ( int _i = 0; _i < _length; _i++ ) {
      EEPROM.update(_i, _str[_i]);
      // Serial.print(_str[_i]);
    }
    // Serial.println();
    return true;
  } // end if
} // end of EEPROM_write()

void load_EEPROM() {
  String page_11 = EEPROM_read(0, 50); // 把第0頁 前50個byte都讀出來

  char charBuf11[50];
  page_11.toCharArray(charBuf11, 50);

  int i = 0;
  char *p = strtok (charBuf11, "\\");
  while (p != NULL)  {
    array[i] = p;
    p = strtok (NULL, "\\");
    // Serial.println(array[i]);//for string to char
    i++;
  }
  i = 0;

  String myString = String(array[0]);

  //前面有個v才代表eeprom已驗證有設過資料
  if (myString == "v")   {
    Serial.println("vertified");
    Serial.println(array[1]);
    Serial.println(array[2]);
    //connectWiFi(array[1], array[2]); //撈EEPROM的帳號密碼後連線AP
    //flag0 = 0;
    //flag1 = 0;
    //flag2 = 1;
  }  else  {
    Serial.println("not vertified");//for string to char
    //flag0 = 0;
    //flag1 = 1;
  }
}


// callback function for timer0
void timer_callback(void *usr_data) {
  usr_led_status = !usr_led_status;
  digitalWrite(usr_led_pin, usr_led_status);
  if (timer_counter < ULONG_MAX - 10) {
    timer_counter++;
  } else {
    timer_counter = 0;
  }

  //Serial.print("-"); // just for debug this timer
  return;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

unsigned long next_mqtt_timer = 0;

void keep_mqtt() {
  //Serial.println("in keep_mqtt");
  //if (next_mqtt_timer <= timer_counter) return; //FIXME
  while (!wifi_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    pubsub_client.setServer(mqtt_server, 1883);
    pubsub_client.setCallback(mqtt_callback);
    // Create a random client ID
    String clientId = "M";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (pubsub_client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.print("mqtt connected");
      // Once connected, publish an announcement...
      pubsub_client.publish(mqtt_topic, "hello world");
      // $ mosquitto_sub -h kay.dlinkddns.com -d -t hello/add_data
      // ... and resubscribe
      pubsub_client.subscribe(mqtt_subtopic);
      // $ mosquitto_pub -h kay.dlinkddns.com -d -t hello/world -m "from raspberrypi"
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubsub_client.state());
      next_mqtt_timer = timer_counter + 10;
    }
  }
}


void keep_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("connecting wifi... ");
    //WiFi.disconnect();
    //delay(1000);
    WiFi.begin(wifi_ssid, wifi_password);
    // FIXME
    //IPAddress wifi_dns =
    //WiFi.setDNS("8.8.8.8","168.95.192.1");
  } else {
    //Serial.print(" wifi connected ");
  }
}

void refresh_connections() {
  if (timer_counter % 3 != 0) return;
  //Serial.println("in refresh_connections");
  keep_wifi();
  keep_mqtt();
  pubsub_client.loop();
}

void publish_mqtt() {
  if (timer_counter % 1 != 0) return;
  char publishing [1000] = {'\0'};
  char counter_str[50];
  strcat(publishing, this_ap_name);
  strcat(publishing, "\\");
  sprintf(counter_str, "%d\\", loop_counter);
  strcat(publishing, "sys ");
  strcat(publishing, counter_str);
  sprintf(counter_str, "%d\\", timer_counter);
  strcat(publishing, "timer ");
  strcat(publishing, counter_str);
  Serial.print(publishing);
  pubsub_client.publish(mqtt_topic, publishing);
}

unsigned long ble_phase = 0;
unsigned long ble_stop_scan = 0;
char scanned_ble[100][100];

//TODO: reducing phases
void scan_ble() {
  //Serial.print("in scan_ble:");
  Serial.print(ble_stop_scan);
  Serial.print("/");
  Serial.print(ble_phase);
  Serial.print("/");
  Serial.print(LBLECentral.getPeripheralCount());
  Serial.println("");
  switch (ble_phase) {
    case 0:
      LBLE.begin();
      ble_phase++;
      break;
    case 1:
      if (!LBLE.ready()) {
        Serial.print("initializing ble...");
        break;
      }
      Serial.print("BLE address: " + LBLE.getDeviceAddress().toString());
      //Serial.println("scanning BLE");
      LBLECentral.scan();
      ble_stop_scan = timer_counter + 3;
      ble_phase++;
      break;
    case 2:
      //Serial.print("waiting for ble scan...");
      if (ble_stop_scan >= timer_counter) {
        //Serial.print("waiting...");
        break;
      }
      ble_phase++;
      break;
    case 3:
      // list advertisements found.
      Serial.println(" BLE devices found:");
      Serial.println("idx\taddress\t\t\tflag\tRSSI");
      for (int i = 0; i < LBLECentral.getPeripheralCount(); ++i) {
        printDeviceInfo(i);
      }
      LBLECentral.stopScan();
      Serial.print("--scan stopped--");
      ble_stop_scan = timer_counter + 1;
      ble_phase = 1;
      break;
    default:
      ble_phase = 0;
      break;
  }
}


//FIXME: to publish each ble device info here
void printDeviceInfo(int i) {
  Serial.print(i);
  Serial.print("\t");
  Serial.print(LBLECentral.getAddress(i));
  Serial.print("\t");
  Serial.print(LBLECentral.getAdvertisementFlag(i), HEX);
  Serial.print("\t");
  Serial.print(LBLECentral.getRSSI(i));
  Serial.print("\t");
  const String name = LBLECentral.getName(i);
  Serial.print(name);
  if (name.length() == 0)  {
    Serial.print("(Unknown)");
  }
  Serial.print(" by ");
  const String manu = LBLECentral.getManufacturer(i);
  Serial.print(manu);
  Serial.print(", service: ");
  if (!LBLECentral.getServiceUuid(i).isEmpty()) {
    Serial.print(LBLECentral.getServiceUuid(i));
  } else {
    Serial.print("(no service info)");
  }

  if (LBLECentral.isIBeacon(i)) {
    LBLEUuid uuid;
    uint16_t major = 0, minor = 0;
    int8_t txPower = 0;
    LBLECentral.getIBeaconInfo(i, uuid, major, minor, txPower);

    Serial.print(" ");
    Serial.print("iBeacon->");
    Serial.print("  UUID: ");
    Serial.print(uuid);
    Serial.print("\tMajor:");
    Serial.print(major);
    Serial.print("\tMinor:");
    Serial.print(minor);
    Serial.print("\ttxPower:");
    Serial.print(txPower);
  }

  Serial.println();
}

void read_button() {
  //如果Linkit7697的按紐按下，重新設定AP帳密
  if (digitalRead(usr_button_pin)) {
    Serial.println();
    Serial.println("Reset WiFi...");
  }
}

// just for debug
void show_wl_status_t() {
  switch (WiFi.status()) {
    case WL_NO_SHIELD:
      Serial.println("WL_NO_SHIELD");
      break;
    case WL_IDLE_STATUS:
      Serial.println("WL_IDLE_STATUS");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("WL_NO_SSID_AVAIL");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("WL_SCAN_COMPLETED");
      break;
    case WL_CONNECTED:
      Serial.println("WL_CONNECTED");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("WL_CONNECT_FAILED");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("WL_CONNECTION_LOST");
      break;
    case WL_DISCONNECTED:
      Serial.println("WL_DISCONNECTED");
      break;
    default:
      Serial.println("wrong wl_status_t");
      break;
  }
}

// just for debug
void show_ap() {
  Serial.print("AP ready:");
  Serial.print(this_ap_name);
  Serial.print(", and please visit http://");
  Serial.print(WiFi.softAPIP());
  Serial.println();
  Serial.print("AP MAC=");
  Serial.print(WiFi.softAPmacAddress());
  Serial.print(", firmware:");
  Serial.print(WiFi.firmwareVersion());
  Serial.println();

  //Serial.println(WiFi.SSID());
  //Serial.println(WiFi.RSSI());
  show_wl_status_t();
  Serial.println(WiFi.softAPgetStationNum());

  //Serial.println(WiFi.localIP());
  //Serial.println(WiFi.subnetMask());
  //Serial.println(WiFi.gatewayIP());
  Serial.println("end");
}


char*parseHTTP_GET(char* request) {
  Serial.println("in parse http_get");
  MatchState ms;
  //ms.Target ("Testing: answer=42");
  //char result = ms.Match ("(%a+)=(%d+)", 0);
  ms.Target (request);
  char result = ms.Match ("GET .* HTTP/", 0);
  if (result == REGEXP_MATCHED) {
    char buf [100];  // large enough to hold expected string
    Serial.println(ms.GetMatch(buf));
  } else if (result == REGEXP_NOMATCH) {
    Serial.println("no match");
  }
}

void ap_mode() {
  Serial.print("Configuring access point... ");

  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(this_ap_name, this_ap_password);
  wifi_server.begin();

  // listen for incoming clients
  while (true) {
    WiFiClient wifi_client = wifi_server.available();
    delay(500);
    if (wifi_client) {
      Serial.println("Someone connected!");

      while (wifi_client.connected()) {
        if (wifi_client.available()) {

          String req = wifi_client.readString();
          char charBuf[50];//for string to char
          int req_len = req.length() + 1; //for string to char
          req.toCharArray(charBuf, req_len);//for string to char

          Serial.println(charBuf);//for string to char
          Serial.println("testing regex");
          Serial.println(parseHTTP_GET(charBuf));

        }
      } // end of while()
    } else {
      Serial.print(rand());
      Serial.print('\t');
      Serial.println("no client");
      show_ap();
    }
  }
}

unsigned long previous_millis = 0;

unsigned long measure_period() {
  unsigned long current_millis = millis();
  unsigned int period_length = current_millis - previous_millis;
  if (false) {
    Serial.print("period:");
    Serial.print(previous_millis);
    Serial.print(" ");
    Serial.print(current_millis);
    Serial.print(".");
  }
  previous_millis = millis();
  return period_length;
}

void setup() {

  //ap_mode();
  pinMode(usr_led_pin, OUTPUT);
  pinMode(usr_button_pin, INPUT);
  pinMode(ultrasonic_trig, OUTPUT);
  pinMode(ultrasonic_echo, INPUT);
  Serial.begin(115200);
  srand( time(NULL) );

  // initialization of timer0
  timer0.begin();
  timer0.start(1000, LTIMER_REPEAT_MODE, timer_callback, NULL);

  Serial.println("EEPROM_read: " + EEPROM_read(0, 50));

  //LWatchDog.begin(23); // wait for 23 seconds before rebooting
  //hint: never use a watchdog if your code never freezes.

  return;
}

void loop() {
  Serial.print("period:");
  Serial.print(measure_period());
  Serial.print(" ");
  Serial.println(measure_distance());
  //return;
  refresh_connections();
  publish_mqtt();
  scan_ble();
  read_button();

  delay(1000); //TODO: how to decrease?
  if (loop_counter < ULONG_MAX - 10) {
    loop_counter++;
  } else {
    loop_counter = 1;
  }

  //LWatchDog.feed();

  return;
}

unsigned long ultrasonic_previous = 0;

int measure_distance() {
  if (millis() - ultrasonic_previous < 200) {
    delay(200 - (millis() - ultrasonic_previous));
  }
  digitalWrite(ultrasonic_trig, HIGH);
  delayMicroseconds(100);
  digitalWrite(ultrasonic_trig, LOW);
  ultrasonic_previous = millis();
  int distance = (pulseIn (ultrasonic_echo, HIGH) / 2) / 29;
  if (false) {
    Serial.print("distance = ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  return distance;
}


/*
   https://docs.labs.mediatek.com/linkit-7697-blocklyduino/linkit-7697-blocklyduino-12879411.html
*/


