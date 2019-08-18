#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <string.h>
#include <Wire.h> //IIC
#include <math.h>
#include <freertos/timers.h>
#include "DHT.h"
#include <WiFiMulti.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include <stdlib.h>  //atoi()函数头文件
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "Bitmap.h"
#include "SD_card.h"
#include <ESP32_Servo.h>

const char* AP_SSID  = "ESP32_2AM77";
const char* AP_PASS  = "12345678";
#define ROOT_HTML  "<!DOCTYPE html><html><head><title>WIFI Config by lwang</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head><style type=\"text/css\">.input{display: block; margin-top: 10px;}.input span{width: 100px; float: left; float: left; height: 36px; line-height: 36px;}.input input{height: 30px;width: 200px;}.btn{width: 120px; height: 35px; background-color: #000000; border:0px; color:#ffffff; margin-top:15px; margin-left:100px;}</style><body><form method=\"GET\" action=\"connect\"><label class=\"input\"><span>WiFi SSID</span><input type=\"text\" name=\"ssid\"></label><label class=\"input\"><span>WiFi PASS</span><input type=\"text\"  name=\"pass\"></label><input class=\"btn\" type=\"submit\" name=\"submit\" value=\"Submie\"></form></body></html>"
WebServer server(80);
WiFiMulti wifiMulti;

//临时存储WIFI，用于回连
String wifi_ssid ;
String wifi_pass ;

//NTP服务器
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);
typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  String FormattedTime;
} Sys_time_;
Sys_time_ Sys_time {
  .hour = 0,
  .minute = 0,
  .second = 0,
  .FormattedTime = ""
};

typedef struct {
  bool Led; //照明
  bool Watering;  //滴灌
  bool Curtain; //卷帘
  bool Exhaust; //排风
  bool Auto;  //自动
  uint8_t L_humidity; //湿度低阀
  uint8_t H_humidity; //湿度高阀
  int L_light;  //亮度低阀
  int H_light;  //亮度高阀
} Switch_;

Switch_ Switch{
  .Led = false,
  .Watering = false,
  .Curtain = false,
  .Exhaust = false,
  .Auto = false,
  .L_humidity = 0,
  .H_humidity = 0,
  .L_light = 0,
  .H_light = 0
};

uint8_t Set_key_mun = 0;

#define led_relay_pin         13
#define pump_relay_pin        14
#define Exhaust_relay_pin     33
#define steering_engine_pin   27
Servo servo1;
// Published values for SG90 servos; adjust if needed
int minUs = 500;
int maxUs = 2400;

//GY-302
int BH1750address = 0x23; //SDA GPIO21 SCL GPIO22
byte buff[2];
uint16_t light_intensity = 0; //光照强度

//土壤湿度传感器
#define Moisture 32
float Soil_Humidity = 0;//土壤湿度

//SD卡
const int cs = 5;
//TFT pins
#define TFT_CS         4
#define TFT_RST        16
#define TFT_DC         5
// For 1.44" and 1.8" TFT with ST7735 (including HalloWing) use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
//#define TFT_MOSI 23  // Data out
//#define TFT_SCLK 18  // Clock out
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//MQ135
#define ANALOGPIN  35
float air_quality = 0;//ppm11

//DH11
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
float Humidity = 0, Temperature = 0;

//风速
#define airspeed_interruptpin 12 //中断引脚
uint16_t airspeed_interrupt_count = 0;//码盘中断计数
float air_speed = 0.0;  //风速
void airspeedinterruptfunction();

//key
#define KEY_1 26
uint32_t key_loop = 0;
Ticker ticker_key;//声明Ticker对象
void Key_Handle_Callback(void);

const char* mqtt_server = "183.230.40.39";  //服务器IP
const char* Applet_PID = "242430";
const char* Device_ID  = "526793465";
const char* APIKey     = "YCfiCBYavQKs=DhW3vJKSoGCQMI=";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
long Sensor_delay_1s = 0;
long MQ135_delay_2s = 0;
long One_minute_delay = 0;

uint8_t resr_count_down = 120;//重启倒计时s

TimerHandle_t xTimer_rest;
void restCallback(TimerHandle_t xTimer );
TimerHandle_t xTimer_connect_status;
void connect_status_Callback(TimerHandle_t xTimer );
TimerHandle_t xTimer_SysTime;
void xTimer_SysTime_Callback(TimerHandle_t xTimer );
TimerHandle_t xTimer_1S_display;
void xTimer_1S_display_Callback(TimerHandle_t xTimer );

uint8_t msg_buf[200];
char msgJson[200];
char debug_buf[200];
unsigned short json_len = 0;
uint8_t* packet_p;
uint8_t debug_buffer_start_index = 0;
void send_one_data(char* id, float temp);

void callback(char* topic, byte* payload, unsigned int length) {  //服务器指令回调
  int num_buff = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    //Serial.printf("payload[%d]=%d\r\n", i, atoi(payload[i]));
  }
  Serial.println();
  Serial.printf("payload=%d", atoi((char *)(&payload[2])));

  switch ((char)payload[0]) {
    case 'l':
      if ((char)payload[1] == '0') {
        Switch.Led = 0;
        digitalWrite(led_relay_pin, LOW);
        send_one_data("switch_1", 0);
        Serial.println("led off");
      } else {
        Switch.Led = 1;
        digitalWrite(led_relay_pin, HIGH);
        send_one_data("switch_1", 1);
        Serial.println("led on");
      }
      break;
    case 'p':
      if ((char)payload[1] == '0') {
        Switch.Exhaust = 0;
        digitalWrite(Exhaust_relay_pin, LOW);
        send_one_data("switch_2", 0);
        Serial.println("排风 off");
      } else {
        Switch.Exhaust = 1;
        digitalWrite(Exhaust_relay_pin, HIGH);
        send_one_data("switch_2", 1);
        Serial.println("排风 on");
      }
      break;
    case 'd':
      if ((char)payload[1] == '0') {
        Switch.Watering = 0;
        digitalWrite(pump_relay_pin, LOW);
        send_one_data("switch_3", 0);
        Serial.println("滴灌 off");
      } else {
        Switch.Watering = 1;
        digitalWrite(pump_relay_pin, HIGH);
        send_one_data("switch_3", 1);
        Serial.println("滴灌 on");
      }
      break;
    case 'q':
      if ((char)payload[1] == '0') {
        Switch.Curtain = 0;
        send_one_data("switch_4", 0);
        Serial.println("卷帘 off");
        servo1.write(0);
      } else {
        Switch.Curtain = 1;
        send_one_data("switch_4", 1);
        Serial.println("卷帘 on");
        servo1.write(180);
      }
      break;
    case 'a':
      if ((char)payload[1] == '0') {
        Switch.Auto = 0;
        send_one_data("about", 0);
        Serial.println("自动 off");
      } else {
        Switch.Auto = 1;
        send_one_data("about", 1);
        Serial.println("自动 on");
      }
      break;
    case 'k':
      num_buff = atoi((char *)(&payload[2]));
      switch ((char)payload[1]) {
        case '1':
          if (Switch.H_humidity <= num_buff) {
            //Switch.L_humidity = Switch.H_humidity;
            send_one_data( "knob_1", Switch.L_humidity);
          } else {
            Switch.L_humidity = num_buff;
            send_one_data( "knob_1", Switch.L_humidity);
          }
          break;
        case '2':
          if (num_buff <= Switch.L_humidity) {
            //Switch.H_humidity = Switch.L_humidity;
            send_one_data( "knob_2", Switch.H_humidity);
          } else {
            Switch.H_humidity = num_buff;
            send_one_data( "knob_2", Switch.H_humidity);
          }
          break;
        case '3':
          if (Switch.H_light <= num_buff) {
            //Switch.L_light = Switch.H_light;
            send_one_data( "knob_3", Switch.L_light);
          } else {
            Switch.L_light = num_buff;
            send_one_data( "knob_3", Switch.L_light);
          }
          break;
        case '4':
          if (num_buff <= Switch.L_light) {
            //Switch.H_light  = Switch.L_light;
            send_one_data( "knob_4", Switch.H_light);
          } else {
            Switch.H_light = num_buff;
            send_one_data( "knob_4", Switch.H_light);
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(Device_ID, Applet_PID, APIKey)) { //One net user name as product ID , and password as APIKey
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      client.setServer(mqtt_server, 6002);  //not 1883 , one net use the port of 6002 as mqtt server
      client.connect(Device_ID, Applet_PID, APIKey);
      client.setCallback(callback);

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int BH1750_Read(int address);
void BH1750_Init(int address);
float getVoltage(int pin);

void setup() {
  pinMode(Moisture, INPUT);//定义Moisture为输入模式
  pinMode(ANALOGPIN, INPUT);
  pinMode(KEY_1, INPUT);
  pinMode(led_relay_pin, OUTPUT );
  pinMode(pump_relay_pin, OUTPUT );
  pinMode(Exhaust_relay_pin, OUTPUT );

  servo1.attach(steering_engine_pin, minUs, maxUs);

  servo1.write(0);
  //  ledcSetup(ledChannel, freq, resolution);
  //  ledcAttachPin(steering_engine_pin, ledChannel);
  //  ledcWrite(ledChannel, 100);

  SPI.begin();
  Serial.begin(115200);
  pinMode(airspeed_interruptpin, INPUT_PULLUP);
  attachInterrupt(airspeed_interruptpin, airspeedinterruptfunction, FALLING);//下降沿触发
  Serial.println("start interupt....");

  Serial.println("Initializing card...");
  if (!SD.begin(cs))
  {
    Serial.println("Card failed to initialize, or not present");
  }

  // OR use this initializer (uncomment) if using a 1.44" TFT:
  tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
  tft.fillScreen(ST77XX_BLACK); //清屏

  Wire.begin();//iic初始化

  Serial.println(F("DHTxx test!"));
  dht.begin();

  WiFi.mode(WIFI_AP);
  boolean result = WiFi.softAP(AP_SSID, AP_PASS);
  if (result)
  {
    IPAddress myIP = WiFi.softAPIP();
    Serial.println("");
    Serial.print("Soft-AP IP address = ");
    Serial.println(myIP);
    Serial.println(String("MAC address = ")  + WiFi.softAPmacAddress().c_str());

    tft.setTextColor(ST77XX_WHITE);
    tft.setTextWrap(true);
    tft.setCursor(0, 0);
    tft.println(String("") + "SSID:" + AP_SSID);
    tft.setCursor(0, 15);
    tft.println(String("") + "PASS:" + AP_PASS);
    tft.setCursor(0, 30);
    tft.println("Please use your mobile phone to connect to the WIFI above, and then open the url \"192.168.4.1\" with your browser." );
    tft.setCursor(0, 95);
    tft.println("waiting ...");

    xTimer_rest = xTimerCreate("xTimer_rest", 1000 / portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, restCallback);
    xTimerStart( xTimer_rest, 0 );

  } else {
    Serial.println("WiFiAP Failed");
    tft.setTextColor(ST7735_RED);
    tft.setTextWrap(true);
    tft.setCursor(0, 0);
    tft.println("WiFiAP Failed");
    delay(3000);
    ESP.restart();
  }

  xTimer_SysTime = xTimerCreate("xTimer_SysTime", 1000 / portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, xTimer_SysTime_Callback);
  xTimerStart(xTimer_SysTime, 0);  //此定时器用于WIFI回连

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  //首页
  server.on("/", []() {
    server.send(200, "text/html", ROOT_HTML);
  });

  //连接
  server.on("/connect", []() {

    server.send(200, "text/html", "<html><body><h1>successd,conning...</h1></body></html>");

    WiFi.softAPdisconnect(true);
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    Serial.println("WiFi Connect SSID:" + ssid + "  PASS:" + pass);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());

    resr_count_down = 120;
    xTimerStop(xTimer_rest, 0);

    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextWrap(true);
    tft.setCursor(0, 0);
    tft.println(String() + "Connecting wifi:" + ssid.c_str() + "...");

    uint8_t Connect_time = 0;

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      Connect_time ++;
      if (Connect_time > 80) {
        tft.setTextColor(ST7735_RED);
        tft.setTextWrap(true);
        tft.setCursor(0, 30);
        tft.println("Connection timeout, check input is correct or try again later!");
        delay(3000);
        ESP.restart();
      }
    }

    wifi_ssid = ssid;
    wifi_pass = pass;

    xTimer_connect_status = xTimerCreate("xTimer_connect_status", 5000 / portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, connect_status_Callback);
    xTimerStart(xTimer_connect_status, 0);  //此定时器用于WIFI回连

    ticker_key.attach_ms(20, Key_Handle_Callback); //按键扫描定时器

    tft.setTextColor(ST7735_GREEN);
    tft.setTextWrap(true);
    tft.setCursor(0, 30);
    tft.println("WIFI connection completed!");

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    randomSeed(micros());
    client.setServer(mqtt_server, 6002);  //not 1883 , one net use the port of 6002 as mqtt server
    client.connect(Device_ID, Applet_PID, APIKey);
    client.setCallback(callback);

    timeClient.begin(); //连接ntp
    timeClient.setTimeOffset(8 * 60 * 60);//校准时区

    if (timeClient.update()) {
      Serial.println("update time is OK!");
      Sys_time.hour = timeClient.getHours();
      Sys_time.minute = timeClient.getMinutes();
      Sys_time.second = timeClient.getSeconds();
      Sys_time.FormattedTime = timeClient.getFormattedTime();
      Serial.println(Sys_time.FormattedTime);
    } else {
      Serial.println("update time is loser !");
      timeClient.begin(); //连接ntp
    }

    xTimer_1S_display = xTimerCreate("xTimer_1S_dasplay", 1000 / portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, xTimer_1S_display_Callback);
    xTimerStart( xTimer_1S_display, 0 );
    tft.fillScreen(ST77XX_BLACK); //清屏
    Display_load_time();
    send_one_data("switch_1", 0); send_one_data("switch_2", 0); send_one_data("switch_3", 0); send_one_data("switch_4", 0);
    send_one_data("about", 0); send_one_data( "knob_1", 0); send_one_data( "knob_2", 0); send_one_data( "knob_3", 0); send_one_data( "knob_4", 0);
  });

  server.begin();
}

void loop() {

  server.handleClient();

  while (WiFi.status() == WL_CONNECTED) {

    if (!client.connected()) {
      if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_TIME) {
        tft.drawBitmap(110, 0, gImage_coon, 18, 18, ST7735_RED, ST7735_BLACK);
      }
      reconnect();
    }
    client.loop();
    if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_TIME) {
      tft.drawBitmap(110, 0, gImage_coon, 18, 18, ST7735_GREEN, ST7735_BLACK);
    }

    long now = millis();
    //    if (now - lastMsg > 3000) {
    //      lastMsg = now;
    //      char dataTemplete[200];
    //      String Send_data = String() + "{\"m_temperature\":" + Temperature + ",\"m_humidity\":" + Humidity + ",\"m_soil_humidity\":" + Soil_Humidity + ",\"light_intensity\":" + light_intensity + ",\"ambient_air\":" + air_quality + "}"; //+ ",\"wind_speed\":" + air_speed
    //      strcpy(dataTemplete, Send_data.c_str());
    //      Serial.print("Send_data len: ");
    //      Serial.println(Send_data.length());
    //      snprintf(msgJson, sizeof(msgJson), dataTemplete, strlen(dataTemplete));
    //      json_len = strlen(msgJson); //packet length count the end char '\0'
    //      msg_buf[0] = char(0x03); //palyLoad packet byte 1, one_net mqtt Publish packet payload byte 1, type3 , json type2
    //      msg_buf[1] = char(json_len >> 8); //high 8 bits of json_len (16bits as short int type)
    //      msg_buf[2] = char(json_len & 0xff); //low 8 bits of json_len (16bits as short int type)
    //
    //      memcpy(msg_buf + 3, msgJson, strlen(msgJson));
    //      msg_buf[3 + strlen(msgJson)] = 0;
    //      Serial.print("Publish message: ");
    //      Serial.println(msgJson);
    //      client.publish("$dp", msg_buf, 3 + strlen(msgJson), false); // msg_buf as payload length which may have a "0x00"byte
    //    }

    if (now - lastMsg > 1000) {
      lastMsg = now;
      float distance;
      float temp;
      temp = float(airspeed_interrupt_count / 20);   //1秒转动的格数
      distance = temp * 0.0753982; //temp*(24/1000)*3.14;  1秒转动的距离
      if (air_speed != distance) {
        air_speed = distance;   //速度
        send_one_data("wind_speed", air_speed);
      }
      airspeed_interrupt_count = 0;
      Serial.println(String("") + "air_speed: " + air_speed);
    }

    if (now - Sensor_delay_1s > 2000) {

      Sensor_delay_1s = now;

      BH1750_Init(BH1750address);

      float Sh = (4095 - analogRead(Moisture)) / 40.95; //获取土壤湿度
      if (Soil_Humidity != Sh) {
        Soil_Humidity = Sh;
        send_one_data("m_soil_humidity", Soil_Humidity);
      }
      Serial.print("Soil_Humidity = ");
      Serial.println(Soil_Humidity);

      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
      } else {
        if (Humidity != h ) {
          Humidity = h;
          send_one_data("m_humidity", Humidity);
        }
        if (Temperature != t) {
          Temperature = t;
          send_one_data("m_temperature", Temperature);
        }
        Serial.printf("Temperature=%f,Humidity=%f\r\n", Temperature, Humidity);
      }

      if (2 == BH1750_Read(BH1750address))
      {
        uint16_t light = ((buff[0] << 8) | buff[1]) / 1.2;
        Serial.print(light, DEC);
        Serial.println("[lx]");
        if (light_intensity != light) {
          light_intensity = light;
          send_one_data("light_intensity", light_intensity);
        }
      }

      float air = getVoltage(ANALOGPIN);
      if (air_quality != air) {
        air_quality = air;
        send_one_data("ambient_air", air_quality);
      }
      Serial.print(air_quality);
      Serial.println("ppm");
    }

    if (now - One_minute_delay > 60000) {
      One_minute_delay = now;
      if (timeClient.update()) {
        Serial.println("update time is OK!");
        Sys_time.hour = timeClient.getHours();
        Sys_time.minute = timeClient.getMinutes();
        Sys_time.second = timeClient.getSeconds();
        Sys_time.FormattedTime = timeClient.getFormattedTime();
        Serial.println(Sys_time.FormattedTime);
      } else {
        Serial.println("update time is loser !");
        timeClient.begin(); //连接ntp
      }
    }

    if (Switch.Auto == true) {
      if (Soil_Humidity < Switch.L_humidity && Switch.L_humidity < Switch.H_humidity && Switch.Watering == false) {
        Switch.Watering = true;
        send_one_data("switch_3", 1);
        digitalWrite(pump_relay_pin, HIGH);
      } else if (Soil_Humidity > Switch.H_humidity && Switch.H_humidity > Switch.L_humidity && Switch.Watering == true) {
        Switch.Watering = false;
        send_one_data("switch_3", 0);
        digitalWrite(pump_relay_pin, LOW);
      }
      if (light_intensity < Switch.L_light && Switch.L_light < Switch.H_light && Switch.Led == false) {
        Switch.Led = true;
        send_one_data("switch_1", 1);
        digitalWrite(led_relay_pin, HIGH);
      } else if (light_intensity > Switch.H_light && Switch.H_light > Switch.L_light && Switch.Led == true) {
        Switch.Led = false;
        send_one_data("switch_1", 0);
        digitalWrite(led_relay_pin, LOW);
      }
    }

  }
}

int BH1750_Read(int address) //读取光照强度
{
  int i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  return i;
}

void BH1750_Init(int address) //初始化光照传感器
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}

float getVoltage(int pin) {
  return (analogRead(pin) * 0.04882814);
  // This equation converts the 0 to 1023 value that analogRead()
  // returns, into a 0.0 to 5.0 value that is the true voltage
  // being read at that pin.
}

void restCallback(TimerHandle_t xTimer ) {  //长时间不访问WIFI Config 将复位设备
  resr_count_down --;
  Serial.print("resr_count_down: ");
  Serial.println(resr_count_down);
  if (resr_count_down < 1) {
    ESP.restart();
  }
}

void connect_status_Callback(TimerHandle_t xTimer ) //用于WIFI断开后回连
{
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected");
  } else {
    wifiMulti.addAP(wifi_ssid.c_str(), wifi_pass.c_str());
    Serial.println("Connecting Wifi...");
    Serial.println(wifi_ssid);
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
}

void Key_Handle_Callback(void)  //按键处理
{
  if (digitalRead(KEY_1) == 1 ) {
    key_loop ++;
    if (key_loop == 100)  //长按
    {
      tft.fillScreen(ST77XX_BLACK); //清屏
      Long_Key(Screen_Display_Struct.SCREEN_DISPLAY_STATUS);
      Serial.println("Buttons long press");
    }
  } else {
    if ((key_loop >= 1) && (key_loop < 100))  //短按
    {
      if (Screen_Display_Struct.NO_SHORT_KEY_HANDLE == false) {
        if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS < OLED_DISPLAY_SECOND_LAYER) {
          Screen_Display_Struct.SCREEN_DISPLAY_STATUS ++;
          if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == OLED_DISPLAY_SECOND_LAYER) {
            Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_TIME;
          }
        } else if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS > OLED_DISPLAY_SECOND_LAYER && Screen_Display_Struct.SCREEN_DISPLAY_STATUS < (OLED_DISPLAY_END - 1)) {
          Screen_Display_Struct.SCREEN_DISPLAY_STATUS ++;
          Serial.println("二级界面");
        } else {
          Screen_Display_Struct.SCREEN_DISPLAY_STATUS = OLED_DISPLAY_SECOND_LAYER - 1;
          Serial.println("二级界面结束");
        }
      }
      tft.fillScreen(ST77XX_BLACK); //清屏
      Short_Key(Screen_Display_Struct.SCREEN_DISPLAY_STATUS);
      Serial.println("Buttons short press");
    }
    key_loop = 0;
  }
  //Serial.printf("key_loop:%d\r\n",key_loop);
}

void airspeedinterruptfunction() {
  airspeed_interrupt_count++;
}

void xTimer_SysTime_Callback(TimerHandle_t xTimer )
{
  Sys_time.second ++;
  if (Sys_time.second > 59) {
    Sys_time.second = 0;
    Sys_time.minute ++ ;
    if (Sys_time.minute > 59) {
      Sys_time.minute = 0;
      Sys_time.hour ++;
    }
    if (Sys_time.hour > 23) {
      Sys_time.hour = 0;
    }
  }
}

void send_one_data(char *id, float temp) {
  char dataTemplete[200];
  String Send_data = String() + "{" + "\"" + id + "\"" + ":" + temp + "}";
  Serial.println(Send_data);
  strcpy(dataTemplete, Send_data.c_str());
  Serial.print("Send_data len: ");
  Serial.println(Send_data.length());
  snprintf(msgJson, sizeof(msgJson), dataTemplete, strlen(dataTemplete));
  json_len = strlen(msgJson); //packet length count the end char '\0'
  msg_buf[0] = char(0x03); //palyLoad packet byte 1, one_net mqtt Publish packet payload byte 1, type3 , json type2
  msg_buf[1] = char(json_len >> 8); //high 8 bits of json_len (16bits as short int type)
  msg_buf[2] = char(json_len & 0xff); //low 8 bits of json_len (16bits as short int type)

  memcpy(msg_buf + 3, msgJson, strlen(msgJson));
  msg_buf[3 + strlen(msgJson)] = 0;
  Serial.print("Publish message: ");
  Serial.println(msgJson);
  client.publish("$dp", msg_buf, 3 + strlen(msgJson), false); // msg_buf as payload length which may have a "0x00"byte
}

void Short_Key(uint8_t DISPLAY_STATUS)
{
  Serial.print("Short_Key:");
  Serial.println(DISPLAY_STATUS);
  Serial.println(Screen_Display_Struct.NO_SHORT_KEY_HANDLE);
  if (DISPLAY_STATUS == DISPLAY_TIME)
  {
    tft.fillScreen(ST77XX_BLACK); //清屏
    Display_load_time();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_QR) {
    tft.fillScreen(ST77XX_BLACK); //清屏
    Display_QR();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_FUNCTION) {
    tft.fillScreen(ST77XX_BLACK); //清屏
    Display_Function();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_SET) {
    //    Screen_Display_Struct.NO_SHORT_KEY_HANDLE = true;
    tft.fillScreen(ST77XX_BLACK); //清屏
    //    Display_Set();
    Display_Set_Short_Key();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_INFO) {
    Display_QR();
    return;
  }
}

void Long_Key(uint8_t DISPLAY_STATUS)
{
  if (DISPLAY_STATUS == DISPLAY_TIME)
  {
    Display_load_time();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_FUNCTION) {
    Display_Set_Short_Key();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_SET) {
    Display_Set_Long_Key();
    Display_Set();
    return;
  } else if (DISPLAY_STATUS == DISPLAY_QR) {
    Display_Info();
    return;
  }
}

void xTimer_1S_display_Callback(TimerHandle_t xTimer )
{
  if (key_loop == 0) {  //保证没有按键事件正在处理，否则可能会花屏
    Serial.println("xTimer_1S_display_Callback");
    if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_TIME)
    {
      Display_load_time();
      return;
    } else if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_QR) {
      Display_QR();
      return;
    } else if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_FUNCTION) {
      Display_Function();
      return;
    } else if (Screen_Display_Struct.SCREEN_DISPLAY_STATUS == DISPLAY_SET) {
      Display_Set();
      return;
    }
  }
}

void listDir(fs::FS & fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS & fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS & fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS & fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS & fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS & fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS & fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS & fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

void Display_load_time(void)
{
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println(String() + (Sys_time.hour / 10) + (Sys_time.hour % 10) + ":" + (Sys_time.minute / 10) + (Sys_time.minute % 10) + ":" + (Sys_time.second / 10) + (Sys_time.second % 10));

  if (Switch.Led) {
    tft.drawBitmap(0, 100, gImage_ld, 18, 18, ST77XX_GREEN, ST7735_BLACK);
  } else {
    tft.drawBitmap(0, 100, gImage_ld, 18, 18, ST77XX_RED, ST7735_BLACK);
  }
  if (Switch.Watering) {
    tft.drawBitmap(32, 100, gImage_qs, 18, 18, ST77XX_GREEN, ST7735_BLACK);
  } else {
    tft.drawBitmap(32, 100, gImage_qs, 18, 18, ST77XX_RED, ST7735_BLACK);
  }
  if (Switch.Curtain) {
    tft.drawBitmap(64, 100, gImage_ql, 18, 18, ST77XX_GREEN, ST7735_BLACK);
  } else {
    tft.drawBitmap(64, 100, gImage_ql, 18, 18, ST77XX_RED, ST7735_BLACK);
  }
  if (Switch.Exhaust) {
    tft.drawBitmap(96, 100, gImage_pf, 18, 18, ST77XX_GREEN, ST7735_BLACK);
  } else {
    tft.drawBitmap(96, 100, gImage_pf, 18, 18, ST77XX_RED, ST7735_BLACK);
  }

  tft.setTextColor(ST77XX_WHITE, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(1);

  tft.startWrite();
  tft.writeFillRect(60, 20, 128 , 10, ST7735_BLACK);
  tft.writeFillRect(72, 30, 128 , 10, ST7735_BLACK);
  tft.writeFillRect(54, 40, 128, 10, ST7735_BLACK);
  tft.writeFillRect(84, 50, 128, 10, ST7735_BLACK);
  tft.writeFillRect(72, 60, 128, 10, ST7735_BLACK);
  tft.writeFillRect(72, 70, 128, 10, ST7735_BLACK);
  tft.endWrite();

  tft.setCursor(0, 20);
  tft.println(String() + "Wind Speed:" + air_speed + "m/s");

  tft.setCursor(0, 30);
  tft.println(String() + "Temperature:" + Temperature + "*C");

  tft.setCursor(0, 40);
  tft.println(String() + "Humidity:" + Humidity + "%R");

  tft.setCursor(0, 50);
  tft.println(String() + "Soil Humidity:" + Soil_Humidity + "%R");

  tft.setCursor(0, 60);
  tft.println(String() + "Illuminance:" + light_intensity + " Lx");

  tft.setCursor(0, 70);
  tft.println(String() + "Air quality:" + air_quality + "ppm");

  Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_TIME;
}

void Display_QR(void)
{
  tft.drawRGBBitmap(0, 0, qr_image, 128, 128);
  Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_QR;
}

void Display_Function(void)
{
  tft.drawBitmap(24, 0, gImage_set, 80, 80, ST77XX_MAGENTA, ST7735_BLACK);
  Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_FUNCTION;
  tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.setCursor(20, 108);
  tft.println("Function");
}

void Display_Set_Short_Key(void)
{
  if (Set_key_mun < 5) {
    Set_key_mun ++;
    Display_Set();
    Screen_Display_Struct.NO_SHORT_KEY_HANDLE = true;
  } else {
    Set_key_mun = 0;
    Screen_Display_Struct.NO_SHORT_KEY_HANDLE = false;
    Display_Function();
  }
  Serial.printf("Set_key_mun=%d\r\n", Set_key_mun);
  Serial.printf("Screen_Display_Struct.NO_SHORT_KEY_HANDLE=%d\r\n", Screen_Display_Struct.NO_SHORT_KEY_HANDLE);
}

void Display_Set_Long_Key(void)
{
  switch (Set_key_mun) {
    case 1:
      if (Switch.Led == true) {
        Switch.Led = false;
        send_one_data("switch_1", 0);
        Serial.println("led off");
        digitalWrite(led_relay_pin, LOW);
      } else {
        Switch.Led = true;
        send_one_data("switch_1", 1);
        Serial.println("led on");
        digitalWrite(led_relay_pin, HIGH);
      }
      break;
    case 2:
      if (Switch.Watering == true) {
        Switch.Watering = false;
        send_one_data("switch_3", 0);
        Serial.println("滴灌 off");
        digitalWrite(pump_relay_pin, LOW);
      } else {
        Switch.Watering = true;
        send_one_data("switch_3", 1);
        Serial.println("滴灌 on");
        digitalWrite(pump_relay_pin, HIGH);
      }
      break;
    case 3:
      if (Switch.Curtain == true) {
        Switch.Curtain = false;
        send_one_data("switch_4", 0);
        Serial.println("卷帘 off");
        servo1.write(0);;
      } else {
        Switch.Curtain = true;
        send_one_data("switch_4", 1);
        Serial.println("卷帘 on");
        servo1.write(180);;
      }
      break;
    case 4:
      if (Switch.Exhaust == true) {
        Switch.Exhaust = false;
        send_one_data("switch_2", 0);
        Serial.println("排风 off");
        digitalWrite(Exhaust_relay_pin, LOW);
      } else {
        Switch.Exhaust = true;
        send_one_data("switch_2", 1);
        Serial.println("排风 on");
        digitalWrite(Exhaust_relay_pin, HIGH);
      }
      break;
    case 5:
      if (Switch.Auto == true) {
        Switch.Auto = false;
        send_one_data("about", 0);
        Serial.println("自动 off");
      } else {
        Switch.Auto = true;
        send_one_data("about", 1);
        Serial.println("自动 on");
      }
      break;
    default:
      break;
  }
}

void Display_Set(void)
{
  Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_SET;

  tft.drawBitmap(0, 0, gImage_ld, 18, 18, ST77XX_CYAN, ST7735_BLACK);
  tft.drawBitmap(0, 24, gImage_qs, 18, 18, ST77XX_CYAN, ST7735_BLACK);
  tft.drawBitmap(0, 48, gImage_ql, 18, 18, ST77XX_CYAN, ST7735_BLACK);
  tft.drawBitmap(0, 72, gImage_pf, 18, 18, ST77XX_CYAN, ST7735_BLACK);
  tft.drawBitmap(0, 96, gImage_back, 18, 18, ST77XX_CYAN, ST7735_BLACK);

  if (Set_key_mun == 1) {

    tft.setTextColor(ST77XX_WHITE, ST7735_BLACK );
    tft.setTextSize(1);
    tft.setCursor(26, 8);
    tft.print("illumination:");
    tft.setTextColor(ST7735_BLACK, ST77XX_WHITE);
    if (Switch.Led) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  } else {

    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    tft.setCursor(26, 8);
    tft.print("illumination:");
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    if (Switch.Led) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  }

  if (Set_key_mun == 2) {
    tft.setTextColor(ST77XX_WHITE, ST7735_BLACK );
    tft.setTextSize(1);
    tft.setCursor(26, 32);
    tft.print("Pump:");
    tft.setTextColor(ST7735_BLACK, ST77XX_WHITE);
    if (Switch.Watering) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  } else {
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    tft.setCursor(26, 32);
    tft.print("Pump:");
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK );
    if (Switch.Watering) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  }

  if (Set_key_mun == 3) {
    tft.setTextColor(ST77XX_WHITE, ST7735_BLACK );
    tft.setTextSize(1);
    tft.setCursor(26, 56);
    tft.print("Curtain:");
    tft.setTextColor(ST7735_BLACK, ST77XX_WHITE);
    if (Switch.Curtain) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  } else {
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    tft.setCursor(26, 56);
    tft.print("Curtain:");
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    if (Switch.Curtain) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  }

  if (Set_key_mun == 4) {
    tft.setTextColor(ST77XX_WHITE, ST7735_BLACK );
    tft.setTextSize(1);
    tft.setCursor(26, 80);
    tft.print("Exhaust:");
    tft.setTextColor(ST7735_BLACK, ST77XX_WHITE);
    if (Switch.Exhaust) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  } else {
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    tft.setCursor(26, 80);
    tft.print("Exhaust:");
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    if (Switch.Exhaust) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  }

  if (Set_key_mun == 5) {
    tft.setTextColor(ST77XX_WHITE, ST7735_BLACK );
    tft.setTextSize(1);
    tft.setCursor(26, 104);
    tft.print("Automatic:");
    tft.setTextColor(ST7735_BLACK, ST77XX_WHITE);
    if (Switch.Auto) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  } else {
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    tft.setCursor(26, 104);
    tft.print("Automatic:");
    tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
    if (Switch.Auto) {
      tft.println(" ON");
    } else {
      tft.println("OFF");
    }
  }
}

void Display_Info(void)
{
  Screen_Display_Struct.SCREEN_DISPLAY_STATUS = DISPLAY_INFO;

  tft.setTextColor( ST77XX_WHITE, ST7735_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Device:ESP32");
  tft.setCursor(0, 10);
  tft.println("Translater:Arduino 1.8.9");
  tft.setCursor(0, 30);
  tft.println("Server:OneNet");
  tft.setCursor(0, 40);
  tft.println("Author:LuoWang");
  tft.setCursor(0, 50);
  tft.println("version:19.6.7");
  tft.setCursor(0, 60);
  tft.println("Field:Smart agriculture");
  tft.setCursor(0, 80);
  tft.println("Use:Greenhouse Environmental Monitoring.");
}
