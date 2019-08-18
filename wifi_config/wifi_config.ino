#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiMulti.h>

const char* AP_SSID  = "ESP32_Config"; //热点名称
const char* AP_PASS  = "12345678";  //密码
#define ROOT_HTML  "<!DOCTYPE html><html><head><title>WIFI Config by lwang</title><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head><style type=\"text/css\">.input{display: block; margin-top: 10px;}.input span{width: 100px; float: left; float: left; height: 36px; line-height: 36px;}.input input{height: 30px;width: 200px;}.btn{width: 120px; height: 35px; background-color: #000000; border:0px; color:#ffffff; margin-top:15px; margin-left:100px;}</style><body><form method=\"GET\" action=\"connect\"><label class=\"input\"><span>WiFi SSID</span><input type=\"text\" name=\"ssid\"></label><label class=\"input\"><span>WiFi PASS</span><input type=\"text\"  name=\"pass\"></label><input class=\"btn\" type=\"submit\" name=\"submit\" value=\"Submie\"></form></body></html>"
WebServer server(80);
WiFiMulti wifiMulti;

uint8_t resr_count_down = 120;//重启倒计时s
TimerHandle_t xTimer_rest;
void restCallback(TimerHandle_t xTimer );

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);//配置为AP模式
  boolean result = WiFi.softAP(AP_SSID, AP_PASS);//开启WIFI热点
  if (result)
  {
    IPAddress myIP = WiFi.softAPIP();

    //打印相关信息
    Serial.println("");
    Serial.print("Soft-AP IP address = ");
    Serial.println(myIP);
    Serial.println(String("MAC address = ")  + WiFi.softAPmacAddress().c_str());
    Serial.println("waiting ...");

    xTimer_rest = xTimerCreate("xTimer_rest", 1000 / portTICK_PERIOD_MS, pdTRUE, ( void * ) 0, restCallback);
    xTimerStart( xTimer_rest, 0 );  //开启定时器

  } else {  //开启热点失败
    Serial.println("WiFiAP Failed");
    delay(3000);
    ESP.restart();  //复位esp32
  }

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
    //获取输入的WIFI账户和密码
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    Serial.println("WiFi Connect SSID:" + ssid + "  PASS:" + pass);
    //设置为STA模式并连接WIFI
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());

    resr_count_down = 120;
    xTimerStop(xTimer_rest, 0);

    uint8_t Connect_time = 0; //用于连接计时，如果长时间连接不成功，复位设备

    while (WiFi.status() != WL_CONNECTED) {  //等待WIFI连接成功
      delay(500);
      Serial.print(".");
      Connect_time ++;
      if (Connect_time > 80) {  //长时间连接不上，复位设备
        Serial.println("Connection timeout, check input is correct or try again later!");
        delay(3000);
        ESP.restart();
      }
    }
  });
  server.begin();
}

void loop() {
  server.handleClient();
  while (WiFi.status() == WL_CONNECTED) {
    //WIFI已连接
  }
}

void restCallback(TimerHandle_t xTimer ) {  //长时间不访问WIFI Config 将复位设备
  resr_count_down --;
  Serial.print("resr_count_down: ");
  Serial.println(resr_count_down);
  if (resr_count_down < 1) {
    ESP.restart();
  }
}
