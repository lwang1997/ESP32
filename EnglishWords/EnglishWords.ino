#include <WiFi.h>
#include <HTTPClient.h>
//url=https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_GFX.h>    // Core graphics library
//url=https://github.com/ananevilya/Arduino-ST7789-Library
#include <Arduino_ST7789.h>  // Hardware-specific library for ST7789 (with or without CS pin)
#include <SPI.h>

#include "cn_font.h"
#include "gb2unicode.h"

#define reacquire   0

// ST7789 TFT module connections
#define TFT_RST         2 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC          4
#define TFT_MOSI        23  // Data out
#define TFT_SCLK        18  // Clock out

// Initialize Adafruit ST7789 TFT library
Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK);

char str_buff[512];

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  // if the display has CS pin try with SPI_MODE0
  tft.init(240, 240);    // Init ST7789 display 240x240 pixel
  // if the screen is flipped, remove this command
  tft.setRotation(2);
  Serial.println(F("Initialized"));

  delay(100);
  tft.fillScreen(BLACK);
  DisplayGbString(0, 0, "正在连接WiFi，请稍后...", WHITE, BLACK);
  if (!AutoConfig())
  {
    tft.fillScreen(BLACK);
    DisplayGbString(0, 0, "WiFi连接失败，请重新配网...", WHITE, BLACK);
    SmartConfig();
  }

  get_en_words();
  tft.fillScreen(BLACK);
  DisplayGbString(0, 0, str_buff, WHITE, BLACK);

  pinMode(reacquire, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(reacquire) == LOW) {
    delay(10);
    if (digitalRead(reacquire) == LOW) {
      while (!digitalRead(reacquire));
      get_en_words();
      tft.fillScreen(BLACK);
      DisplayGbString(0, 0, str_buff, WHITE, BLACK);
    }
  }

}

void SmartConfig()
{
  WiFi.mode(WIFI_AP_STA);
  Serial.println("WIFI Wait for Smartconfig");
  WiFi.beginSmartConfig();
  while (1)
  {
    Serial.print(".");
    if (WiFi.smartConfigDone())
    {
      Serial.println("WIFI SmartConfig Success");
      Serial.printf("SSID:%s", WiFi.SSID().c_str());
      Serial.printf(", PSW:%s\r\n", WiFi.psk().c_str());
      Serial.print("LocalIP:");
      Serial.print(WiFi.localIP());
      Serial.print(" ,GateIP:");
      Serial.println(WiFi.gatewayIP());
      WiFi.setAutoConnect(true);  // 设置自动连接
      break;
    }
    delay(1000);
  }
}

bool AutoConfig()
{
  WiFi.begin();
  for (int i = 0; i < 20; i++)
  {
    int wstatus = WiFi.status();
    if (wstatus == WL_CONNECTED)
    {
      Serial.println("WIFI SmartConfig Success");
      Serial.printf("SSID:%s", WiFi.SSID().c_str());
      Serial.printf(", PSW:%s\r\n", WiFi.psk().c_str());
      Serial.print("LocalIP:");
      Serial.print(WiFi.localIP());
      Serial.print(" ,GateIP:");
      Serial.println(WiFi.gatewayIP());
      return true;
    }
    else
    {
      Serial.print("WIFI AutoConfig Waiting......");
      Serial.println(wstatus);
      delay(1000);
    }
  }
  Serial.println("WIFI AutoConfig Faild!" );
  return false;
}

void DisplayGbString(int16_t x, int16_t y, char* utf8str, uint16_t color, uint16_t bg)
{
  char gbBuff[2048];
  uint16_t gb_len = 0;
  gb_len = utf8StrToGB2312(gbBuff, utf8str, strlen(utf8str));
  uint8_t char_len = 0;
  uint16_t charnum = 0;
  uint8_t display_buff[72];
  for (int i = 0; i < gb_len ;) {
    memset(display_buff, 0, 72);
    char_len = get_cn_font_one( &gbBuff[i], display_buff);
    i += char_len;
    if (charnum > LCD_Wight - 24)
    {
      charnum = 0;
      y += 24;
      if (y > LCD_High - 24) break; //y=0;
    }
    if (char_len == 2) {
      tft.drawBitmap(x + charnum, y, display_buff, 24, 24, color, bg);
      charnum += 24;
    } else if (char_len == 1) {
      tft.drawBitmap(x + charnum, y, display_buff, 12, 24, color, bg);
      charnum += 12;
    }
  }
}

void get_en_words(void)
{
  HTTPClient http;
  http.begin("http://lwang.xyz/hitokoto/");//Specify the URL
  int httpCode = http.GET();//Make the request
  if (httpCode > 0) {
    String payload = http.getString();
    memset(str_buff, 0, sizeof(str_buff));
    strcpy(str_buff, payload.c_str());
    Serial.println(httpCode);
    Serial.println(payload);
  }
  http.end();
}
