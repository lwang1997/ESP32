/* *****************************************************************

   Download latest Blinker library here:
   https://github.com/blinker-iot/blinker-library/archive/master.zip


   Blinker is a cross-hardware, cross-platform solution for the IoT.
   It provides APP, device and server support,
   and uses public cloud services for data transmission and storage.
   It can be used in smart home, data monitoring and other fields
   to help users build Internet of Things projects better and faster.

   Make sure installed 2.5.0 or later ESP8266/Arduino package,
   if use ESP8266 with Blinker.
   https://github.com/esp8266/Arduino/releases

   Make sure installed 1.0.2 or later ESP32/Arduino package,
   if use ESP32 with Blinker.
   https://github.com/espressif/arduino-esp32/releases

   Docs: https://doc.blinker.app/
         https://github.com/blinker-iot/blinker-doc/wiki

 * *****************************************************************

   Blinker 库下载地址:
   https://github.com/blinker-iot/blinker-library/archive/master.zip

   Blinker 是一套跨硬件、跨平台的物联网解决方案，提供APP端、设备端、
   服务器端支持，使用公有云服务进行数据传输存储。可用于智能家居、
   数据监测等领域，可以帮助用户更好更快地搭建物联网项目。

   如果使用 ESP8266 接入 Blinker,
   请确保安装了 2.5.0 或更新的 ESP8266/Arduino 支持包。
   https://github.com/esp8266/Arduino/releases

   如果使用 ESP32 接入 Blinker,
   请确保安装了 1.0.2 或更新的 ESP32/Arduino 支持包。
   https://github.com/espressif/arduino-esp32/releases

   文档: https://doc.blinker.app/
         https://github.com/blinker-iot/blinker-doc/wiki

 * *****************************************************************/

#define BLINKER_WIFI
#define BLINKER_MIOT_OUTLET

#include <Blinker.h>

char auth[] = "Your Device Secret Key";

bool oState = false;

#define LED_STATUS  2 //状态灯

void miotPowerState(const String & state)
{
  BLINKER_LOG("need set power state: ", state);

  if (state == BLINKER_CMD_ON) {
    digitalWrite(LED_STATUS, HIGH);

    BlinkerMIOT.powerState("on");
    BlinkerMIOT.print();

    oState = true;
  }
  else if (state == BLINKER_CMD_OFF) {
    digitalWrite(LED_STATUS, LOW);

    BlinkerMIOT.powerState("off");
    BlinkerMIOT.print();

    oState = false;
  }
}

void miotQuery(int32_t queryCode)
{
  BLINKER_LOG("MIOT Query codes: ", queryCode);

  switch (queryCode)
  {
    case BLINKER_CMD_QUERY_ALL_NUMBER :
      BLINKER_LOG("MIOT Query All");
      BlinkerMIOT.powerState(oState ? "on" : "off");
      BlinkerMIOT.print();
      break;
    case BLINKER_CMD_QUERY_POWERSTATE_NUMBER :
      BLINKER_LOG("MIOT Query Power State");
      BlinkerMIOT.powerState(oState ? "on" : "off");
      BlinkerMIOT.print();
      break;
    default :
      BlinkerMIOT.powerState(oState ? "on" : "off");
      BlinkerMIOT.print();
      break;
  }
}

void dataRead(const String & data)
{
  BLINKER_LOG("Blinker readString: ", data);

  Blinker.vibrate();

  uint32_t BlinkerTime = millis();

  Blinker.print("millis", BlinkerTime);
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
      WiFi.setAutoConnect(false);  // 设置自动连接
      break;
    }
    digitalWrite(LED_STATUS, LOW);
    delay(1000);
    digitalWrite(LED_STATUS, HIGH);
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
      digitalWrite(LED_STATUS, LOW);
      delay(1000);
      digitalWrite(LED_STATUS, HIGH);
    }
  }
  Serial.println("WIFI AutoConfig Faild!" );
  return false;
}

void setup()
{
  Serial.begin(115200);
  BLINKER_DEBUG.stream(Serial);

  pinMode(LED_STATUS, OUTPUT);

  if (!AutoConfig())
  {
    SmartConfig();
  }

  digitalWrite(LED_STATUS, LOW);

  Blinker.begin(auth, WiFi.SSID().c_str(), WiFi.psk().c_str());
  Blinker.attachData(dataRead);

  BlinkerMIOT.attachPowerState(miotPowerState);
  BlinkerMIOT.attachQuery(miotQuery);
}

void loop()
{
  Blinker.run();
}
