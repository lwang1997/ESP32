#include <WiFi.h>
#include <esp_wifi.h>

#define RESET_PIN   13  //用于删除WiFi信息

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(RESET_PIN, INPUT_PULLUP);

  delay(100);

  if (!AutoConfig())
  {
    SmartConfig();
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(RESET_PIN) == LOW) {
    esp_wifi_restore();
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
