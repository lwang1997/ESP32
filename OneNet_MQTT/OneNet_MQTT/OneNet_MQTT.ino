#include<WiFi.h>
#include<PubSubClient.h>
#include <ArduinoJson.h>

// Update these with values suitable for your network.

const char* user_ssid = "*****";
const char* user_password = "*****";

const char* mqtt_server = "*****"; //服务器IP(固定的)
const char* Product_PID = "*****"; //产品ID
const char* Device_ID = "*****"; //设备ID
const char* APIKey = "*****"; //APIKey

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
uint8_t msg_buf[200];
char msgJson[200];
char debug_buf[200];
unsigned short json_len = 0;
uint8_t* packet_p;
uint8_t debug_buffer_start_index = 0;
void send_one_data_onenet(char* id, float temp);

//服务器指令回调
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[0] == '1') {

  } else {

  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(Device_ID, Product_PID, APIKey)) { //One net user name as product ID , and password as APIKey
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
      client.connect(Device_ID, Product_PID, APIKey);
      client.setCallback(callback);

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void send_one_data_onenet(char *id, float temp) {
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(user_ssid);

  WiFi.begin(user_ssid, user_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 6002);  //not 1883 , one net use the port of 6002 as mqtt server
  client.connect(Device_ID, Product_PID, APIKey);
  client.setCallback(callback);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg > 2000) { //2S
    lastMsg = now;
    send_one_data_onenet("test", random(20)); //发送测试数据
  }
}
