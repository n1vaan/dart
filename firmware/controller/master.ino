#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// MASTER: ESP32 DEVKIT V1
 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int pinX1 = 34;
int pinY1 = 35;
int pinX2 = 33;
int pinY2 = 32;

int UPDATESPEED = 100;

typedef struct struct_message {
  int x1;
  int y1;
  int x2;
  int y2;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  
  esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE);
  
  delay(100);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 2; 
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("Master setup complete");
}

void loop() {


  myData.x1 = analogRead(pinX1);
  myData.y1 = analogRead(pinY1);
  myData.x2 = analogRead(pinX2);
  myData.y2 = analogRead(pinY2);

  Serial.print("X1: ");
  Serial.println(myData.x1);
  Serial.print("Y1: ");
  Serial.println(myData.y1);
  Serial.print("X2: ");
  Serial.println(myData.x2);
  Serial.print("Y2: ");
  Serial.println(myData.y2);

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(UPDATESPEED);
}
