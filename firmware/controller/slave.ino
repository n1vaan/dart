#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// SLAVE: XIAO ESP32-S3

int oldx1; 
int oldy1;
int oldx2; 
int oldy2; 

int cx1; 
int cy1;
int cx2; 
int cy2; 

int dx1;
int dy1;
int dx2;
int dy2;

typedef struct struct_message {
  int x1;
  int y1;
  int x2;
  int y2;
} struct_message;

struct_message myData;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  oldx1 = cx1;
  oldy1 = cy1;
  oldy2 = cx2; 
  oldx2 = cy2;

  cx1 = myData.x1;
  cy1 = myData.y1;
  cx2 = myData.x2; 
  cy2 = myData.y2; 

  Serial.print("X1: ");
  Serial.print(cx1);
  Serial.print(" | Y1: ");
  Serial.print(cy1);
  Serial.print(" | X2: ");
  Serial.print(cx2);
  Serial.print(" | Y2: ");
  Serial.println(cy2);

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
    
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("Slave setup complete");
}

void loop() {
  
}
