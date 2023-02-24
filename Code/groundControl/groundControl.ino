//Include statements
#include <esp_now.h>//For ESP-NOW protocol
#include <WiFi.h>//For Wifi Communication
#include <Wire.h>//I2C
//#include "SPIFFS.h"
//Sensor libraries
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
#include SD
//Define I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//Declaration of Variables
const int WAIT_TIME = 50;//Wait time between polling
//Variables to hold data from the telemetry transmitter
float pressure;
float altitude;
float acceleration;
float timeFromLaunch=0;
bool deployRec;//Create variable to see if recovery measure should be deployed based on if pin 16 is high
bool resetToZero;//Boolean to check to set launch altitude to zero.

//Creation of structure for reception of data
typedef struct struct_message {
    float pres;
    float alt;
    float acc;
} struct_message;
// Create a struct_message called telem to hold sensor readings
struct_message telem;

//ESP-NOW configuration
esp_now_peer_info_t peerInfo;
//Address of other board
uint8_t broadcastAddress[] ={0xAC,0x67,0xB2,0xCC,0x44,0x88};

//Functions to check data transmission success
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//Function to handle data reception
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&telem, incomingData, sizeof(telem));
  Serial.print("Bytes received: ");
  Serial.println(len);
  pressure = telem.pres;
  altitude = telem.alt;
  acceleration = telem.acc;
}

//Setup function
void setup() {
//Creat CSV file
File dataLog = SD.open("/data.csv", "w");
if(!dataLog)
{
  // File not found
  Serial.println("Failed to open test file");
  return;
}
 else 
 {
  dataLog.println("Time,Altitude,Acceleration,Pressure");
  dataLog.close();
  }
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
    esp_now_register_recv_cb(OnDataRecv);
}

//Main loop
void loop()
{
  //File dataLog = SPIFFS.open("/data.csv", "w");
  //Check if the switch to deploy recovery measures is toggled
  deployRec = digitalRead(16);
  resetToZero= digitalRead(17);
    // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &deployRec, sizeof(deployRec));//Send the value of deployRec
  if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
  else 
    {
      Serial.println("Error sending the data");
    }

  //Print incoming data to serial console
  Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("altitude = "); Serial.print(altitude*3.281); Serial.println(" ft");
  Serial.print(" \tVertical Acceleration: "); Serial.print(acceleration);Serial.println(" m/s^2 ");
  dataLog.println(timeFromLaunch);
  dataLog.print(",");dataLog.print(altitude);
  dataLog.print(",");dataLog.print(acceleration);
  dataLog.print(",");dataLog.print(pressure);
  dataLog.close();
  timeFromLaunch=+WAIT_TIME;
  delay(WAIT_TIME);//Send data after the defined waiting period
} 
