//Include statements
#include <esp_now.h>//For ESP-NOW protocol
#include <WiFi.h>//For Wifi Communication
#include <Wire.h>//I2C library
//Sensor libraries
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
//Define I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

Adafruit_MPL3115A2 baro;

esp_now_peer_info_t peerInfo;
const int WAIT_TIME = 50;
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0xCC, 0x43, 0x74};

// Define variables to store BME280 readings to be sent
float pressure;
float altitude;
float acceleration;

// Define variables to store incoming readings
bool deployRec;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float pres;
    float alt;
    float acc;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message telem;

// Create a struct_message to hold incoming sensor readings

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&deployRec, incomingData, sizeof(deployRec));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Should recovery be deployed(0 for no; 1 for yes): ");
  Serial.println(deployRec);
}

 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  pinMode(16,OUTPUT);

  Serial.println("Adafruit_MPL3115A2 test!");

   Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);
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
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if (deployRec==1)
{
  digitalWrite(16, HIGH);//Set pin 13 high to allow for recovery measures to be deployed
  Serial.println("Deploying");
}
else
{
  digitalWrite(16,LOW);//Set pin 13 low to turn off LED
}
pressure = baro.getPressure();
altitude = baro.getAltitude();
//temperature = baro.getTemperature();
sensors_event_t event;
lis.getEvent(&event);
acceleration = event.acceleration.z;
telem.pres = pressure;
telem.alt = altitude;
telem.acc = acceleration;
esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &telem, sizeof(telem));
delay(WAIT_TIME);//Wait 1 ms to check again

}
