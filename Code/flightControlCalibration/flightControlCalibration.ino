//Include statements
#include <esp_now.h>//For ESP-NOW protocol
#include <WiFi.h>//For Wifi Communication
#include <Wire.h>//I2C library
#include <ESP32Servo.h> 
//Sensor libraries
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <MPL3115A2.h>
//Define I2C pins
#define I2C_SDA 21
#define I2C_SCL 22
//Define Altitude Basis; Set for Topographic Map Altitude for Emmanuel Hill
#define ALTBASIS 407
#define OVERSAMPLE_RATE 4
//Sensor Assignment
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
MPL3115A2 baro3115;
Servo recovery;
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
void doCalibration()
{
  //calculate pressure for current ALTBASIS altitude by averaging a few readings
  Serial.println("Starting pressure calibration...");

  // this function calculates a new sea level pressure ref value
  // I adde this function to the original library and it will NOT change the sensor registers
  // see below setBarometricInput() where that value is actually set
  // in the registers. the sensor will start using it just after.
  baro3115.runCalibration(ALTBASIS);

  Serial.print("calculated sea level pressure: ");
  Serial.print(baro3115.calculated_sea_level_press, 2);
  Serial.println(" Pa");

  Serial.print("calculated elevation_offset: ");
  Serial.print(baro3115.elevation_offset, 2);
  Serial.println(" Pa");

  // i originally had big errors in pressure and altitude readouts,
  // once i added the elevation_offset calculation the error
  // decreased and is now close to that of another barometer
  // (whose manual calibration for altitude should be checked anyway)

  // I initally implemented this code without using the calls to setModeStandby() and setModeActive()
  // the results were polluted by occasional weird behavior with exceedingly large and wrong values.
  // Also, the temperature measurements were constantly increasing, probably due to the 
  // lower oversample rate and faster reading loop
  // I decided to compare the various libraries on github for this sensor and found out that
  // both Sparkfun's and Adafruit's versions did NOT use a specific calls sequence when setting
  // mode and registers !
  // Michael Lange's sequence instead, is the one that finally gave me correct, smooth and repeatable measurement sessions. See below:
  
  baro3115.setModeStandby();    // <-- this one starts a config sequence
  baro3115.setModeBarometer();
  baro3115.setBarometricInput(baro3115.calculated_sea_level_press);
  baro3115.setOversampleRate(0);
  baro3115.enableEventFlags();
  baro3115.setModeActive();   // <-- this one ends the sequence and starts the measurement mode

  // calibration is now completed
  //
  // setBarometricInput() :
  // This configuration option calibrates the sensor according to
  // the sea level pressure for the measurement location (2 Pa per LSB)
  // The default value for "Barometric input for Altitude calculation" is 101,326 Pa

  // About the oversample rate:
  // Set the # of samples from 1 to 128. See datasheet.
  // Integer values between 0 < n < 7 give oversample ratios 2^n and 
  // sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
  // Seems that the suggested value is 7 and the measurement could take 512ms
  // I'm using it with good results, but I'm still trying to understand why the time taken
  // for the pressure reading is just 3ms (measured with the millis() function)
  
  // add temperature offset for my tested sensor
  // seems the temperature probe is within the ADC and should not be used
  // to measure the environment. Will try adding the offset i got by comparison
  // with another thermometer
  // you can enable this if you need it:
  // baro3115.setModeStandby();
  // baro3115.setOffsetTemperature((char) (0.65 / 0.0625) );
  // baro3115.setModeActive();

  Serial.println("Pressure calibration completed.");

  // let's have a look, just in case:
  //Serial.println("OFFSETS:");
  //Serial.print("  pressure: ");
  //Serial.println(baro3115.offsetPressure() ,2);
  //Serial.print("  altitude: ");
  //Serial.println((float)baro3115.offsetAltitude() ,2);
  //Serial.print("  temperature(C): ");
  //Serial.println(baro3115.offsetTemperature() ,2);

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
  //if (!baro.begin()) {
   // Serial.println("Could not find sensor. Check wiring.");
    //while(1);
  recovery.attach(16);

  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  //baro.setSeaPressure(1013.26);
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
  recovery.write(90);//Set pin 13 high to allow for recovery measures to be deployed
  Serial.println("Deploying");
}
else
{
  recovery.write(180);//Set pin 13 low to turn off LED
}
baro3115.setModeStandby();
baro3115.setModeBarometer();
baro3115.setOversampleRate(OVERSAMPLE_RATE);
baro3115.enableEventFlags();
baro3115.setModeActive();
// when we are using the calibration then we also have to add the
// calculated elevation related pressure offset to our readings:
pressure = baro3115.readPressure() + baro3115.elevation_offset;

// output is in Pa
// 1 kPa = 10 hPa = 1000 Pa
// 1 hPa = 100 Pascals = 1 mb
pressure = (pressure / 100) ;   //  ... / 1000 * 10 ;
Serial.print("Pressure(hPa): ");
Serial.print(pressure, 2);

float temperature = baro3115.readTemp();
Serial.print(" Temperature(C): ");
Serial.print(temperature, 2);

baro3115.setModeStandby();
baro3115.setModeAltimeter();
baro3115.setOversampleRate(OVERSAMPLE_RATE);
baro3115.enableEventFlags();
baro3115.setModeActive();
altitude = baro3115.readAltitude();
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
