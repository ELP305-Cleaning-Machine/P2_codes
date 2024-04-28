#include <Adafruit_MPU6050.h>  // Include MPU6050 library
#include <Wire.h>              // Include Wire library for I2C(Inter-Integrated Circuit) communication
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>




Adafruit_MPU6050 mpu;         // Create an instance of the MPU6050 sensor
esp_now_peer_info_t peerInfo_1;
esp_now_peer_info_t peerInfo_2;
uint8_t broadcastAddress[2][6] = {{0xEC, 0x64, 0xC9, 0x86, 0x13, 0xC0},{0xEC, 0x64, 0xC9, 0x82, 0x7E, 0x10}};
String ssid[2] = {"BUS_STOP_A","BUS_STOP_B"};
int vertex = 0;
const int buttonPin = 0;
int nOOS = 0;
int hope = 0;
volatile bool buttonPressed = false; // Flag to indicate if the button is pressed
bool passenger = false;


int halt = 0;
int latch = 0;

typedef struct struct_message {
    String a;
    int nBusStop;
    int nBusIndex;
    int nDirection;

} struct_message;

// Create a struct_message called myData
struct_message myData;

///////////////////////////////////////////
const int ledPin = 2; // LED pin

// Structure to receive data
typedef struct {
  bool buttonPressed;
} Message;

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  Message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.println("passengers waiting");

  passenger=true;
}


////////////////////////////////////////////

void send_data_service(int i){
  if(i==0){
    myData.a="BUS OOS";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[0], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    result = esp_now_send(broadcastAddress[1], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  else{
    myData.a="BUS NOOS";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[0], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    result = esp_now_send(broadcastAddress[1], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

}

void IRAM_ATTR handleInterrupt() {
   buttonPressed = !buttonPressed;
 }


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status == ESP_NOW_SEND_SUCCESS){
  }
  else{
  }
}


void setup(void) {
  Serial.begin(115200);       // Initialize serial communication
  while (!Serial)
    delay(10);                // Wait for serial port to connect

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleInterrupt, FALLING);
  Serial.println("Adafruit MPU6050 test!");  // Print a message to serial monitor

  
  if (!mpu.begin()) {                            // Try to initialize MPU6050 sensor
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");  // Print a message indicating MPU6050 is found

  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);                  // Set accelerometer range to +/- 8G
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
  }
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);                      // Set filter bandwidth to 21 Hz
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
  }

  Serial.println("");                       // Print empty line for readability
  delay(100);                               // delay for stabilization

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("BUS",NULL,4);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo_1.peer_addr, broadcastAddress[0], 6);
  peerInfo_1.channel = 4;  
  peerInfo_1.encrypt = false;
   if (esp_now_add_peer(&peerInfo_1) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo_2.peer_addr, broadcastAddress[1], 6);
  peerInfo_2.channel = 4;  
  peerInfo_2.encrypt = false;
   if (esp_now_add_peer(&peerInfo_2) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  /////////////////////////////////////////////////////
  esp_now_register_recv_cb(onDataRecv);

  pinMode(ledPin, OUTPUT); // Set LED pin as output
  digitalWrite(ledPin, LOW); // Start with LED off
  /////////////////////////////////////////////////////



}

int get_rssi(int j){
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks(false, false, false, 50, 4);
  Serial.println("scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      
      
      if(WiFi.SSID(i)==ssid[j]){
        Serial.println(WiFi.RSSI(i));
        if(WiFi.RSSI(i)>=-50){
          //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
          return(0);
        }
        return(2);
      }
    }
    return(1);
  }
  Serial.println("");
}

int avg_sample = 0;
int arr[10];

void send_data(int i, int v){
  if(i==0){
    myData.a="BUS STOPPED!";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[v], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  else{
    myData.a="BUS STARTED!";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[v], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  }

void send_data_halt(int i, int v){
  if(i==0){
    myData.a="BUS HALTED";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[v], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

  }
  else{
    myData.a="BUS MOVING";
    myData.nBusIndex=1;
    myData.nBusStop=0;
    myData.nDirection=0;
    esp_err_t result = esp_now_send(broadcastAddress[v], (uint8_t *) &myData, sizeof(myData));
      
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

  }
}

void loop() {
    /* Get new sensor events with the readings */

  if(buttonPressed){
    if(hope==0){
    Serial.println("BUS OUT OF SERVICE!");
    send_data_service(0);
    nOOS = 1;
    }
    hope = 1;
    /*while(buttonPressed){
      if(!buttonPressed){
        Serial.println("CONTINUING SERVICE");
        send_data_service(1);
      }
    }*/
  }
  if(nOOS==1 && !buttonPressed){
    Serial.println("CONTINUING SERVICE");
    send_data_service(1);
    nOOS=0;
    hope = 0;
  }

  if(passenger){
    Serial.println("Received signal, turning on LED");
    for(int i =0; i<2; i++){
      digitalWrite(ledPin, HIGH); // Turn on LED
      delay(250);
      digitalWrite(ledPin, LOW);
      delay(250);
  }
  passenger = false;
  }

  
  if(avg_sample==10){
    int s =0;
    for(int j =0; j<10;j++){
      s = s+arr[j];
    }
    avg_sample = 0;
    if(s>=7){
      Serial.println("Bus Moving");
      if(latch==1){
        if(halt == 0){
          send_data(1,vertex%2);
          latch = 0;
          vertex++;
        }
        else{
          send_data_halt(1,vertex%2);
          send_data_halt(1,(vertex+1)%2);
          latch = 0;
          halt = 0;
        }
      }
      
    }
    else{
      Serial.println("Bus stopped");
      int t = get_rssi(vertex%2);
      int x = get_rssi((vertex+1)%2);
      if(t==0 && latch==0){
        Serial.println("bus arrived at");
        Serial.println(ssid[vertex%2]);
        send_data(0,vertex%2);
        latch =1;
        /*esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
        if (result == ESP_OK) {
        Serial.println("Sent with success");
        }
        else {
        Serial.println("Error sending the data");
        }
        */
      }
      else if((t==2 || x==2)&& latch==0){
        send_data_halt(0,vertex%2);
        send_data_halt(0,(vertex+1)%2);
        latch =1;
        halt = 1;
      }
      else{
        //Do nothin
      }
    }
    }
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int flag [5];                           // Array to store motion detection flags
  double net;
  // Loop to check motion for 10 iterations
  for(int i =0; i<5; i++){
    double a_x = a.acceleration.x;        // Acceleration along x-axis
    double a_y = a.acceleration.y;        // Acceleration along y-axis
    double a_z = a.acceleration.z;        // Acceleration along z-axis
     net = sqrt((a_x*a_x)+(a_y*a_y)+(a_z*a_z));                 // Calculate net acceleration

    // Check if net acceleration falls within a certain range from the offset (acceleration when bus is stopped)
    if (net>9.4 and net<10.3) {
      //Serial.println("bs");              // Print motion status as "bs=Bus stopped"
      //Serial.print(a.acceleration.x);
      flag[i] = 0;                       // Set flag to 0 indicating bus stopped
    }
    else {
      //Serial.println("bm");              // Print motion status as "bm=Bus moving"
      //Serial.print(a.acceleration.x);               
      flag[i] = 1;                       // Set flag to 1 indicating bus moving 
    }
    delay(100);                          // Delay betrween iterations for stability
  }
  int sum = 0;
  // Calculate sum of motion detection flags
  for(int i = 0; i<5; i++){
    sum = sum + flag[i];
  }
  // Check if sum indicates bus stopped or moving
  if(sum==0){
    Serial.println("BS");            // Print message indicating bus stopped
    //Serial.print(net);
    arr[avg_sample]=0;
  }
  else {
    Serial.println("BM");    // Print message indicating bus is moving
    //Serial.print(net);
    arr[avg_sample]=1;
  }
  avg_sample = avg_sample +1;
  Serial.println("");                   // Print empty line for readability
  
}