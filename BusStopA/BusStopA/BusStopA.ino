//Code for Bus stop A with communication with Bus stop B 

#include <esp_now.h>
#include <WiFi.h>

#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <SPI.h>
////////////////////
#define MAX_DEVICES   2
#define CLK_PIN       18    // or SCK
#define DATA_PIN      23    // or MOSI
#define CS_PIN        5    // or SS

// Change this to work with your matrix - see video 1 you have 4 choices
#define HARDWARE_TYPE MD_MAX72XX::PAROLA_HW

MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

int stop=1;
uint8_t scrollSpeed = 25;    // default frame delay value
textEffect_t scrollEffect = PA_SCROLL_LEFT;
textPosition_t scrollAlign = PA_LEFT;
uint16_t scrollPause = 200; // in milliseconds

#define  BUF_SIZE  75
char curMessage[BUF_SIZE] = { "" };
char newMessage[BUF_SIZE] = { "OUT OF SERVICE." };
bool newMessageAvailable = true;
char Buffer[3] = " ";  // create a buffer to hold the numbers

///////////////

uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0x82, 0x7E, 0x10};

int flag = 0;
unsigned long dwTime;
unsigned long dwTimeLast;
bool service = true;
int nHalt=0;
int nService=0;
//bool notservice = false;
int initial = 0;

int nStopID=1;     //unique ID of the operating stop
const int nBusNum=1;      //total number of buses
const int nStopNum=2; //total number of stops
int rgEstimated[nBusNum];   //store in seconds 
int rgScheduled[nStopNum][nStopNum] = {{100,50},{50,100}};   //store in seconds
// int rgDirection[nBusNum];  //Direction of each bus
esp_now_peer_info_t peerInfo;

bool halt = false;
//bool nothalt = true;
int nShowTime = 0;


// MAC address of the receiver (bus)
uint8_t busMAC[] = {0xE8, 0x6B, 0xEA, 0xD4, 0x27, 0x7C};

// Structure to send data
typedef struct {
  bool buttonPressed;
} Message;

volatile bool button = false; // Flag to indicate if the button is pressed

void IRAM_ATTR handleInterrupt() {
  button = true;
}
int buttonPin = 0;


//////////////////////////////////////

//void UpdateInterrupt(int nBusIndex,int nStopIndex);
//int  UpdateEstimated();
//void SendRequest(int *retarr);

void send_data(const uint8_t * peer_addr, const uint8_t * data, size_t len){
  esp_err_t result = esp_now_send(peer_addr, data, len);
    
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
      Serial.println("Error sending the data");
      }

}

void UpdateInterrupt(int nBusIndex,int nBusStop, int nStopID) {
  //On Interrupt update the scheduled times for the given bus
  rgEstimated[nBusIndex] = rgScheduled[nBusStop][nStopID];
}

int UpdateEstimated(){
  //Decrementing the times and taking minm
  for (int nBusID=0;nBusID<nBusNum;nBusID++){
    rgEstimated[nBusID]=max(0,rgEstimated[nBusID]-1);
  }
  int nMinm=rgEstimated[0];
  for (int nBusID=0;nBusID<nBusNum;nBusID++){
    nMinm=min(nMinm,rgEstimated[nBusID]);
  }
  return nMinm;
}

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    //float c;
    //bool d;
    String a ;
    //int id;
    int nBusStop;
    int nBusIndex;
    int nDirection;
    //int from_where;
    //int Stop_or_start;
} struct_message;

// Create a struct_message called myData
struct_message Send_Data;
struct_message Recieve_Data;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  initial = 1;
  memcpy(&Recieve_Data, incomingData, sizeof(Recieve_Data));
  Serial.println("received");
  Serial.println(Recieve_Data.a);
  if(Recieve_Data.nBusStop!=0){
    if(Recieve_Data.a=="BUS OOS"){
      service = false;
      nService=0;
    }
    else if(Recieve_Data.a=="BUS NOOS"){
      service = true;
    }
    else if(Recieve_Data.a =="BUS HALTED"){
      halt = true;
      nHalt=0;
    }
    else if(Recieve_Data.a =="BUS MOVING"){
      halt = false;
    }
    else{
      initial=1;
      Serial.println("Bus Departing from other stop");
      if(Recieve_Data.nDirection==0){
        UpdateInterrupt(Recieve_Data.nBusIndex-1,Recieve_Data.nBusStop-1,nStopID-1);
      }
      else{
        UpdateInterrupt(Recieve_Data.nBusIndex-1,nStopID-1,Recieve_Data.nBusStop-1);
      }
    }
  }
  if(Recieve_Data.nBusStop==0){
    if(Recieve_Data.a=="BUS STOPPED!"){
      stop=0;
      Serial.println("BUS ARRIVED!");
      //nothing
    }
    else if(Recieve_Data.a=="BUS STARTED!"){
      initial=1;
      Serial.println("BUS DEPARTING!");
      stop=1;
      Send_Data.a="BusStop A";
      Send_Data.nBusStop = nStopID;
      Send_Data.nBusIndex = Recieve_Data.nBusIndex;
      Send_Data.nDirection=Recieve_Data.nDirection;
      //Send_Data.from_where=0;
      UpdateInterrupt(Recieve_Data.nBusIndex-1,nStopID-1,nStopID-1);
      //flag = 1;
      delay(1000);
      send_data(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
      /*esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
    
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
      Serial.println("Error sending the data");
      }*/

    }

    else if(Recieve_Data.a =="BUS HALTED"){
      halt = true;
      nHalt=0;
      Serial.println("BUS HALTED");
      Send_Data.a="BUS HALTED";
      Send_Data.nBusStop = nStopID;
      Send_Data.nBusIndex = Recieve_Data.nBusIndex;
      Send_Data.nDirection=Recieve_Data.nDirection;
      send_data(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
    }
    else if(Recieve_Data.a =="BUS MOVING"){
      halt = false;
      Serial.println("BUS MOVING");
      Send_Data.a="BUS MOVING";
      Send_Data.nBusStop = nStopID;
      Send_Data.nBusIndex = Recieve_Data.nBusIndex;
      Send_Data.nDirection=Recieve_Data.nDirection;
      send_data(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
    }

    else if(Recieve_Data.a == "BUS OOS"){
      service = false;
      nService=0;
      Serial.println("BUS OOS");
      Send_Data.a="BUS OOS";
      Send_Data.nBusStop = nStopID;
      Send_Data.nBusIndex = Recieve_Data.nBusIndex;
      Send_Data.nDirection=Recieve_Data.nDirection;
      send_data(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));

     /* esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
    
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
      Serial.println("Error sending the data");
      }*/


    }
    else{
      service = true;
      Serial.println("BUS NOOS");
      Send_Data.a="BUS NOOS";
      Send_Data.nBusStop = nStopID;
      Send_Data.nBusIndex = Recieve_Data.nBusIndex;
      Send_Data.nDirection=Recieve_Data.nDirection;
      send_data(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));

      /*esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Send_Data, sizeof(Send_Data));
    
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
      Serial.println("Error sending the data");
      }*/


    }
    
    
    }

}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("BUS_STOP_A",NULL,4);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 4;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  ///////////////////////////////////////////////////////////////

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleInterrupt, FALLING);

   // Define and add the peer (the bus ESP32)
  esp_now_peer_info_t bus_peer;
  memset(&bus_peer, 0, sizeof(bus_peer)); // Clear peer structure
  memcpy(bus_peer.peer_addr, busMAC, 6); // Set peer MAC address
  bus_peer.channel = 4; // Use default channel
  bus_peer.encrypt = false; // No encryption
  if (esp_now_add_peer(&bus_peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


/////////////////////////////////////////////////////////////////

////
P.begin();
P.setIntensity(1);        // keep it 3 or below as we are powering off the chip/usb
P.displayText(Buffer, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
////


  dwTimeLast=millis();
}

void scroll(){
     
  if (P.displayAnimate()){
    P.displayText(newMessage, scrollAlign, scrollSpeed, scrollPause, scrollEffect, scrollEffect);
    P.displayReset();
  }
}
 
void loop() {
  if (button == true) { // If the boot button is pressed
    Message msg;
    msg.buttonPressed = true;
    esp_err_t result = esp_now_send(busMAC, (uint8_t*)&msg, sizeof(msg));
    
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
      Serial.println("Error sending the data");
      }
    delay(100); // Debounce delay
    button = false;
  }

//////////////////////////////////////////////////////////////////////////////////////
 
if(!halt){
  nShowTime=UpdateEstimated();
  Serial.println(nShowTime);
  if(P.displayAnimate()){
   int s = (nShowTime/60) +1;
  sprintf(Buffer, "%01d", nShowTime);  // the number "1" is the number of digits that you want always shown on the screen for this font we can have 5
     P.displayText(Buffer, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
  }
  
  delay(1000);
  }

  else{
    if(nHalt ==0){
      Serial.println("BUS HALTED IN BETWEEN");
      nHalt = 1;
    }
    if(!service){
      if(nService==0){
        Serial.println("BUS OUT OF SERVICE");
        if (P.displayAnimate()){
           P.displayText("OOS",PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
        
        P.displayReset();
        }
       
        nService = 1;
      }

    }
  }

///////////////////////////////////////////////////////////////////////

  /*if(halt ==true){
    Serial.println("BUS HALTED IN BETWEEN");
    while(!nothalt==true){
      if(service==false){
        Serial.println("OUT OF SERVICE!");
        while(notservice){
          //nothing
        }
        service = true;
        notservice=false;
      }

    }
    halt=false;
    nothalt=true;
  }
  nShowTime=UpdateEstimated();
  Serial.println(nShowTime);
  delay(1000);*/
 /* else{
  if(service==false){
    Serial.println("OUT OF SERVICE!");
    while(!service){
      //nothing
    }
    service = true;
  }
  else{
  
  if( nShowTime >1 || initial == 0){
  nShowTime=UpdateEstimated();
  Serial.println(nShowTime);
  delay(1000);
  }
  else{
    Serial.println(1);
    delay(1000);
  }
  }
  }*/
}
