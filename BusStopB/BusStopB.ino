#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>

////////////////////
#define MAX_DEVICES 2 ///< The maximum number of devices.

#define CLK_PIN 18  ///< The pin for the clock signal.
#define DATA_PIN 23 ///< The pin for the data signal.
#define CS_PIN 5    ///< The pin for the chip select signal.

#define HARDWARE_TYPE                                      \
    MD_MAX72XX::PAROLA_HW ///< The type of hardware being
                          ///< used.

MD_Parola P = MD_Parola(
    HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN,
    MAX_DEVICES); ///< Instance of the MD_Parola class.
int stop = 0;     ///< Stop variable.

uint8_t scrollSpeed = 25; ///< The speed of the scroll.
textEffect_t scrollEffect =
    PA_SCROLL_LEFT; ///< The effect of the scroll.
textPosition_t scrollAlign =
    PA_LEFT; ///< The alignment of the scroll.
uint16_t scrollPause =
    200; ///< The pause time of the scroll in milliseconds.
#define BUF_SIZE 75 ///< The size of the buffer.

char curMessage[BUF_SIZE] = {""}; ///< The current message.
char newMessage[BUF_SIZE] = {
    "OUT OF SERVICE."}; ///< The new message.
bool newMessageAvailable =
    true; ///< Flag to check if a new message is available.
char Buffer[3] = " "; ///< Buffer to hold the numbers.

uint8_t broadcastAddress[] = {
    0xEC, 0x64, 0xC9,
    0x86, 0x13, 0xC0}; ///< The broadcast address.

int flag = 0;             ///< Flag variable.
unsigned long dwTime;     ///< Time variable.
unsigned long dwTimeLast; ///< Last time variable.
bool service = true;      ///< Service flag.
int nHalt = 0;            ///< Halt variable.
int nService = 0;         ///< Service variable.
int initial = 0;          ///< Initial variable.

int nStopID = 2;       ///< Unique ID of the operating stop.
const int nBusNum = 1; ///< Total number of buses.
const int nStopNum = 2;   ///< Total number of stops.
int rgEstimated[nBusNum]; ///< Array to store estimated
                          ///< times in seconds.
int rgScheduled[nStopNum][nStopNum] = {
    {100, 50}, {50, 100}}; ///< 2D array to store scheduled
                           ///< times in seconds.

esp_now_peer_info_t peerInfo; ///< Peer information.

bool halt = false; ///< Halt flag.
int nShowTime = 0; ///< Show time variable.

uint8_t busMAC[] = {
    0xE8, 0x6B, 0xEA, 0xD4,
    0x27, 0x7C}; ///< MAC address of the receiver (bus).

/**
 * @struct Message
 * @brief Structure to send data.
 *
 * @var Message::buttonPressed
 * @brief Button pressed flag.
 */
typedef struct
{
    bool buttonPressed;
} Message;
volatile bool button =
    false; ///< Flag to indicate if the button is pressed.

/**
 * @brief Interrupt service routine for handling button
 * press.
 *
 * This function is called when an interrupt is triggered.
 * It sets the button flag to true.
 *
 * @note This function is placed in IRAM, which is a region
 * of RAM that is directly accessible by the CPU.
 */
void IRAM_ATTR handleInterrupt()
{
    button = true;
}

int buttonPin = 0; ///< The pin connected to the button.
//////////////////////////////////////

// void UpdateInterrupt(int nBusIndex,int nStopIndex);
// int  UpdateEstimated();
// void SendRequest(int *retarr);

/**
 * @brief Sends data to a peer device over ESP-NOW.
 *
 * @param peer_addr The MAC address of the peer device.
 * @param data The data to be sent.
 * @param len The length of the data.
 *
 * This function sends data to a peer device over ESP-NOW.
 * It prints a success message to the serial port if the
 * data is sent successfully, otherwise it prints an error
 * message.
 */
void send_data(const uint8_t *peer_addr,
               const uint8_t *data, size_t len)
{
    esp_err_t result = esp_now_send(peer_addr, data, len);

    if (result == ESP_OK)
    {
        Serial.println("Sent with success");
    }
    else
    {
        Serial.println("Error sending the data");
    }
}

/**
 * @brief Updates the estimated time of arrival for a bus at
 * a stop.
 *
 * @param nBusIndex The index of the bus.
 * @param nBusStop The index of the bus stop.
 * @param nStopID The ID of the stop.
 *
 * This function updates the scheduled times for the given
 * bus when an interrupt occurs.
 */
void UpdateInterrupt(int nBusIndex, int nBusStop,
                     int nStopID)
{
    // On Interrupt update the scheduled times for the given
    // bus
    rgEstimated[nBusIndex] = rgScheduled[nBusStop][nStopID];
}

/**
 * @brief Updates the estimated time of arrival for all
 * buses and returns the minimum.
 *
 * @return The minimum estimated time of arrival.
 *
 * This function decrements the estimated times and takes
 * the minimum.
 */
int UpdateEstimated()
{
    // Decrementing the times and taking minm
    for (int nBusID = 0; nBusID < nBusNum; nBusID++)
    {
        rgEstimated[nBusID] =
            max(0, rgEstimated[nBusID] - 1);
    }
    int nMinm = rgEstimated[0];
    for (int nBusID = 0; nBusID < nBusNum; nBusID++)
    {
        nMinm = min(nMinm, rgEstimated[nBusID]);
    }
    return nMinm;
}

/**
 * @struct struct_message
 * @brief Structure for sending and receiving data.
 *
 * @var struct_message::a
 * @brief A string message.
 * @var struct_message::nBusStop
 * @brief The index of the bus stop.
 * @var struct_message::nBusIndex
 * @brief The index of the bus.
 * @var struct_message::nDirection
 * @brief The direction of the bus.
 *
 * This structure must match the sender structure.
 */
typedef struct struct_message
{
    String a;
    int nBusStop;
    int nBusIndex;
    int nDirection;
} struct_message;
// Create a struct_message called myData

struct_message Send_Data;    ///< Data to send.
struct_message Recieve_Data; ///< Data received.

// callback function that will be executed when data is
// received
/**
 * @brief Callback function that is executed when data is
 * received.
 *
 * This function handles the incoming data and performs
 * different actions based on the received message. It
 * updates the status of the bus (halted, moving, out of
 * service, etc.) and the estimated time of arrival.
 *
 * @param mac The MAC address of the sender.
 * @param incomingData The incoming data.
 * @param len The length of the incoming data.
 */
void OnDataRecv(const uint8_t *mac,
                const uint8_t *incomingData, int len)
{
    initial = 1;
    memcpy(&Recieve_Data, incomingData,
           sizeof(Recieve_Data));
    Serial.println("received");
    Serial.println(Recieve_Data.a);
    if (Recieve_Data.nBusStop != 0)
    {
        if (Recieve_Data.a == "BUS OOS")
        {
            service = false;
            nService = 0;
        }
        else if (Recieve_Data.a == "BUS NOOS")
        {
            service = true;
        }
        else if (Recieve_Data.a == "BUS HALTED")
        {
            halt = true;
            nHalt = 0;
        }
        else if (Recieve_Data.a == "BUS MOVING")
        {
            halt = false;
        }
        else
        {
            initial = 1;
            Serial.println("Bus Departing from other stop");
            if (Recieve_Data.nDirection == 0)
            {
                UpdateInterrupt(Recieve_Data.nBusIndex - 1,
                                Recieve_Data.nBusStop - 1,
                                nStopID - 1);
            }
            else
            {
                UpdateInterrupt(Recieve_Data.nBusIndex - 1,
                                nStopID - 1,
                                Recieve_Data.nBusStop - 1);
            }
        }
    }
    if (Recieve_Data.nBusStop == 0)
    {
        if (Recieve_Data.a == "BUS STOPPED!")
        {
            stop = 1;
            Serial.println("BUS ARRIVED!");
            // nothing
        }
        else if (Recieve_Data.a == "BUS STARTED!")
        {
            initial = 1;
            stop = 0;
            Serial.println("BUS DEPARTING!");
            Send_Data.a = "BusStop A";
            Send_Data.nBusStop = nStopID;
            Send_Data.nBusIndex = Recieve_Data.nBusIndex;
            Send_Data.nDirection = Recieve_Data.nDirection;
            // Send_Data.from_where=0;
            UpdateInterrupt(Recieve_Data.nBusIndex - 1,
                            nStopID - 1, nStopID - 1);
            // flag = 1;
            delay(1000);
            send_data(broadcastAddress,
                      (uint8_t *)&Send_Data,
                      sizeof(Send_Data));
            /*esp_err_t result =
            esp_now_send(broadcastAddress, (uint8_t *)
            &Send_Data, sizeof(Send_Data));

            if (result == ESP_OK) {
              Serial.println("Sent with success");
            }
            else {
            Serial.println("Error sending the data");
            }*/
        }

        else if (Recieve_Data.a == "BUS HALTED")
        {
            halt = true;
            nHalt = 0;
            Serial.println("BUS HALTED");
            Send_Data.a = "BUS HALTED";
            Send_Data.nBusStop = nStopID;
            Send_Data.nBusIndex = Recieve_Data.nBusIndex;
            Send_Data.nDirection = Recieve_Data.nDirection;
            send_data(broadcastAddress,
                      (uint8_t *)&Send_Data,
                      sizeof(Send_Data));
        }
        else if (Recieve_Data.a == "BUS MOVING")
        {
            halt = false;
            Serial.println("BUS MOVING");
            Send_Data.a = "BUS MOVING";
            Send_Data.nBusStop = nStopID;
            Send_Data.nBusIndex = Recieve_Data.nBusIndex;
            Send_Data.nDirection = Recieve_Data.nDirection;
            send_data(broadcastAddress,
                      (uint8_t *)&Send_Data,
                      sizeof(Send_Data));
        }

        else if (Recieve_Data.a == "BUS OOS")
        {
            service = false;
            nService = 0;
            Serial.println("BUS OOS");
            Send_Data.a = "BUS OOS";
            Send_Data.nBusStop = nStopID;
            Send_Data.nBusIndex = Recieve_Data.nBusIndex;
            Send_Data.nDirection = Recieve_Data.nDirection;
            send_data(broadcastAddress,
                      (uint8_t *)&Send_Data,
                      sizeof(Send_Data));

            /* esp_err_t result =
             esp_now_send(broadcastAddress, (uint8_t *)
             &Send_Data, sizeof(Send_Data));

             if (result == ESP_OK) {
               Serial.println("Sent with success");
             }
             else {
             Serial.println("Error sending the data");
             }*/
        }
        else
        {
            service = true;
            Serial.println("BUS NOOS");
            Send_Data.a = "BUS NOOS";
            Send_Data.nBusStop = nStopID;
            Send_Data.nBusIndex = Recieve_Data.nBusIndex;
            Send_Data.nDirection = Recieve_Data.nDirection;
            send_data(broadcastAddress,
                      (uint8_t *)&Send_Data,
                      sizeof(Send_Data));

            /*esp_err_t result =
            esp_now_send(broadcastAddress, (uint8_t *)
            &Send_Data, sizeof(Send_Data));

            if (result == ESP_OK) {
              Serial.println("Sent with success");
            }
            else {
            Serial.println("Error sending the data");
            }*/
        }
    }
}

/**
 * @brief Callback function that is executed when data is
 * sent.
 *
 * This function prints the status of the last packet sent.
 * It prints "Delivery Success" if the packet was sent
 * successfully, otherwise it prints "Delivery Fail".
 *
 * @param mac_addr The MAC address of the receiver.
 * @param status The status of the send operation.
 */
void OnDataSent(const uint8_t *mac_addr,
                esp_now_send_status_t status)
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS
                       ? "Delivery Success"
                       : "Delivery Fail");
}
/**
 * @brief Setup function for the Arduino sketch.
 *
 * This function initializes the serial monitor, sets the
 * device as a Wi-Fi station, initializes ESP-NOW, registers
 * callback functions, sets up the button interrupt, and
 * initializes the display.
 */
void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("BUS_STOP_B", NULL, 4);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 4;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    // Once ESPNow is successfully Init, we will register
    // for recv CB to get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    ///////////////////////////////////////////////////////////////

    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin),
                    handleInterrupt, FALLING);

    // Define and add the peer (the bus ESP32)
    esp_now_peer_info_t bus_peer;
    memset(&bus_peer, 0,
           sizeof(bus_peer)); // Clear peer structure
    memcpy(bus_peer.peer_addr, busMAC,
           6);                // Set peer MAC address
    bus_peer.channel = 4;     // Use default channel
    bus_peer.encrypt = false; // No encryption

    if (esp_now_add_peer(&bus_peer) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    /////////////////////////////////////////////////////////////////

    // Initialize the display
    P.begin();
    P.setIntensity(1); // keep it 3 or below as we are
                       // powering off the chip/usb
    P.displayText(Buffer, PA_CENTER, 0, 0, PA_PRINT,
                  PA_NO_EFFECT);

    dwTimeLast = millis();
}
/**
 * @brief Scrolls the text on the display.
 *
 * This function animates the display to scroll the text. It
 * resets the display after each scroll.
 */

void scroll()
{
    if (P.displayAnimate())
    {
        P.displayText(newMessage, scrollAlign, scrollSpeed,
                      scrollPause, scrollEffect,
                      scrollEffect);
        P.displayReset();
    }
}

/**
 * @brief Main loop function for the Arduino sketch.
 *
 * This function continuously checks if the button is
 * pressed and sends a message if it is. It also updates the
 * estimated time of arrival and displays it on the screen.
 * If the bus is halted or out of service, it displays the
 * appropriate message.
 */
void loop()
{
    if (button == true)
    { // If the boot button is pressed
        Message msg;
        msg.buttonPressed = true;
        esp_err_t result = esp_now_send(
            busMAC, (uint8_t *)&msg, sizeof(msg));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
        delay(100); // Debounce delay
        button = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////

    if (!halt)
    {
        nShowTime = UpdateEstimated();

        Serial.println(nShowTime);
        if (P.displayAnimate())
        {
            int s = (nShowTime / 60) + 1;
            sprintf(
                Buffer, "%01d",
                nShowTime); // the number "1" is the number
                            // of digits that you want
                            // always shown on the screen
                            // for this font we can have 5
            P.displayText(Buffer, PA_CENTER, 0, 0, PA_PRINT,
                          PA_NO_EFFECT);
        }

        delay(1000);
    }

    else
    {
        if (nHalt == 0)
        {
            Serial.println("BUS HALTED IN BETWEEN");
            nHalt = 1;
        }
        if (!service)
        {
            if (nService == 0)
            {
                Serial.println("BUS OUT OF SERVICE");
                if (P.displayAnimate())
                {
                    P.displayText("OOS", PA_CENTER, 0, 0,
                                  PA_PRINT, PA_NO_EFFECT);

                    P.displayReset();
                }

                nService = 1;
            }
        }
    }
}