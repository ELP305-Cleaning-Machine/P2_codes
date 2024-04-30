#include <Adafruit_MPU6050.h> // Include MPU6050 library
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h> // Include Wire library for I2C(Inter-Integrated Circuit) communication
#include <esp_now.h>

Adafruit_MPU6050 g_mpuSensor; ///< Create an instance of the
                              ///< MPU6050 sensor

esp_now_peer_info_t
    g_peerInfoFirst; ///< Peer information for ESP-NOW
                     ///< communication with the first peer
esp_now_peer_info_t
    g_peerInfoSecond; ///< Peer information for ESP-NOW
                      ///< communication with the second
                      ///< peer

uint8_t g_arrBroadcastAddress[2][6] = {
    {0xEC, 0x64, 0xC9, 0x86, 0x13, 0xC0},
    {0xEC, 0x64, 0xC9, 0x82, 0x7E,
     0x10}}; ///< MAC addresses for broadcast

String g_arrSSID[2] = {
    "BUS_STOP_A", "BUS_STOP_B"}; ///< SSIDs of the bus stops

int g_iVertex = 0; ///< Vertex index

const int g_iButtonPin = 0; ///< Pin number for the button

int g_iOOSFlag = 0; ///< Flag for out of service status

int g_iHopeFlag = 0; ///< Flag for hope status

volatile bool g_bButtonPressed =
    false; ///< Flag to indicate if the button is pressed

bool g_bPassengerFlag =
    false; ///< Flag for passenger status

int g_iHaltFlag = 0; ///< Flag for halt status

int g_iLatchFlag = 0; ///< Flag for latch status
/**
 * @struct tagMESSAGE
 * @brief Structure for sending and receiving data.
 *
 * @var tagMESSAGE::strMessage
 * @brief A string message.
 * @var tagMESSAGE::iBusStopIndex
 * @brief The index of the bus stop.
 * @var tagMESSAGE::iBusIndex
 * @brief The index of the bus.
 * @var tagMESSAGE::iDirection
 * @brief The direction of the bus.
 *
 * This structure must match the sender structure.
 */
typedef struct tagMESSAGE
{
    String strMessage;
    int iBusStopIndex;
    int iBusIndex;
    int iDirection;
} MESSAGE;
// Create a struct_message called myData
struct_message
    myData; ///< Instance of struct_message to hold data.

const int ledPin = 2; ///< Pin number for the LED.

/**
 * @struct Message
 * @brief Structure to receive data.
 *
 * @var Message::buttonPressed
 * @brief Flag to indicate if the button is pressed.
 */
typedef struct
{
    bool buttonPressed;
} Message;

/**
 * @brief Callback function that is executed when data is
 * received.
 *
 * This function handles the incoming data and sets the
 * passenger flag to true if a message is received.
 *
 * @param mac The MAC address of the sender.
 * @param incomingData The incoming data.
 * @param len The length of the incoming data.
 */
void onDataRecv(const uint8_t *mac,
                const uint8_t *incomingData, int len)
{
    Message msg;
    memcpy(&msg, incomingData, sizeof(msg));
    Serial.println("passengers waiting");

    passenger = true;
}
////////////////////////////////////////////
/**
 * @brief Sends a service message to the bus stops.
 *
 * This function sends a service message (either "BUS OOS"
 * or "BUS NOOS") to the bus stops.
 *
 * @param i The index of the message to be sent.
 */
void send_data_service(int i)
{
    if (i == 0)
    {
        myData.a = "BUS OOS";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[0],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }

        result = esp_now_send(broadcastAddress[1],
                              (uint8_t *)&myData,
                              sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }

    else
    {
        myData.a = "BUS NOOS";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[0],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }

        result = esp_now_send(broadcastAddress[1],
                              (uint8_t *)&myData,
                              sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
}

/**
 * @brief Interrupt service routine for handling button
 * press.
 *
 * This function is called when an interrupt is triggered.
 * It toggles the buttonPressed flag.
 */
void IRAM_ATTR handleInterrupt()
{
    buttonPressed = !buttonPressed;
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
    if (status == ESP_NOW_SEND_SUCCESS)
    {
    }
    else
    {
    }
}

/**
 * @brief Setup function for the Arduino sketch.
 *
 * This function initializes the serial communication, sets
 * the button pin as input and attaches an interrupt to it,
 * initializes the MPU6050 sensor, sets the accelerometer
 * range and filter bandwidth, initializes ESP-NOW,
 * registers the send callback function, adds peers for
 * ESP-NOW communication, registers the receive callback
 * function, and sets the LED pin as output.
 */
void setup(void)
{
    Serial.begin(115200); // Initialize serial communication
    while (!Serial)
        delay(10); // Wait for serial port to connect

    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin),
                    handleInterrupt, FALLING);
    Serial.println(
        "Adafruit MPU6050 test!"); // Print a message to
                                   // serial monitor

    if (!mpu.begin())
    { // Try to initialize MPU6050 sensor
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println(
        "MPU6050 Found!"); // Print a message indicating
                           // MPU6050 is found

    mpu.setAccelerometerRange(
        MPU6050_RANGE_8_G); // Set accelerometer range to
                            // +/- 8G
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    }

    mpu.setFilterBandwidth(
        MPU6050_BAND_21_HZ); // Set filter bandwidth to 21
                             // Hz
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth())
    {
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    }

    Serial.println(""); // Print empty line for readability
    delay(100);         // delay for stabilization

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("BUS", NULL, 4);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register
    // for Send CB to get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    memcpy(peerInfo_1.peer_addr, broadcastAddress[0], 6);
    peerInfo_1.channel = 4;
    peerInfo_1.encrypt = false;
    if (esp_now_add_peer(&peerInfo_1) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    memcpy(peerInfo_2.peer_addr, broadcastAddress[1], 6);
    peerInfo_2.channel = 4;
    peerInfo_2.encrypt = false;
    if (esp_now_add_peer(&peerInfo_2) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    /////////////////////////////////////////////////////
    esp_now_register_recv_cb(onDataRecv);

    pinMode(ledPin, OUTPUT);   // Set LED pin as output
    digitalWrite(ledPin, LOW); // Start with LED off
    /////////////////////////////////////////////////////
}

/**
 * @brief Gets the RSSI value for a specific bus stop.
 *
 * This function scans the networks and returns the RSSI
 * value for a specific bus stop.
 *
 * @param j The index of the bus stop.
 * @return The RSSI value. Returns 0 if the RSSI is greater
 * than or equal to -50, 2 if the SSID matches the bus
 * stop, and 1 otherwise.
 */
int get_rssi(int j)
{
    Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks
    // found
    int n = WiFi.scanNetworks(false, false, false, 50, 4);
    Serial.println("scan done");
    if (n == 0)
    {
        Serial.println("no networks found");
    }
    else
    {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i)
        {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println(
                (WiFi.encryptionType(i) == WIFI_AUTH_OPEN)
                    ? " "
                    : "*");

            if (WiFi.SSID(i) == ssid[j])
            {
                Serial.println(WiFi.RSSI(i));
                if (WiFi.RSSI(i) >= -50)
                {
                    // esp_err_t result =
                    // esp_now_send(broadcastAddress,
                    // (uint8_t *) &myData,
                    // sizeof(myData));
                    return (0);
                }
                return (2);
            }
        }
        return (1);
    }
    Serial.println("");
}
int avg_sample = 0;
int arr[10];

/**
 * @brief Sends a data message to the bus stops.
 *
 * This function sends a data message (either "BUS
 * STOPPED!" or "BUS STARTED!") to the bus stops.
 *
 * @param i The index of the message to be sent. If i is 0,
 * "BUS STOPPED!" is sent. Otherwise, "BUS STARTED!" is
 * sent.
 * @param v The index of the bus stop to send the message
 * to.
 */
void send_data(int i, int v)
{
    if (i == 0)
    {
        myData.a = "BUS STOPPED!";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[v],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
    else
    {
        myData.a = "BUS STARTED!";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[v],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
}

/**
 * @brief Sends a halt message to the bus stops.
 *
 * This function sends a halt message (either "BUS HALTED"
 * or "BUS MOVING") to the bus stops.
 *
 * @param i The index of the message to be sent. If i is 0,
 * "BUS HALTED" is sent. Otherwise, "BUS MOVING" is sent.
 * @param v The index of the bus stop to send the message
 * to.
 */
void send_data_halt(int i, int v)
{
    if (i == 0)
    {
        myData.a = "BUS HALTED";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[v],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
    else
    {
        myData.a = "BUS MOVING";
        myData.nBusIndex = 1;
        myData.nBusStop = 0;
        myData.nDirection = 0;
        esp_err_t result = esp_now_send(broadcastAddress[v],
                                        (uint8_t *)&myData,
                                        sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
}

/**
 * @brief Main loop function for the Arduino sketch.
 *
 * This function continuously checks if the button is
 * pressed and sends a service message if it is. It also
 * checks if a passenger signal is received and turns on
 * the LED if it is. It calculates the average sample of
 * the accelerometer readings and determines if the bus is
 * moving or stopped based on the average. If the bus is
 * moving, it sends a "BUS MOVING" message. If the bus is
 * stopped, it sends a "BUS STOPPED" message. It also
 * checks the RSSI values and sends a halt message if the
 * bus is near a bus stop.
 */
void loop()
{
    /* Get new sensor events with the readings */

    if (buttonPressed)
    {
        if (hope == 0)
        {
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
    if (nOOS == 1 && !buttonPressed)
    {
        Serial.println("CONTINUING SERVICE");
        send_data_service(1);
        nOOS = 0;
        hope = 0;
    }

    if (passenger)
    {
        Serial.println("Received signal, turning on LED");
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(ledPin, HIGH); // Turn on LED
            delay(250);
            digitalWrite(ledPin, LOW);
            delay(250);
        }
        passenger = false;
    }

    if (avg_sample == 10)
    {
        int s = 0;
        for (int j = 0; j < 10; j++)
        {
            s = s + arr[j];
        }
        avg_sample = 0;
        if (s >= 7)
        {
            Serial.println("Bus Moving");
            if (latch == 1)
            {
                if (halt == 0)
                {
                    send_data(1, vertex % 2);
                    latch = 0;
                    vertex++;
                }
                else
                {
                    send_data_halt(1, vertex % 2);
                    send_data_halt(1, (vertex + 1) % 2);
                    latch = 0;
                    halt = 0;
                }
            }
        }
        else
        {
            Serial.println("Bus stopped");
            int t = get_rssi(vertex % 2);
            int x = get_rssi((vertex + 1) % 2);
            if (t == 0 && latch == 0)
            {
                Serial.println("bus arrived at");
                Serial.println(ssid[vertex % 2]);
                send_data(0, vertex % 2);
                latch = 1;
                /*esp_err_t result =
                esp_now_send(broadcastAddress, (uint8_t *)
                &myData, sizeof(myData));

                if (result == ESP_OK) {
                Serial.println("Sent with success");
                }
                else {
                Serial.println("Error sending the data");
                }
                */
            }
            else if ((t == 2 || x == 2) && latch == 0)
            {
                send_data_halt(0, vertex % 2);
                send_data_halt(0, (vertex + 1) % 2);
                latch = 1;
                halt = 1;
            }
            else
            {
                // Do nothin
            }
        }
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    int flag[5]; // Array to store motion detection flags
    double net;
    // Loop to check motion for 10 iterations
    for (int i = 0; i < 5; i++)
    {
        double a_x =
            a.acceleration.x; // Acceleration along x-axis
        double a_y =
            a.acceleration.y; // Acceleration along y-axis
        double a_z =
            a.acceleration.z; // Acceleration along z-axis
        net =
            sqrt((a_x * a_x) + (a_y * a_y) +
                 (a_z * a_z)); // Calculate net acceleration

        // Check if net acceleration falls within a certain
        // range from the offset (acceleration when bus is
        // stopped)
        if (net > 9.4 and net < 10.3)
        {
            // Serial.println("bs");              // Print
            // motion status as "bs=Bus stopped"
            // Serial.print(a.acceleration.x);
            flag[i] =
                0; // Set flag to 0 indicating bus stopped
        }
        else
        {
            // Serial.println("bm");              // Print
            // motion status as "bm=Bus moving"
            // Serial.print(a.acceleration.x);
            flag[i] =
                1; // Set flag to 1 indicating bus moving
        }
        delay(100); // Delay betrween iterations for
                    // stability
    }
    int sum = 0;
    // Calculate sum of motion detection flags
    for (int i = 0; i < 5; i++)
    {
        sum = sum + flag[i];
    }
    // Check if sum indicates bus stopped or moving
    if (sum == 0)
    {
        Serial.println(
            "BS"); // Print message indicating bus stopped
        // Serial.print(net);
        arr[avg_sample] = 0;
    }
    else
    {
        Serial.println("BM"); // Print message indicating
                              // bus is moving
        // Serial.print(net);
        arr[avg_sample] = 1;
    }
    avg_sample = avg_sample + 1;
    Serial.println(""); // Print empty line for readability
}