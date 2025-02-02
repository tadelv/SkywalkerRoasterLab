/***************************************************
 * HiBean ESP32 BLE Roaster Control
 * 
 * Replaces Classic Bluetooth with BLE (NUS).
 * 
 * Service UUID:
 *     6e400001-b5a3-f393-e0a9-e50e24dcca9e
 * Characteristics UUIDs:
 *   - Write Characteristic (RX):
 *     6e400002-b5a3-f393-e0a9-e50e24dcca9e
 *   - Notify Characteristic (TX):
 *     6e400003-b5a3-f393-e0a9-e50e24dcca9e
 *
 * Sends notifications for temperature/status data.
 * Expects commands via the write characteristic.
 ***************************************************/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <QuickPID.h>


// -----------------------------------------------------------------------------
// BLE UUIDs for Nordic UART Service
// -----------------------------------------------------------------------------
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // NUS service
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify


// -----------------------------------------------------------------------------
// BLE Globals
// -----------------------------------------------------------------------------
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;

// -----------------------------------------------------------------------------
// Pin Definitions
// -----------------------------------------------------------------------------
const int TX_PIN = 19;  // Output pin to roaster
const int RX_PIN = 20;  // Input pin from roaster

// -----------------------------------------------------------------------------
// Timing Constants
// -----------------------------------------------------------------------------
const int PREAMBLE        = 7000;
const int PULSE_ONE       = 1200;
const int PULSE_ZERO      = 650;
const int POST_PULSE_DELAY= 750;
const int START_PULSE     = 7500;
const int START_DELAY     = 3800;

// -----------------------------------------------------------------------------
// Buffer Sizes
// -----------------------------------------------------------------------------
const int ROASTER_LENGTH    = 7;   // 7 bytes received from roaster
const int CONTROLLER_LENGTH = 6;   // 6 bytes sent to roaster


// -----------------------------------------------------------------------------
// Control Byte Indices
// -----------------------------------------------------------------------------
enum ControlBytes {
    VENT_BYTE = 0,
    DRUM_BYTE = 3,
    COOL_BYTE = 2,
    FILTER_BYTE = 1,
    HEAT_BYTE = 4,
    CHECK_BYTE = 5
};

uint8_t receiveBuffer[ROASTER_LENGTH];
uint8_t sendBuffer[CONTROLLER_LENGTH];

// Raw temperature values
uint16_t rawTempX, rawTempY;

// Control Bytes & Checksum
void setControlChecksum() {
    uint8_t sum = 0;
    for (int i = 0; i < (CONTROLLER_LENGTH - 1); i++) {
        sum += sendBuffer[i];
    }
    sendBuffer[CHECK_BYTE] = sum;  // Correct use of CHECK_BYTE
}


void setValue(uint8_t* bytePtr, uint8_t value) {
    *bytePtr = value;
    setControlChecksum();
}



// -----------------------------------------------------------------------------
// Temperature Variables & Filter
// -----------------------------------------------------------------------------
double temp          = 0.0;           // Filtered temperature
unsigned long lastEventTime = 0;
const unsigned long LAST_EVENT_TIMEOUT = 10UL * 1000000UL; // 10 seconds (micros)
char CorF            = 'C';           // 'C' or 'F'
int filtWeight       = 80;            // Filter weight (Artisan default)
// Define PID variables
float setpoint = 250.0; // Desired temperature (adjustable)
float input, output;
double Kp = 15, Ki = 0.270, Kd = 25.2;
QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct);
int manualHeatLevel = 50;

// -----------------------------------------------------------------------------
// Command Strings
// -----------------------------------------------------------------------------
const String CMD_READ         = "READ";
const String CMD_HEAT         = "OT1";
const String CMD_VENT         = "OT2";
const String CMD_OFF          = "OFF";
const String CMD_DRUM         = "DRUM";
const String CMD_FILTER       = "FILTER";
const String CMD_COOL         = "COOL";
const String CMD_CHAN         = "CHAN";
const String CMD_UNITS        = "UNITS";
const String CMD_FILTER_WEIGHT= "FILT";

// -----------------------------------------------------------------------------
// Forward Declarations
// -----------------------------------------------------------------------------
void parseAndExecuteCommands(String input);
void executeCommand(String command, uint8_t value);
void shutdown();
void sendMessage();
void getRoasterMessage();

// -----------------------------------------------------------------------------
// BLE Server Callbacks
// -----------------------------------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE: Client connected.");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE: Client disconnected. Restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

// -----------------------------------------------------------------------------
// BLE Characteristic Callbacks
// -----------------------------------------------------------------------------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = String(pCharacteristic->getValue().c_str());

    if (rxValue.length() > 0) {
      String input = String(rxValue.c_str());
      Serial.print("BLE Write Received: ");
      Serial.println(input);
      parseAndExecuteCommands(input);
    }
  }
};

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------
bool itsbeentoolong() {
  unsigned long now = micros();
  unsigned long duration = now - lastEventTime;
  return (duration > LAST_EVENT_TIMEOUT);
}

void notifyClient(const String& message) {
    Serial.println("Attempting to notify BLE client with: " + message);
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(message.c_str());
        pTxCharacteristic->notify();
        Serial.println("Notification sent successfully.");
    } else {
        Serial.println("Notification failed. Device not connected or TX characteristic unavailable.");
    }
}

void shutdown() {
    Serial.println("Shutting down...");
    for (int i = 0; i < CONTROLLER_LENGTH; i++) {
        sendBuffer[i] = 0;
    }
    sendBuffer[CHECK_BYTE] = 0; // Reset checksum
}

void sendMessage() {
    // Start pulse
    pulsePin(TX_PIN, START_PULSE);
    delayMicroseconds(START_DELAY);

    // Send each byte, bit by bit
    for (int i = 0; i < CONTROLLER_LENGTH; i++) {
        for (int j = 0; j < 8; j++) {
            if (bitRead(sendBuffer[i], j) == 1) {
                // '1' bit
                pulsePin(TX_PIN, 1500);
            } else {
                // '0' bit
                pulsePin(TX_PIN, PULSE_ZERO);
            }
            delayMicroseconds(POST_PULSE_DELAY);
        }
    }
}

void pulsePin(int pin, int duration) {
    digitalWrite(pin, LOW);
    delayMicroseconds(duration);
    digitalWrite(pin, HIGH);
}

void getRoasterMessage() {
    bool passedChecksum = false;
    // Keep trying until a valid checksum is found
    while (!passedChecksum) {
        getMessage(ROASTER_LENGTH, RX_PIN);
        passedChecksum = calculateRoasterChecksum();
    }
    // Once valid, compute temperature with filtering
    filtTemp(calculateTemp());
}

void getMessage(int bytes, int pin) {
    unsigned long timeIntervals[ROASTER_LENGTH * 8];
    unsigned long pulseDuration = 0;
    int bits = bytes * 8;

    // Wait for a pulse >= PREAMBLE
    while (pulseDuration < PREAMBLE) {
        pulseDuration = pulseIn(pin, LOW);
    }

    for (int i = 0; i < bits; i++) {
        timeIntervals[i] = pulseIn(pin, LOW);
    }

    for (int i = 0; i < bytes; i++) {
        receiveBuffer[i] = 0;
    }

    for (int i = 0; i < bits; i++) {
        if (timeIntervals[i] > PULSE_ONE) {
            receiveBuffer[i / 8] |= (1 << (i % 8));
        }
    }
}

bool calculateRoasterChecksum() {
    uint8_t sum = 0;
    for (int i = 0; i < (ROASTER_LENGTH - 1); i++) {
        sum += receiveBuffer[i];
    }
    return (sum == receiveBuffer[ROASTER_LENGTH - 1]);
}

double calculateTemp() {
    rawTempX = ((receiveBuffer[0] << 8) + receiveBuffer[1]);
    rawTempY = ((receiveBuffer[2] << 8) + receiveBuffer[3]);

    double x = 0.001 * rawTempX;
    double y = 0.001 * rawTempY;
    double v;  // Declare v here

    if (rawTempX > 836 || rawTempY > 221) {
        v = -224.2 * y * y * y + 385.9 * y * y - 327.1 * y + 171;
    } else {
        v = -278.33 * x * x * x + 491.944 * x * x - 451.444 * x + 310.668;
    }

    if (CorF == 'F') {
        v = 1.8 * v + 32.0;
    }
    return v;
}


void filtTemp(double v) {
    if (fabs(temp - v) > 10.0) {
        temp = v;
    } else {
        temp = (v * (100.0 - filtWeight) + temp * filtWeight) / 100.0;
    }
}

void handleCHAN() {
    String message = "# Active channels set to 2100\r\n";
    Serial.println(message);
    notifyClient(message);
}

//Quickpid hControls///

//adjusting the heating power based on PID temperature control
void handlePIDControl() {
    if (myPID.GetMode() == static_cast<uint8_t>(QuickPID::Control::automatic)) {
        input = temp; // Read current temperature
        myPID.Compute();
        int heatValue = constrain(output, 0, 100);
        handleHEAT(heatValue);
    } else {
        handleHEAT(manualHeatLevel);  // Use stored manual heat level
    }
}


void setPIDMode(bool usePID) {
    if (usePID) {
        myPID.SetMode(QuickPID::Control::automatic); // Enable PID
        Serial.println("PID mode set to AUTOMATIC");
    } else {
        myPID.SetMode(QuickPID::Control::manual); // Disable PID
        manualHeatLevel = 0;  // Set heat to 0% for safety
        handleHEAT(manualHeatLevel); // Apply the change immediately
        Serial.println("PID mode set to MANUAL");
    }
}


void handleOT1(uint8_t value) {
    if (myPID.GetMode() == static_cast<uint8_t>(QuickPID::Control::manual)) {
        manualHeatLevel = constrain(value, 0, 100); // Set manual heat level
        handleHEAT(manualHeatLevel); // Apply the new setting
    }
}

void handleREAD() {
    String output = "0," + String(temp, 1) + "," + String(temp, 1) + "," +
                    String(sendBuffer[HEAT_BYTE]) + "," +
                    String(sendBuffer[VENT_BYTE]) + "\r\n";

    Serial.print("READ Output: ");
    Serial.println(output);
    notifyClient(output);
    lastEventTime = micros();
}

void handleHEAT(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[HEAT_BYTE], value);
    }
    lastEventTime = micros();
}

void handleVENT(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[VENT_BYTE], value);
    }
    lastEventTime = micros();
}

void handleDRUM(uint8_t value) {
    if (value != 0) {
        setValue(&sendBuffer[DRUM_BYTE], 100);
    } else {
        setValue(&sendBuffer[DRUM_BYTE], 0);
    }
    lastEventTime = micros();
}

void handleFILTER(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[FILTER_BYTE], value);
    }
    lastEventTime = micros();
}

void handleCOOL(uint8_t value) {
    if (value <= 100) {
        setValue(&sendBuffer[COOL_BYTE], value);
    }
    lastEventTime = micros();
}

void eStop() {
    Serial.println("Emergency Stop Activated! Heater OFF, Vent 100%");
    handleHEAT(0);   // Turn off heater
    handleVENT(100); // Set vent to 100%
}

void parseAndExecuteCommands(String input) {
    input.trim();
    Serial.println("Parsing command: " + input);

    int split1 = input.indexOf(';');
    String command = "";
    String param = "";
    String subcommand = "";
    
    if (split1 >= 0) {
        command = input.substring(0, split1);
        String remainder = input.substring(split1 + 1);
        int split2 = remainder.indexOf(';');

        if (split2 >= 0) {
            subcommand = remainder.substring(0, split2);
            param = remainder.substring(split2 + 1);
        } else {
            param = remainder;
        }
    } else {
        command = input;
    }

    if (command == "PID") {
        if (param == "ON") {
            setPIDMode(true);  // Enable PID control
        } else if (param == "OFF") {
            setPIDMode(false); // Disable PID control
        } else if (subcommand == "SV") {
            double newSetpoint = param.toDouble();
            if (newSetpoint > 0 && newSetpoint <= 300) {  // Example range check
                setpoint = newSetpoint;
                Serial.print("New Setpoint: ");
                Serial.println(setpoint);
            }
        }
    } else if (command == "OT1") {  
        uint8_t value = param.toInt();
        handleOT1(value);  // Manual heater control (only in MANUAL mode)
    } else if (command == "READ") {
        handleREAD();
    } else if (command == "OT2") {  
        handleVENT(param.toInt());  // Set fan duty
    } else if (command == "OFF") {  
        shutdown();  // Shut down system
    } else if (command == "ESTOP") {  
        eStop();  // Emergency stop (heater = 0, vent = 100)
    } else if (command == "DRUM") {  
        handleDRUM(param.toInt());  // Start/stop the drum
    } else if (command == "FILTER") {  
        handleFILTER(param.toInt());  // Turn on/off filter fan
    } else if (command == "COOL") {  
        handleCOOL(param.toInt());  // Cool the beans
    } else if (command == "CHAN") {  
        handleCHAN();  // Handle TC4 init message
    } else if (command == "UNITS") {  
        if (split1 >= 0) CorF = input.charAt(split1 + 1);  // Set temperature units
    }
}




void setup() {
    Serial.begin(115200);
    Serial.println("Starting HiBean ESP32 BLE Roaster Control...");

    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);
    pinMode(RX_PIN, INPUT);

    BLEDevice::init("ESP32_Skycommand_BLE");
    BLEDevice::setMTU(185);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pRxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("BLE Advertising started...");

// Set QuickPID to start in MANUAL mode
    myPID.SetMode(QuickPID::Control::manual);

    // Ensure heat starts at 0% for safety
    manualHeatLevel = 0;
    handleHEAT(manualHeatLevel);

    shutdown();
}

void loop() {
    if (itsbeentoolong()) {
        shutdown();
    }

    sendMessage();
    getRoasterMessage();

    // Ensure PID or manual heat control is handled
    handlePIDControl();
}
