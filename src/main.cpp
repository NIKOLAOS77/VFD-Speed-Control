#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <regex>
#include <string>
#include <function.h>
#include <PID_v1.h>

#pragma region variousDeclarations

#define outputToInverter 25
#define inputTemp1 34
#define inputTemp2 35
int NTCValue1 = 0;
int NTCValue2 = 0;
float valueeTemperature1 = 0;
float valueeTemperature2 = 0;
//  initial values for the arrays. these are taken from NTC sensor datasheet.
double ResistanceArray[] = {161638, 153694, 146187, 139088, 132375, 126023, 120012, 114321, 108932, 103827, 98990, 94405, 90057, 85934, 82022, 78310, 74786, 71440, 68261, 65241, 62372, 59643, 57049, 54582, 52234, 50000, 47873, 45848, 43920, 42082, 40332, 38663, 37072, 35554, 34107, 32726, 31408, 30150, 28949, 27802, 26706, 25659, 24659, 23702, 22787, 21913, 21076, 20275, 19509, 18776, 18074, 17401, 16757, 16140, 15549, 14982, 14439, 13918, 13418, 12939, 12479, 12038, 11615, 11208, 10818, 10443, 10083, 9737, 9405, 9085, 8778, 8483, 8199, 7926, 7663, 7410, 7167, 6933, 6707, 6490, 6281, 6080, 5886, 5699, 5519, 5345, 5178, 5017, 4861, 4711, 4566, 4427, 4292, 4162, 4037, 3916, 3799, 3686, 3577, 3471, 3369, 3271, 3176, 3084, 2995, 2909, 2826, 2746, 2668, 2593, 2520, 2450, 2382, 2316, 2252, 2191, 2131, 2073, 2017, 1962, 1910, 1859, 1809, 1761, 1715, 1670, 1626, 1584, 1543, 1503, 1465, 1427, 1391, 1355, 1321, 1288, 1256, 1224, 1194, 1164, 1136, 1108, 1081, 1055, 1029, 1004, 980, 957, 934, 912, 890};  // Temperature ohm or volt 
double TemperatureArray[] = {0  , 1  , 2  , 3  , 4  , 5  , 6  , 7  , 8  , 9  , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 , 28 , 29 , 30 , 31 , 32 , 33 , 34 , 35 , 36 , 37 , 38 , 39 , 40 , 41 , 42 , 43 , 44 , 45 , 46 , 47 , 48 , 49 , 50 , 51 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 66 , 67 , 68 , 69 , 70 , 71 , 72 , 73 , 74 , 75 , 76 , 77 , 78 , 79 , 80 , 81 , 82 , 83 , 84 , 85 , 86 , 87 , 88 , 89 , 90 , 91 , 92 , 93 , 94 , 95 , 96 , 97 , 98 , 99 , 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150};    // Temperature in celcius
int arraySize = sizeof(ResistanceArray) / sizeof(ResistanceArray[0]); // Calculate the size of the arrays
int count=0;

double linearInterpolate(float ntcResistance, double inputArray[], double outputArray[], int arraySize) 
  {
   if (ntcResistance < inputArray[arraySize - 1]) 
   {
     return outputArray[arraySize - 1];
   } 
   else if (ntcResistance > inputArray[0]) 
   {
      // Handle the case where ntcResistance is higher than any value in inputArray
      // For example, return the lowest temperature or a special error value
      return outputArray[0]; // or another appropriate value
   }
   for (int i = 0; i < arraySize - 1; i++) 
   {
      if (ntcResistance <= inputArray[i] && ntcResistance > inputArray[i + 1]) 
      {
        return outputArray[i] + (ntcResistance - inputArray[i]) * (outputArray[i + 1] - outputArray[i]) / (inputArray[i + 1] - inputArray[i]);
      }
   
      // Default case (should not happen if inputArray is properly sorted)
      return 0;
   }
  }
String floatToString(float value) {
      char buffer[10];  // Buffer to hold the converted float, adjust size if needed
      dtostrf(value, 0, 2, buffer);  // Convert float to string with 2 decimal places
      return String(buffer);  // Return the converted string as a String object
  }
// Calculates the NTC resistance
float Rntc(int a)
  {
    if (a >= 4095) 
    {
      // Handle the case as needed. This might be returning a maximum resistance value,
      // or a special value indicating an out-of-range measurement.
      // For example, returning a very high resistance value:
      return 170000; // or any other appropriate value
    }
  
    float e=0;
    float Vntc=3.3*a/4095;
    e=81000*((3.3/Vntc)-1);
  
    return e;
  }
String doubleToString(double value)
  {
    String strValue = String(value, 2);
    return strValue;
  }
//////////////////////PID///////////////////////////////////////
  // Define PID constants these are the default values on startup.Normally stored in EEPROM
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// Define variables for temperature measurement
double setpoint = 45;  // Desired temperature
double input, output;
int outputMode=0;
// Define the PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
////////////////////////////////////////////////////////////////
// Function to parse the BLE message and set Kp, Ki, Kd
void parseBLEMessage(const std::string& message) {
    std::istringstream iss(message);
    std::string segment;
    std::getline(iss, segment, '#'); // Skip the first segment ("set1")

    // Extract Kp
    if (std::getline(iss, segment, '#')) {
        Kp = std::stod(segment);
    }

    // Extract Ki
    if (std::getline(iss, segment, '#')) {
        Ki = std::stod(segment);
    }

    // Extract Kd
    if (std::getline(iss, segment, '#')) {
        Kd = std::stod(segment);
    }
    // Extract Kd
    if (std::getline(iss, segment, '#')) {
        setpoint = std::stod(segment);
    }
    // Extract mode
    if (std::getline(iss, segment)) {
        outputMode = std::stoi(segment);
    }
}

#pragma endregion variousDeclarations
#pragma region timersDeclarations

unsigned long currentTime = millis();//used to calculate delta time
unsigned long oldTime = 0; //used to calculate delta time
float timer1SEC = 0;
float timer3BLEtimeout = 0;
int timer2ForBLEupdateData = 0;
int timer2ForOutputVoltage = 0;
int actualOutput=0;
// calculates the delta time of the loop ie the time taken for one circle.
unsigned long CalculateDeltaTime()
{
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - oldTime;
  oldTime = currentTime;
  return deltaTime;
}

#pragma endregion timersDeclarations
#pragma region blueToothDeclerations

char btPassword[6];
//#define SERVICE_UUID "5FAFC201-1FB5-459E-8fCC-C5C9C331914B"
#define SERVICE_UUID         "1b1af12c-b8de-4d36-8543-eae5f8a5594f"
#define CHARACTERISTIC_UUIDI "1b1af12e-b8de-4d36-8543-eae5f8a5594f"
#define CHARACTERISTIC_UUIDO "1b1af12f-b8de-4d36-8543-eae5f8a5594f"
BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic *pCharacteristicI = NULL;
BLECharacteristic *pCharacteristicO = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool allowBTcomms = false;
bool blePassEnabled = false;
uint16_t conIID = 0;
bool BLEstayConnected = false;

void sendLargeMessageWithBLE(BLECharacteristic* pCharacteristic, const std::string& message) {
    const size_t maxPacketSize = 20;  // Assuming 20 bytes of MTU
    std::string modifiedMessage = "BEG" + message + "END";
    size_t offset = 0;
    std::string chunk;
    
    while(offset < modifiedMessage.size()) {
        bool isLastChunk = offset + maxPacketSize >= modifiedMessage.size();
        bool isFirstChunk = offset == 0;

        if (isFirstChunk) 
        {
            chunk = modifiedMessage.substr(offset, maxPacketSize);
            offset += maxPacketSize;
        }
        else if (!isLastChunk)
        {
            chunk = "MID" + modifiedMessage.substr(offset, maxPacketSize - 3); // Subtracting the size of "MID"
            offset += (maxPacketSize - 3);
        }
        else // For the last chunk, no "MID" prefix.
        {
            chunk = modifiedMessage.substr(offset);
            offset = modifiedMessage.size();  // Ending the loop in the next iteration.
        }
        Serial.println(chunk.c_str());
        pCharacteristic->setValue(chunk);
        pCharacteristic->notify();
        delay(20);  // Small delay to prevent overwhelming the receiver
    }
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    
    deviceConnected = true;
    
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    
    // pServer->startAdvertising(); //  removed 9/5/22
  }
};

void checkToReconnect() // added
{
 
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(200);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising                                 
    oldDeviceConnected = deviceConnected;
    //digitalWrite(6, LOW);
    allowBTcomms = false;
     Serial.println("disconnected ble");
  }

  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
    conIID = pServer->getConnId();
    Serial.println("connected ble");
    delay(10);
    allowBTcomms = true;    
    BLEstayConnected = true;
    timer3BLEtimeout = 0;
  }
}

#pragma endregion blueToothDeclerations

#pragma region EEPROMDeclarations

#define EEPROM_SIZE 150

char* readFromEEPROM(int startByte) {
    byte readValue;
    int currentByte = startByte;
    int index = 0;

    // Allocate memory for the char array.
    char* result = new char[50];

    do {
        readValue = EEPROM.read(currentByte++);
        if (readValue != 0) {
            result[index++] = char(readValue);
        } else {
            result[index] = '\0';
        }
    } while (readValue != 0 && index < 50 - 1);

    return result;
}
void writeToEEPROM(int startByte, const char* data) {
    int currentByte = startByte;
    int index = 0;
    char currentChar = data[index];

    while (currentChar != '\0') {
        EEPROM.write(currentByte++, currentChar);
        currentChar = data[++index];
    }

    // Write the null terminator
    EEPROM.write(currentByte, '\0');

    EEPROM.commit();  // Make sure changes are written to EEPROM
}

void writeToEEPROM(int startByte, const std::string &data) {
    writeToEEPROM(startByte, data.c_str());
}
void writeToEEPROM(int startByte, const String &data) {
    writeToEEPROM(startByte, data.c_str());
}

// Function to write a double to the EEPROM at the specified address
void writeDouble(int address, double value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        EEPROM.write(address + i, *p++);
        EEPROM.commit();
}

// Function to read a double from the EEPROM at the specified address
double readDouble(int address) {
    double value;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(address + i);
    return value;
}
#pragma endregion EEPROMDeclarations



void setup() {
  Serial.begin(115200);
  delay(200);

#pragma region EEPROMsetup
  
if (!EEPROM.begin(EEPROM_SIZE))
  {
    // Serial.println("failed to init EEPROM");
    delay(300);
  }
delay(100);
Kp=readDouble(0);
delay(20);
Ki=readDouble(8);
delay(20);
Kd=readDouble(16);
delay(20);
setpoint=readDouble(24);
delay(20);
outputMode=EEPROM.read(32);
delay(20);

#pragma endregion EEPROMsetup

#pragma region IOSetup

  pinMode(outputToInverter, OUTPUT);
  pinMode(inputTemp1, INPUT);
  pinMode(inputTemp2, INPUT);
  dacWrite(outputToInverter,128);

#pragma endregion IOSetup

#pragma region BluetoothSetup
  
BLEDevice::init("Chiller W2");
BLESecurity *pSecurity = new BLESecurity();
pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY); //no Bonding required
pSecurity->setCapability(ESP_IO_CAP_NONE); // No IO capabilities
pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
pServer = BLEDevice::createServer();
pServer->setCallbacks(new MyServerCallbacks());
pService = pServer->createService(SERVICE_UUID);
BLEDevice::setMTU(50);

  pCharacteristicI = pService->createCharacteristic(
      CHARACTERISTIC_UUIDI,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicI->setValue("hi1");

  pCharacteristicO = pService->createCharacteristic(
      CHARACTERISTIC_UUIDO,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicO->setValue("hi2");

pService->start();
BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
pAdvertising->addServiceUUID(SERVICE_UUID);
pAdvertising->setScanResponse(false); 
pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
pAdvertising->setMaxPreferred(0x12);  
BLEDevice::startAdvertising();

Serial.println("BLE setup done");

#pragma endregion BluetoothSetup

#pragma region PIDsetup
// Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // PWM range for fan speed
#pragma endregion PIDsetup



} //end setup loop

//main loop
void loop() {

myFunction(5);
#pragma region timersFore

  float deltaTime = CalculateDeltaTime();
  timer1SEC = timer1SEC + deltaTime;
  
  if (deviceConnected == true && BLEstayConnected)
    timer3BLEtimeout = timer3BLEtimeout + deltaTime;
 

  #pragma endregion timersFore

#pragma region bluetooth
  checkToReconnect(); // BLE check connects and disconnects

  // notify changed value
  std::string valueO = pCharacteristicO->getValue();
  std::string valueI = pCharacteristicI->getValue();
  
  //if (valueI != "") Serial.println(valueI.c_str());
   
  if (valueI != "" && allowBTcomms) //when ble pass reguired and not provided yet. All requstes ignored untill pass provided.
  {   

    if (valueI.rfind("set1", 0) == 0)
    {
      parseBLEMessage(valueI);
      writeDouble(0, Kp);
      writeDouble(8, Ki);
      writeDouble(16, Kd);
      writeDouble(24, setpoint);
      EEPROM.write(32, outputMode);
      EEPROM.commit();
      //pCharacteristicO->setValue("");
       //pCharacteristicO->notify();
    }
    else
    if (valueI.rfind("set2", 0) == 0)
    {
     //  pCharacteristicO->setValue("");
      // pCharacteristicO->notify();
       
    }
    else
    if (valueI.rfind("set3", 0) == 0)
    {
       //pCharacteristicO->setValue("");
       //pCharacteristicO->notify();
       
    }

    timer3BLEtimeout = 0; valueI = ""; pCharacteristicI->setValue("");pCharacteristicI->notify();
    Serial.println("finished routine valueI");
  }

 if (deviceConnected) timer3BLEtimeout = 0; 

 if (allowBTcomms && timer2ForBLEupdateData>=1)
 {
      count++;

      float floatValue = (output / 255.0) * 10.0;
      
      String w = "t" + floatToString(valueeTemperature1) +
       "#" + floatToString(valueeTemperature2)+
       "#" + floatToString(floatValue)+
       "#" + doubleToString(Kp)+ 
       "#" + doubleToString(Ki)+
       "#" + doubleToString(Kd)+
       "#" + doubleToString(setpoint)+
       "#";
       String modee="";
      if (outputMode==1) modee="1"; else if (outputMode==2) modee="2"; else modee="3";
      w+=modee;
       
      sendLargeMessageWithBLE(pCharacteristicO, w.c_str());

      timer2ForBLEupdateData=0;
 }
#pragma endregion bluetooth

#pragma region outputVoltage


   timer2ForOutputVoltage = 0;
   float usedValue = 0;

   //Values read from input ports for NTC sensors
   NTCValue1 = analogRead(inputTemp1); //0 to 4095
   NTCValue2 = analogRead(inputTemp2); //0 to 4095

   //The actual temerature values measured using the NTC sensors.
   valueeTemperature1 = linearInterpolate(Rntc(NTCValue1), ResistanceArray, TemperatureArray, 151);
   valueeTemperature2 = linearInterpolate(Rntc(NTCValue2), ResistanceArray, TemperatureArray, 151);
 

   if (valueeTemperature1>=valueeTemperature2) {input = valueeTemperature1;}
   else
   if (valueeTemperature2>=valueeTemperature1) {input = valueeTemperature2;}
   
   if (outputMode==1)
   {
     // Compute the PID value
    myPID.Compute();
    dacWrite(outputToInverter, output);
    Serial.print(Rntc(NTCValue1));Serial.print("  ");Serial.print(valueeTemperature1);Serial.print("  ");Serial.print(Rntc(NTCValue2));Serial.print("  ");Serial.println(valueeTemperature2);
   }
   else if (outputMode==2)
   {
   if (usedValue>=20 && usedValue<25) actualOutput=100;
      else if (usedValue>=25 && usedValue<30) actualOutput=200;
      else if (usedValue>=30 && usedValue<35) actualOutput=200;
      else if (usedValue>=40 && usedValue<45) actualOutput=255;
      else actualOutput = 50;
      
   }
  
#pragma endregion outputVoltage



#pragma region timersAft

  if (timer3BLEtimeout >= 15000)
  {
    timer3BLEtimeout = 0;
    BLEstayConnected = false;
    deviceConnected=false;
    allowBTcomms = false;
    pServer->disconnect(conIID);
  }

  if (timer1SEC >= 1000)
  {
    timer1SEC=0;
    timer2ForBLEupdateData +=1;
    timer2ForOutputVoltage +=1;
  }

#pragma endregion timersAft

} // end main loop
