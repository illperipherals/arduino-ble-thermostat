#include <ArduinoBLE.h>
#include "DHT.h"
#include <U8x8lib.h>
#include <Wire.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/ PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE);   // OLEDs without Reset of the Display

#define DHTPIN 0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define RELAYPIN D2
#define ONPIN D3
#define HYSTERESIS 2
 
BLEService bleService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service
 
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic switchOverrideCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic relayCharacteristic("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

BLEStringCharacteristic temperatureChar("2A6E", BLERead | BLEWrite | BLENotify, 10);
BLEStringCharacteristic humidityChar("2A6F", BLERead | BLENotify, 10);
 
const int redLED = LED_BUILTIN;
const int blueLED = LEDB;

long previousMillis = 0;
int previousOverride = 0;

bool onState = false;
int setTemperature = 21;
float currentTemperature;
float currentHumidity;
 
void setup() {
  Serial.begin(9600);
  //while (!Serial);

  u8x8.begin();
  u8x8.setFlipMode(1);   // set number from 1 to 3, the screen word will rotary 180
 
  // set pin mode
  pinMode(redLED, OUTPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(ONPIN, INPUT_PULLUP);
  digitalWrite(RELAYPIN, HIGH);
 
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
 
  // set advertised local name and service UUID:
  BLE.setLocalName("Thermostat");
  BLE.setAdvertisedService(bleService);
 
  // add the characteristic to the service
  bleService.addCharacteristic(switchCharacteristic);
  bleService.addCharacteristic(switchOverrideCharacteristic);
  bleService.addCharacteristic(relayCharacteristic);
  bleService.addCharacteristic(temperatureChar);
  bleService.addCharacteristic(humidityChar);
 
  // add service
  BLE.addService(bleService);
 
  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  switchOverrideCharacteristic.writeValue(0);
  relayCharacteristic.writeValue(0);
  temperatureChar.writeValue("0");
  humidityChar.writeValue("0");
 
  // start advertising
  BLE.advertise();
 
  Serial.println("BLE Thermostat");

  dht.begin();
}
 
void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();
 
  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setCursor(0, 0);
    u8x8.print("Connected: ");
    // print the central's MAC address:
    Serial.println(central.address());
    u8x8.setCursor(0, 1);
    u8x8.print(central.address().substring(0,8));
    u8x8.setCursor(0, 2);
    u8x8.print(central.address().substring(9,17));
    // while the central is still connected to peripheral:
    while (central.connected()) {

      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   
          Serial.println("LED on");
          digitalWrite(redLED, LOW); // changed from HIGH to LOW
          onState = true;
        } else {                              
          Serial.println(F("LED off"));
          digitalWrite(redLED, HIGH); // changed from LOW to HIGH
          onState = false;    
        }
      } else {
        switchCharacteristic.writeValue(onState || previousOverride ? 1 : 0);
      }

      if (temperatureChar.written()) {
        setTemperature = temperatureChar.value().toInt();
        temperatureChar.writeValue(String(currentTemperature, 2));
        Serial.print(F("Set new temperature: "));
        Serial.println(setTemperature);
        u8x8.setCursor(0, 4);
        u8x8.print("Set Temp: ");
        u8x8.setCursor(11, 4);
        u8x8.print(setTemperature);
      }
  
      getTemperatureAndHumidity();
      handleRelayOutput();
    }
 
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    //u8x8.setCursor(0, 0);
    //u8x8.print("Disconnected");
  }
  //relay output should be handled even if BT is not connected
  getTemperatureAndHumidity();
  handleRelayOutput();
}

void getTemperatureAndHumidity() {
  //check read temperature once every 5s
  long currentMillis = millis();
  if (currentMillis - previousMillis >= 5000) {
    previousMillis = currentMillis;
    currentHumidity = dht.readHumidity();
    currentTemperature = dht.readTemperature();
    if (isnan(currentHumidity) || isnan(currentTemperature)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      //return;
    }
    Serial.print(F("Humidity: "));
    Serial.println(currentHumidity);
    humidityChar.writeValue(String(currentHumidity, 2));
    
    u8x8.setCursor(0, 6);
    u8x8.print("H: ");
    u8x8.setCursor(4, 6);
    u8x8.print(currentHumidity);
    
    Serial.print(F("Temperature: "));
    Serial.println(currentTemperature);
    temperatureChar.writeValue(String(currentTemperature, 2));

    u8x8.setCursor(0, 7);
    u8x8.print("T: ");
    u8x8.setCursor(4, 7);
    u8x8.print(currentTemperature);
  }
}

void handleRelayOutput() {
  bool isItON = digitalRead(ONPIN) == HIGH;

  //indicate manual mode
  if(isItON) {
    if(previousOverride == 1) {
      digitalWrite(blueLED, HIGH);
      switchOverrideCharacteristic.writeValue(0);
      previousOverride = 0;
    }
  } else {
    if(previousOverride == 0) {
      digitalWrite(blueLED, LOW);
      switchOverrideCharacteristic.writeValue(1);
      previousOverride = 1;
    }
  }
  
  if(isItON && !onState) {
    //only turn on thermostat if switch or state are on
    digitalWrite(RELAYPIN, HIGH);
    relayCharacteristic.writeValue(0);
    return;
  }
  
  if(isnan(currentTemperature)) {
    //temperature is not set, default to thermostat off
    digitalWrite(RELAYPIN, HIGH);
    relayCharacteristic.writeValue(0);
    return;
  }

  
  if(currentTemperature >= setTemperature + HYSTERESIS) {
    //temperature acheived, turn off relay
    digitalWrite(RELAYPIN, HIGH);
    relayCharacteristic.writeValue(0);
  } else if (currentTemperature - HYSTERESIS <= setTemperature) {
    //temperature below set, turn on heating
    digitalWrite(RELAYPIN, LOW);
    relayCharacteristic.writeValue(1);
  }
}
