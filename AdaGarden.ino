#include "AdaGarden.h"

// Board
#include "Esp.h"
#include "SPI.h"
// Adafruit IO
#include "AdafruitIO_WiFi.h"
// Sensors
#include "Adafruit_Si7021.h"
#include "DallasTemperature.h"
#include "OneWire.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
// Time 
#include <NTPClient.h>
#include "Time.h"
#include <TimeLib.h>
#include "WiFiUdp.h"

ADC_MODE(ADC_VCC);

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *garden_pump = io.feed("garden-pump");
AdafruitIO_Feed *garden_vcc = io.feed("garden-vcc");
AdafruitIO_Feed *garden_outdoor_temp = io.feed("garden-outdoor-temp");
AdafruitIO_Feed *garden_board_temp = io.feed("garden-board-temp");
AdafruitIO_Feed *garden_board_rh = io.feed("garden-board-rh");
AdafruitIO_Feed *garden_free_heap = io.feed("garden-free-heap");
AdafruitIO_Feed *garden_battery_voltage = io.feed("garden-battery-voltage");
AdafruitIO_Feed *garden_battery_current = io.feed("garden-battery-current");

// Sensors
Adafruit_Si7021 sensor = Adafruit_Si7021();
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 ina219;
Adafruit_INA219 ina219b;

// Time 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Sketch variables
typedef struct {
 bool pump_is_open;
 time_t last_push;
 time_t last_debug_push;
 time_t pump_last_open;
} storeDefinition;

struct {
  uint32_t crc32;
  storeDefinition store;
} rtcData;

uint32_t push_interval = 60;
uint32_t push_debug_interval = 5 * 60;
uint32_t pump_max_interval = 5 * 60;

void setup() {
  pinMode(PUMP_SET_PIN, OUTPUT);
  pinMode(FAN_SET_PIN, OUTPUT);
  pinMode(FAN_UNSET_PIN, OUTPUT);

  if(sensor.begin()){
    Serial.println("Temp/RH Sensor found");
  } else {
    Serial.println("Temp/RH Sensor not found");
  }
  
  float board_temp = sensor.readTemperature();

  // Start fan when its hot (before connecting to wifi)
  if(board_temp > FAN_TRIGGER_TEMP) {
    openFan();
  } else {
    closeFan(); 
  }
  
  // Start current sensors
  ina219.begin(0x41);
  ina219.setCalibration_16V_400mA();
  ina219b.begin(0x44);
  ina219b.setCalibration_16V_400mA();
    
  // Start the serial connection
  Serial.begin(115200);
  // Wait for serial monitor to open
  while(! Serial);
  
  loadRtc();
  
  Serial.println("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // set up a message handler for the count feed.
  // the handleMessage function (defined below)
  // will be called whenever a message is
  // received from adafruit io.
  garden_pump->onMessage(handlePump);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());

  // We are connected
  printWifiDetails();

  sensors.begin();

  // Sync time
  timeClient.begin();
  setSyncProvider(getNtpTime);
  while(timeStatus() == timeNotSet) { delay(50); }
}

void loop() {
  bool store_updated = false;
   
  // Max pump run time
  if(rtcData.store.pump_is_open && now() > (rtcData.store.pump_last_open + pump_max_interval)) {
    closeRelay();
    store_updated = true;
    garden_pump->save(0);
  }

  Serial.println("Time now is :");
  Serial.println(now());
  
  float board_temp = sensor.readTemperature();
  float board_humidity = sensor.readHumidity();
  Serial.print("Board Temp:       "); Serial.print(board_temp); Serial.println("c");
  Serial.println("");
  
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");

  
  float shuntvoltage_b = ina219b.getShuntVoltage_mV();
  float busvoltage_b = ina219b.getBusVoltage_V();
  float current_mA_b = ina219b.getCurrent_mA();
  float loadvoltage_b = busvoltage_b + (shuntvoltage_b / 1000);  
  Serial.print("Bus Voltage B:   "); Serial.print(busvoltage_b); Serial.println(" V");
  Serial.print("Shunt Voltage B: "); Serial.print(shuntvoltage_b); Serial.println(" mV");
  Serial.print("Load Voltage B:  "); Serial.print(loadvoltage_b); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA_b); Serial.println(" mA");
  
  // Start fan when its hot
  if(board_temp > FAN_TRIGGER_TEMP) {
    openFan();
    Serial.println("opening fan");
  } else {
    closeFan(); 
    Serial.println("closing fan");
  }
    
  if(now() > (rtcData.store.last_push + push_interval)) {
    rtcData.store.last_push = now();
    store_updated = true;
    sensors.requestTemperatures();
    
    garden_board_temp->save(board_temp);
    garden_board_rh->save(board_humidity);
    garden_outdoor_temp->save(sensors.getTempCByIndex(0));
    garden_battery_voltage->save(loadvoltage);
    garden_battery_current->save(current_mA);
  }

  
  if(now() > (rtcData.store.last_debug_push + push_debug_interval)) {
    rtcData.store.last_debug_push = now();
    store_updated = true;
    garden_vcc->save((float)ESP.getVcc()/1024.0);
    garden_free_heap->save((float)ESP.getFreeHeap());
  }

  if(store_updated) { 
    Serial.println("Saving store...");
    saveRtc();
  }
  
  io.run();
}

void handlePump(AdafruitIO_Data *data) {
  Serial.print("received <- ");
  Serial.println(data->value());
  
  if(data->toBool()) {
    openRelay();
  } else {
    closeRelay();
  }
  Serial.println("");
}

void openRelay() {
  digitalWrite(PUMP_SET_PIN, HIGH);
  rtcData.store.pump_is_open = true;
  rtcData.store.pump_last_open = now();
}

void closeRelay() {
  digitalWrite(PUMP_SET_PIN, LOW);
  rtcData.store.pump_is_open = false;
}

void openFan() {
  digitalWrite(FAN_SET_PIN, HIGH);
  delay(50);
  digitalWrite(FAN_SET_PIN, LOW);
}

void closeFan() {
  digitalWrite(FAN_UNSET_PIN, HIGH);
  delay(50);
  digitalWrite(FAN_UNSET_PIN, LOW);
}

void printWifiDetails() {
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
}

void printStoreState() {  
  Serial.println("");
  Serial.print("pump_is_open : ");
  Serial.println(rtcData.store.pump_is_open);
  Serial.print("last_push : ");
  Serial.println(rtcData.store.last_push);
  Serial.print("last_debug_push : ");
  Serial.println(rtcData.store.last_debug_push);
  Serial.print("pump_last_open : ");
  Serial.println(rtcData.store.pump_last_open);
}

void loadRtc(){
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Read: ");
    printStoreState();
    Serial.println();
    uint32_t crcOfData = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    
    if (crcOfData != rtcData.crc32) {
      Serial.println("CRC32 INVALID");
      Serial.print("CRC32 of data: ");
      Serial.println(crcOfData, HEX);
      Serial.print("CRC32 read from RTC: ");
      Serial.println(rtcData.crc32, HEX);
      Serial.println("Reseting store...");
      rtcData.store = { false, 0, 0, 0 };
      saveRtc();
    } else {
      Serial.println("CRC32 VALID");
    }
  }
}

void saveRtc() {
   // Update CRC32 of data
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Write: ");
    printStoreState();
    Serial.println();
  }
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}


// Return the time from the Ntp time client
time_t getNtpTime() {
  timeClient.update();
  return timeClient.getEpochTime();
}
