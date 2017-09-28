#include "AdaGarden.h"
#include "Esp.h"
#include "SPI.h"
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AdafruitIO_Feed *garden_pump = io.feed("garden-pump");
AdafruitIO_Feed *garden_vcc = io.feed("garden-vcc");
AdafruitIO_Feed *garden_free_heap = io.feed("garden-free-heap");

uint32_t last_set = 0;
uint32_t set_interval = 15000;

ADC_MODE(ADC_VCC);

void setup() {
  pinMode(PUMP_SET_PIN, OUTPUT);
  pinMode(PUMP_UNSET_PIN, OUTPUT);

  // Close relay during boot
  closeRelay();
  
  // Start the serial connection
  Serial.begin(115200);
  
  // wait for serial monitor to open
  while(! Serial);
  
  Serial.print("Connecting to Adafruit IO");

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

  // we are connected
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  Serial.println(io.statusText());

  last_set = millis();
}

void loop() {
  io.run();
  
  if(millis() > (last_set + set_interval)) {
    garden_vcc->save((float)ESP.getVcc()/1024.0);
    garden_free_heap->save((float)ESP.getFreeHeap());
    last_set = millis();
  }
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
  delay(10);
  digitalWrite(PUMP_SET_PIN, LOW);
}

void closeRelay() {
  digitalWrite(PUMP_UNSET_PIN, HIGH);
  delay(10);
  digitalWrite(PUMP_UNSET_PIN, LOW);
 }
