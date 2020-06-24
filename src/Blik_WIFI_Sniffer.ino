#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "Arduino.h"
#include "ArduinoJson.h"
#include "etl/map.h"
#include "SD.h"

#include <Entity.h>
#include <config.h>
#include <State.h>
#include <FreematicsPlus.h>

#define MACBYTES 5
#define WIFI_CHANNEL_SWITCH_INTERVAL (500)
#define WIFI_CHANNEL_MAX (13)
#define MAXMAC 3
#define MAXMACHOLDERSIZE MAXMAC * 4
#define PIN_SD_CS 5
#define SPI_FREQ 1000000

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
uint8_t accCount = 0;
#endif

// Freematics declarations
State state;
MEMS_I2C *mems = 0;

// internal declarations
uint8_t level = 0, channel = 1;
// int channels[] = {1, 6, 11, 99}; // all required channels. 99 is a marker for restart
int channels[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 99}; // all required channels
int *curChannel;
File myFile;
String json;

// String knownMacs[10][2] = { // Just for testing
//     {"iPhone", "E8:FB:E9:68:7D:FD"},
//     {"Mi5", "AC:C1:EE:39:EE:7C"},
//     {"Note7", "70:3A:51:25:24:71"},
//     {"PC", "00:D8:61:0D:03:6E"}};

// String knownManufacturer[10][3] = {
//     {"Apple", "E8FBE9000000", "E8FBE9FFFFFF"}};
// // Xiaomi 70:3a:51:25:24:71
// // Xiaomi ac:c1:ee:39:ee:7c

etl::map<String, Entity, MAXMACHOLDERSIZE> entitiesByMac; // increase the size of the map to not run out of space

static wifi_country_t wifi_country = {.cc = "FR", .schan = 1, .nchan = 13}; //Most recent esp32 library struct

typedef struct
{
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static void wifi_sniffer_packet(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

void wifi_sniffer_init(void)
{
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

void wifi_sniffer_packet(void *buff, wifi_promiscuous_pkt_type_t type)
{
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  // if(ppkt->rx_ctrl.rssi)
  // create an entity so it's easier to manipulate only if it's in the desired range
  Entity entity = Entity(ppkt->rx_ctrl.channel, ppkt->rx_ctrl.rssi, buildString(hdr->addr2, HEX));
  // scan one channel, create JSON and flush
  if (appendToMap(entity))
  {
    printMac(ppkt->rx_ctrl.channel, ppkt->rx_ctrl.rssi, hdr->addr2);
  }
}

bool appendToMap(Entity entity)
{
  bool result = false;
  if (entitiesByMac.find(entity.getMAC()) == entitiesByMac.end())
  {
    // MAC doesn't exist, inserting
    Serial.println("New MAC found, inserting");
    entitiesByMac.insert({entity.getMAC(), entity});
    Serial.println("New size:" + String(entitiesByMac.size()));
    result = true;
  }
  // else
  // {
  //   Serial.println("Existing MAC found, skipping");
  // }
  return result;
}

void jsonToSD()
{
  createJSON();
  entitiesByMac.clear();
}

void createJSON()
{
  if (!entitiesByMac.empty())
  {
    Serial.println("Flushing channel to SD");
    const size_t capacity = JSON_ARRAY_SIZE(MAXMACHOLDERSIZE) + MAXMACHOLDERSIZE * JSON_OBJECT_SIZE(4);
    DynamicJsonDocument doc(capacity);

    auto it = entitiesByMac.begin();
    while (it != entitiesByMac.end())
    {
      JsonObject nestedDoc = doc.createNestedObject();
      nestedDoc["mac"] = it->second.getMAC();
      nestedDoc["rssi"] = it->second.getRSSI();
      nestedDoc["ch"] = it->second.getCH();

      Serial.println(it->second.getMAC() + " Added to JSON ");
      it++;
    }
    myFile = SD.open("/test.json", FILE_APPEND);
    serializeJson(doc, myFile);
    serializeJson(doc, Serial);
    Serial.println();
  }
}

bool waitMotion(long timeout)
{
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY))
  {
    do
    {
      // serverProcess(100);
      // calculate relative movement
      float motion = 0;
      float acc[3];
      if (!mems->read(acc))
        continue;
      if (accCount == 10)
      {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++)
      {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD)
      {
        //lastMotionTime = millis();
        return true;
      }
    } while ((long)(millis() - t) < timeout || timeout == -1);
    return false;
  }
#endif
  // serverProcess(timeout);
  return false;
}

void printMac(unsigned int chan, signed int rssi, const uint8_t addr[])
{
  printf("CHAN=%02d, RSSI=%02d,"
         " MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
         chan, rssi,
         addr[0], addr[1], addr[2],
         addr[3], addr[4], addr[5]);
}

String buildString(const uint8_t addr[], unsigned char decimalPlaces)
{
  String result = String();
  for (int i = 0; i <= MACBYTES; i++)
  {
    result += String(addr[i], decimalPlaces);
    if (i != MACBYTES)
    {
      result += ":";
    }
  }
  return result;
}

void initializeFreematics()
{
  initializeSD();
  // initializeMEMS();
  // initializeOBD();
}

void initializeOBD()
{
}

void initializeSD()
{
  if (!SD.begin(PIN_SD_CS, SPI, SPI_FREQ))
  {
    Serial.println("SD Initialization failed!");
    return;
  }
  Serial.println("SD Initialization succesful.");
}

void initializeMEMS()
{
#if ENABLE_MEMS
  if (!state.check(STATE_MEMS_READY))
  {
    Serial.print("MEMS:");
    mems = new MPU9250;
    byte ret = mems->begin(ENABLE_ORIENTATION);
    if (ret)
    {
      state.set(STATE_MEMS_READY);
      Serial.println("MPU-9250");
    }
    else
    {
      mems->end();
      delete mems;
      mems = new ICM_20948_I2C;
      ret = mems->begin(ENABLE_ORIENTATION);
      if (ret)
      {
        state.set(STATE_MEMS_READY);
        Serial.println("ICM-20948");
      }
      else
      {
        Serial.println("NO");
      }
    }
  }
#endif
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin 5 as an output.
  Serial.begin(115200);
  delay(10);
  wifi_sniffer_init();
  curChannel = channels;
  initializeFreematics();
}

// the loop function runs over and over again forever
void loop()
{
  if (*curChannel == 99)
  {
    curChannel = channels;
  }
  Serial.println("Changed channel:" + String(*curChannel));
  // vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
  wifi_sniffer_set_channel(*curChannel);
  jsonToSD();
  // if (waitMotion(1000))
  // {
  //   Serial.println("++++++++++++++++++++++++++MOTION DETECTED");
  // }
  // else
  // {
  //   Serial.println("NO MOTION DETECTED");
  // }

  delay(1000); // wait for a second
  curChannel++;
}
