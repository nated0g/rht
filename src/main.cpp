#include <ETH.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "modbus.h"
#include <SPI.h>
#include <Wire.h>

#include "uFire_SHT20.h"

#define SHT20_SAMPLE_RATE_MS 2000
#define MB_UPDATE_RATE_MS 2000
#define IP_CONFIG_FILE "/ipConfig.txt"

uFire_SHT20 sht20;

typedef struct
{
  float temp;
  float rh;
} sht20_sensor_data_t;

static sht20_sensor_data_t sht20_data;

/* Web server template processing */
String processor(const String &var)
{
  if (var == "LOCAL_IP")
  {
    return ETH.localIP().toString().c_str();
  }
  if (var == "GATEWAY_IP")
  {
    return ETH.gatewayIP().toString().c_str();
  }
  if (var == "SUBNET_MASK")
  {
    return ETH.subnetMask().toString().c_str();
  }
  if (var == "DNS_1")
  {
    return ETH.dnsIP().toString().c_str();
  }
  if (var == "SKETCH_SIZE")
  {
    return (String)ESP.getSketchSize();
  }
  if (var == "FREE_HEAP")
  {
    return (String)ESP.getFreeHeap();
  }
  if (var == "FREE_PSRAM")
  {
    return (String)ESP.getFreePsram();
  }
  if (var == "TEMP")
  {
    return (String)((float)mb_holding_regs[0] / 10);
  }
  if (var == "RH")
  {
    return (String)((float)mb_holding_regs[1] / 10);
  }

  return String();
}

unsigned char modbus_buffer[100];

extern bool mb_discrete_input[MAX_DISCRETE_INPUT];
extern bool mb_coils[MAX_COILS];
extern uint16_t mb_input_regs[MAX_INP_REGS];
extern uint16_t mb_holding_regs[MAX_HOLD_REGS];

static bool eth_connected = false;

int processModbusMessage(unsigned char *buffer, int bufferSize);

AsyncWebServer webServer(80);
AsyncServer mbServer(502);

// write static IP config to SPIFFS
bool writeEthConfig(IPAddress ip, IPAddress gw, IPAddress sm, IPAddress dns)
{
  // Remove existing config file
  SPIFFS.remove(IP_CONFIG_FILE);
  // Open file for writing
  File file = SPIFFS.open(IP_CONFIG_FILE, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create config file."));
    return 0;
  }
  // Create json document
  const int capacity = JSON_OBJECT_SIZE(8);
  StaticJsonDocument<capacity> doc;

  doc["ip"] = ip.toString();   // local IP
  doc["gw"] = gw.toString();   // gateway IP
  doc["sm"] = sm.toString();   // subnet mask
  doc["dns"] = dns.toString(); // dns
  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to config file"));
    return 0;
  }
  file.close();
  return 1;
}
/* Web server handlers */
ArRequestHandlerFunction postConfigUpdate = [](AsyncWebServerRequest *req)
{
  IPAddress ip, gw, sm, dns;
  const char *newIP;
  AsyncWebParameter *p;
  if (req->hasParam("ip"), true)
  {
    p = req->getParam("ip", true);
    newIP = p->value().c_str();
    ip.fromString(p->value().c_str());
  }
  if (req->hasParam("sm"), true)
  {
    p = req->getParam("sm", true);
    sm.fromString(p->value().c_str());
  }

  if (req->hasParam("gw"), true)
  {
    p = req->getParam("gw", true);
    gw.fromString(p->value().c_str());
  }

  if (req->hasParam("dns"), true)
  {
    p = req->getParam("dns", true);
    dns.fromString(p->value().c_str());
  }

  //req->send(SPIFFS, "/index.html", String(), false, processor);
  if (writeEthConfig(ip, gw, sm, dns))
    ESP.restart();
  //ETH.config(ip, gw, sm, dns, dns);
};

bool readEthConfig(const char *filename)
{
  const int capacity = JSON_OBJECT_SIZE(10);
  StaticJsonDocument<capacity> doc;
  IPAddress config[4];

  File file = SPIFFS.open(filename);
  if (!file)
  {
    Serial.println(F("Failed to read file"));
    return 0;
  }

  deserializeJson(doc, file);

  config[0].fromString(doc["ip"].as<char *>());
  config[1].fromString(doc["gw"].as<char *>());
  config[2].fromString(doc["sm"].as<char *>());
  config[3].fromString(doc["dns"].as<char *>());

  return ETH.config(config[0], config[1], config[2], config[3]);
}

void printFile(const char *filename)
{
  File file = SPIFFS.open(filename);
  if (!file)
  {
    Serial.println(F("Failed to read file"));
    return;
  }
  while (file.available())
  {
    Serial.print((char)file.read());
  }
  Serial.println();

  file.close();
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started");
    //set eth hostname here

    // TODO:
    // Utilize environment variable
    // for custom hostname specified at build time

    ETH.setHostname("bloom-mb1");
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}
void taskGetSHT(void *args)
{
  sht20_sensor_data_t *data = (sht20_sensor_data_t *)args;
  for (;;)
  {
    data->rh = sht20.humidity();
    data->temp = sht20.temperature();

    vTaskDelay(SHT20_SAMPLE_RATE_MS / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void taskUpdateMB(void *args)
{
  sht20_sensor_data_t *data = (sht20_sensor_data_t *)args;
  for (;;)
  {
    mb_holding_regs[0] = (uint16_t)(data->temp * 10);
    mb_holding_regs[1] = (uint16_t)(data->rh * 10);
    vTaskDelay(MB_UPDATE_RATE_MS / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  WiFi.onEvent(WiFiEvent);

  if (!SPIFFS.begin(true))
  {
    Serial.println("Error mounting SPIFFS");
    return;
  }
  //SPIFFS.format();

  sht20.begin();

  ETH.begin();
  while (!eth_connected)
  {
    delay(500);
    Serial.print(".");
  }
  readEthConfig(IP_CONFIG_FILE);
  printFile(IP_CONFIG_FILE);

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
               { req->send(SPIFFS, "/index.html", String(), false, processor); });

  webServer.on("/ip-config", HTTP_GET, [](AsyncWebServerRequest *req)
               { req->send(SPIFFS, "/ipConfig.txt", "application/json"); });
  webServer.on("/config-update", HTTP_POST, postConfigUpdate);

  mbServer.begin();
  webServer.begin();
  delay(1000);
  Serial.println(ETH.localIP());
  /* TODO: refactor so not nesting lambda functions, for readability */
  mbServer.onClient([](void *s, AsyncClient *c)
                    {
                      if (c == NULL)
                        return;
                      Serial.println("** NEW CLIENT CONNECTED **");
                      c->onData([](void *r, AsyncClient *c, void *buf, size_t len)
                                {
                                  size_t i = 0;
                                  char *adu = (char *)buf;
                                  Serial.print("REQ: ");
                                  for (i = 0; i < len; i++)
                                  {
                                    modbus_buffer[i] = adu[i];
                                    if (adu[i] < 0x10)
                                      Serial.print("0");
                                    Serial.print(adu[i], HEX);
                                    Serial.print(" ");
                                  }
                                  Serial.println();

                                  unsigned int return_length = processModbusMessage(modbus_buffer, static_cast<int>(len) + 1);
                                  c->add((const char *)modbus_buffer, (size_t)return_length);
                                  c->send();
                                });
                    },
                    NULL);

  ArduinoOTA
      .onStart([]()
               {
                 String type;
                 if (ArduinoOTA.getCommand() == U_FLASH)
                   type = "sketch";
                 else // U_SPIFFS
                   type = "filesystem";
                 // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                 Serial.println("Start updating " + type);
               })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
                 Serial.printf("Error[%u]: ", error);
                 if (error == OTA_AUTH_ERROR)
                   Serial.println("Auth Failed");
                 else if (error == OTA_BEGIN_ERROR)
                   Serial.println("Begin Failed");
                 else if (error == OTA_CONNECT_ERROR)
                   Serial.println("Connect Failed");
                 else if (error == OTA_RECEIVE_ERROR)
                   Serial.println("Receive Failed");
                 else if (error == OTA_END_ERROR)
                   Serial.println("End Failed");
               });

  ArduinoOTA.begin();

  // Create main sensor reading task
  xTaskCreate(&taskGetSHT, "getSHT", 10000, &sht20_data, 1, NULL);
  xTaskCreate(&taskUpdateMB, "updateMB", 10000, &sht20_data, 1, NULL);
};

void loop()
{
  ArduinoOTA.handle();
};
