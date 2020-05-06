#include <bsec.h>
#include "BLEUnits.h"

#define ACCURACY(a) ((a==0)?"Unreliable":((a==1)?"Low":((a==2)?"Medium":((a==3)?"High":"Unknown"))))

#define TOUCH_PIN T6 //connected to GPIO14
#define TOUCH_THRESHOLD 50
#define TOUCH_TIME_LONG 2000UL
#define TOUCH_TIME_SHORT 200UL
unsigned long touched_time;
boolean touched;

uint8_t show_screen=2;
boolean updateHeaders=true;

uint8_t battery_level = 99;

#define ENABLE_GxEPD2_GFX 0
#define DISABLE_DIAGNOSTIC_OUTPUT 1

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>

#include <Preferences.h>
#define NVS_NAMESPACE "BME"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#include "esp_log.h"
static const char* BME = "BME";
static const char* GEN = "GEN";
static const char* NVS = "NVS";
static const char* BSEC = "BSEC";


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2904.h>

#define BLE_NAME "ESP_SENSOR"
BLEServer *bsecServer = NULL;
//#define BatteryService_UUID BLEUUID((uint16_t)0x180F)
BLEService *battService = NULL;
BLECharacteristic battCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_BATTERY_LEVEL), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor battDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));

#define BSEC_SERVICE_UUID "9f6d0000-6dd3-4f81-8b40-46822158d9ee"
BLEService *bsecService = NULL;

//#define tempOffsetUUID "d918703b-2705-440e-a784-944e76fa8053"
#define tempOffsetUUID "9f6d0001-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic tempOffsetCharacteristic(tempOffsetUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor tempOffsetDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 tempOffsetDescriptor_2904;

//#define millisSinceRunUUID "6a2d8c13-9d4c-4645-a1bd-5593e5c64a6f"
#define millisSinceRunUUID "9f6d0002-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic millisSinceRunCharacteristic(millisSinceRunUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor millisSinceRunDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 millisSinceRunDescriptor_2904;

//#define iaqEstimateUUID "4027890e-e147-418f-95ea-ed1455afa183"
#define iaqEstimateUUID "9f6d0003-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic iaqEstimateCharacteristic(iaqEstimateUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor iaqEstimateDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 iaqEstimateDescriptor_2904;

//#define rawTemperatureUUID "17c048d2-6859-4975-8533-03dae651a553"
#define rawTemperatureUUID "9f6d0004-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic rawTemperatureCharacteristic(rawTemperatureUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor rawTemperatureDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 rawTemperatureDescriptor_2904;

//#define pressureUUID "c0a8f050-4131-40b1-ad4d-b2bd6022f27c"
#define pressureUUID "9f6d0005-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic pressureCharacteristic(pressureUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor pressureDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 pressureDescriptor_2904;

//#define rawHumidityUUID "299ed8a6-1fcd-4d60-bfcd-ef392fd13440"
#define rawHumidityUUID "9f6d0006-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic rawHumidityCharacteristic(rawHumidityUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor rawHumidityDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 rawHumidityDescriptor_2904;

//#define gasResistanceUUID "aefcc469-8b66-4f7e-97ff-6f501d312763"
#define gasResistanceUUID "9f6d0007-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic gasResistanceCharacteristic(gasResistanceUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor gasResistanceDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 gasResistanceDescriptor_2904;

//#define stabStatusUUID "def30a68-1ec0-4e8c-adab-998fc34a2dba"
#define stabStatusUUID "9f6d0008-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic stabStatusCharacteristic(stabStatusUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor stabStatusDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 stabStatusDescriptor_2904;

//#define runInStatusUUID "a953bb17-1551-4259-99db-2718abb127de"
#define runInStatusUUID "9f6d0009-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic runInStatusCharacteristic(runInStatusUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor runInStatusDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 runInStatusDescriptor_2904;

//#define temperatureUUID "ebbc45a2-f69b-492d-ae89-ef1930d461c1"
#define temperatureUUID "9f6d000a-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic temperatureCharacteristic(temperatureUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor temperatureDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 temperatureDescriptor_2904;

//#define humidityUUID "cce509b8-af76-4ca2-9407-8dbd120e1ad2"
#define humidityUUID "9f6d000b-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic humidityCharacteristic(humidityUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor humidityDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 humidityDescriptor_2904;

//#define staticIaqUUID "65a7331e-f44d-45f9-a232-7124dc190576"
#define staticIaqUUID "9f6d000c-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic staticIaqCharacteristic(staticIaqUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor staticIaqDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 staticIaqDescriptor_2904;

//#define co2EquivalentUUID "079bb634-27a1-4bbd-9828-4bafa3a84d0b"
#define co2EquivalentUUID "9f6d000d-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic co2EquivalentCharacteristic(co2EquivalentUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor co2EquivalentDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 co2EquivalentDescriptor_2904;

//#define breathVocEquivalentUUID "2167372f-4c0f-4d43-b441-39432fb4fb91"
#define breathVocEquivalentUUID "9f6d000e-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic breathVocEquivalentCharacteristic(breathVocEquivalentUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor breathVocEquivalentDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 breathVocEquivalentDescriptor_2904;

//#define iaqAccuracyUUID "42d542ff-efa3-47cd-b088-b5b98b139b51"
#define iaqAccuracyUUID "9f6d000f-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic iaqAccuracyCharacteristic(iaqAccuracyUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor iaqAccuracyDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 iaqAccuracyDescriptor_2904;

//#define staticIaqAccuracyUUID "ba38b5e8-1f0f-48fd-a1c7-7e168d651507"
#define staticIaqAccuracyUUID "9f6d0010-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic staticIaqAccuracyCharacteristic(staticIaqAccuracyUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor staticIaqAccuracyDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 staticIaqAccuracyDescriptor_2904;

//#define co2AccuracyUUID "84f9bfcd-9a84-45bb-9dec-d3a99f5e97ba"
#define co2AccuracyUUID "9f6d0011-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic co2AccuracyCharacteristic(co2AccuracyUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor co2AccuracyDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 co2AccuracyDescriptor_2904;

//#define breathVocAccuracyUUID "8200fb3e-b3bf-457a-b848-f6efab02740c"
#define breathVocAccuracyUUID "9f6d0012-6dd3-4f81-8b40-46822158d9ee"
BLECharacteristic breathVocAccuracyCharacteristic(breathVocAccuracyUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor breathVocAccuracyDescriptor(BLEUUID((uint16_t)ESP_GATT_UUID_CHAR_DESCRIPTION));
BLE2904 breathVocAccuracyDescriptor_2904;

Preferences pref;
unsigned long sensorRunTime;
unsigned long sensorRunTime_onBoot;
unsigned long lastStateSaveTime;
#define SAVE_STATE_PERIOD 60000UL //save state of BME to the NVM memory every 60 seconds

GxEPD2_BW<GxEPD2_154, GxEPD2_154::HEIGHT> display(GxEPD2_154(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));
Bsec iaqSensor;

#define TEMP_OFFSET_DFLT 1.5
float temp_offset = 1.5;

const uint8_t lines = 14;                   //use 14 lines of text, 12 actual text and 1 empty from top and bottom. Used for Y coordinate calculation
const char timestamp[] = "Time[s]:";
int16_t tsbx, tsby; uint16_t tsbw, tsbh;  
const char rawtemp[] = "RTmp[C]:";
int16_t rtbx, rtby; uint16_t rtbw, rtbh;  
const char pressur[] = "P[mmHg]:";
int16_t prbx, prby; uint16_t prbw, prbh;  
const char rawhumidity[] = "RHum[%]:";
int16_t rhbx, rhby; uint16_t rhbw, rhbh;  
const char rawgas[] = "RGas   :";
int16_t rgbx, rgby; uint16_t rgbw, rgbh;  
const char iaq[] = "IAQ    :";
int16_t iabx, iaby; uint16_t iabw, iabh;  
const char iaqaccur[] = "IAQaccr:";
int16_t irbx, irby; uint16_t irbw, irbh;  
const char temp[] = "Temp[C]:";
int16_t tpbx, tpby; uint16_t tpbw, tpbh;  
const char humidit[] = "Hum[%] :";
int16_t hmbx, hmby; uint16_t hmbw, hmbh;  
const char statiaq[] = "StatIAQ:";
int16_t sibx, siby; uint16_t sibw, sibh;  
const char co2[] = "CO2_eqv:";
int16_t cobx, coby; uint16_t cobw, cobh;  
const char voc[] = "VOC_eqv:";
int16_t vobx, voby; uint16_t vobw, vobh;  

uint16_t maxwidth;

uint8_t _BLEClientConnected = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected ++;
      ESP_LOGI(BSEC,"Client Connected. Total connected %d, %d", _BLEClientConnected, pServer->getConnectedCount());
      bsecServer->getAdvertising()->start(); 
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected --;
      ESP_LOGI(BSEC,"Client Disconnected. Total connected %d, %d", _BLEClientConnected, pServer->getConnectedCount());
    }
};

void writeNVS_TOF(float tos);

class MyTempOffsetCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string new_temp_offset = pCharacteristic->getValue();

  if (new_temp_offset.length() > 0) {
    float tmp=atof(new_temp_offset.c_str());
    if (tmp>0 && tmp<=20) {
      temp_offset=tmp;
      ESP_LOGI(GEN,"Setting new value of temp_offset %f",temp_offset);
      iaqSensor.setTemperatureOffset(temp_offset);      
      writeNVS_TOF(temp_offset);
    } else {
      ESP_LOGE(GEN,"Error setting value temp_offset as received value is %s",new_temp_offset.c_str());      
    }
  }
          
    };
};

void InitBLE() {
  BLEDevice::init(BLE_NAME); 

  bsecServer = BLEDevice::createServer(); 
  bsecServer->setCallbacks(new MyServerCallbacks());

  battService = bsecServer->createService(BLEUUID((uint16_t)ESP_GATT_UUID_BATTERY_SERVICE_SVC));

  battService->addCharacteristic(&battCharacteristic);
  battDescriptor.setValue("Percentage 0 - 100");
  battCharacteristic.addDescriptor(&battDescriptor);
  battCharacteristic.addDescriptor(new BLE2902());

  //Second parameter is number of handles to create. Each characteristic use 2 and descriptior use 1
  bsecService = bsecServer->createService(BLEUUID(BSEC_SERVICE_UUID),110); 


//Add Data change rate UUID as seconds.
//This will tell the device how often the data is  changed, thus allowing to disconnect and save power
  
  tempOffsetCharacteristic.setCallbacks(new MyTempOffsetCharacteristicCallbacks());
  bsecService->addCharacteristic(&tempOffsetCharacteristic);
  tempOffsetDescriptor.setValue("Temperature Offset[C]");
  tempOffsetDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  tempOffsetCharacteristic.addDescriptor(&tempOffsetDescriptor);
  tempOffsetCharacteristic.addDescriptor(new BLE2902());
  tempOffsetDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  tempOffsetDescriptor_2904.setExponent(0);
  tempOffsetDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_TEMPERATURE_CELSIUS);
  tempOffsetDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  tempOffsetCharacteristic.addDescriptor(&tempOffsetDescriptor_2904);
  
  bsecService->addCharacteristic(&millisSinceRunCharacteristic);
  millisSinceRunDescriptor.setValue("Seconds since first Run");
  millisSinceRunDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  millisSinceRunCharacteristic.addDescriptor(&millisSinceRunDescriptor);
  millisSinceRunCharacteristic.addDescriptor(new BLE2902());
  millisSinceRunDescriptor_2904.setFormat(BLE2904::FORMAT_UINT32);
  millisSinceRunDescriptor_2904.setExponent(0);
  millisSinceRunDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_TIME_SECONDS);
  millisSinceRunDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  millisSinceRunCharacteristic.addDescriptor(&millisSinceRunDescriptor_2904);

  bsecService->addCharacteristic(&iaqEstimateCharacteristic);
  iaqEstimateDescriptor.setValue("IAQ Estimate");
  iaqEstimateDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  iaqEstimateCharacteristic.addDescriptor(&iaqEstimateDescriptor);
  iaqEstimateCharacteristic.addDescriptor(new BLE2902());
  iaqEstimateDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  iaqEstimateDescriptor_2904.setExponent(0);
  iaqEstimateDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  iaqEstimateDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  iaqEstimateCharacteristic.addDescriptor(&iaqEstimateDescriptor_2904);

  bsecService->addCharacteristic(&rawTemperatureCharacteristic);
  rawTemperatureDescriptor.setValue("Temperature[C] Raw");
  rawTemperatureDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  rawTemperatureCharacteristic.addDescriptor(&rawTemperatureDescriptor);
  rawTemperatureCharacteristic.addDescriptor(new BLE2902());
  rawTemperatureDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  rawTemperatureDescriptor_2904.setExponent(0);
  rawTemperatureDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_TEMPERATURE_CELSIUS);
  rawTemperatureDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  rawTemperatureCharacteristic.addDescriptor(&rawTemperatureDescriptor_2904);

  bsecService->addCharacteristic(&pressureCharacteristic);
  pressureDescriptor.setValue("Pressure[mmHg]");
  pressureDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  pressureCharacteristic.addDescriptor(&pressureDescriptor);
  pressureCharacteristic.addDescriptor(new BLE2902());
  pressureDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  pressureDescriptor_2904.setExponent(0);
  pressureDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_PRESSURE_MMHG);
  pressureDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  pressureCharacteristic.addDescriptor(&pressureDescriptor_2904);

  bsecService->addCharacteristic(&rawHumidityCharacteristic);
  rawHumidityDescriptor.setValue("Humiduty[%] Raw");
  rawHumidityDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  rawHumidityCharacteristic.addDescriptor(&rawHumidityDescriptor);
  rawHumidityCharacteristic.addDescriptor(new BLE2902());
  rawHumidityDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  rawHumidityDescriptor_2904.setExponent(0);
  rawHumidityDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_PERCENTAGE);
  rawHumidityDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  rawHumidityCharacteristic.addDescriptor(&rawHumidityDescriptor_2904);

  bsecService->addCharacteristic(&gasResistanceCharacteristic);
  gasResistanceDescriptor.setValue("Gas Resistance[Ohm] Raw");
  gasResistanceDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  gasResistanceCharacteristic.addDescriptor(&gasResistanceDescriptor);
  gasResistanceCharacteristic.addDescriptor(new BLE2902());
  gasResistanceDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  gasResistanceDescriptor_2904.setExponent(0);
  gasResistanceDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_RESISTANCE_OHM);
  gasResistanceDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  gasResistanceCharacteristic.addDescriptor(&gasResistanceDescriptor_2904);
  
  bsecService->addCharacteristic(&stabStatusCharacteristic);
  stabStatusDescriptor.setValue("Stabilization Status");
  stabStatusDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  stabStatusCharacteristic.addDescriptor(&stabStatusDescriptor);
  stabStatusCharacteristic.addDescriptor(new BLE2902());
  stabStatusDescriptor_2904.setFormat(BLE2904::FORMAT_BOOLEAN);
  stabStatusDescriptor_2904.setExponent(0);
  stabStatusDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  stabStatusDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  stabStatusCharacteristic.addDescriptor(&stabStatusDescriptor_2904);
  
  bsecService->addCharacteristic(&runInStatusCharacteristic);
  runInStatusDescriptor.setValue("RunIn Status");
  runInStatusDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  runInStatusCharacteristic.addDescriptor(&runInStatusDescriptor);
  runInStatusCharacteristic.addDescriptor(new BLE2902());
  runInStatusDescriptor_2904.setFormat(BLE2904::FORMAT_BOOLEAN);
  runInStatusDescriptor_2904.setExponent(0);
  runInStatusDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  runInStatusDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  runInStatusCharacteristic.addDescriptor(&runInStatusDescriptor_2904);
  
  bsecService->addCharacteristic(&temperatureCharacteristic);
  temperatureDescriptor.setValue("Temperature[C]");
  temperatureDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  temperatureCharacteristic.addDescriptor(&temperatureDescriptor);
  temperatureCharacteristic.addDescriptor(new BLE2902());
  temperatureDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  temperatureDescriptor_2904.setExponent(0);
  temperatureDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_TEMPERATURE_CELSIUS);
  temperatureDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  temperatureCharacteristic.addDescriptor(&temperatureDescriptor_2904);
  
  bsecService->addCharacteristic(&humidityCharacteristic);
  humidityDescriptor.setValue("Humidity[%]");
  humidityDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  humidityCharacteristic.addDescriptor(&humidityDescriptor);
  humidityCharacteristic.addDescriptor(new BLE2902());
  humidityDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  humidityDescriptor_2904.setExponent(0);
  humidityDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_PERCENTAGE);
  humidityDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  humidityCharacteristic.addDescriptor(&humidityDescriptor_2904);
  
  bsecService->addCharacteristic(&staticIaqCharacteristic);
  staticIaqDescriptor.setValue("IAQ Static");
  staticIaqDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  staticIaqCharacteristic.addDescriptor(&staticIaqDescriptor);
  staticIaqCharacteristic.addDescriptor(new BLE2902());
  staticIaqDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  staticIaqDescriptor_2904.setExponent(0);
  staticIaqDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  staticIaqDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  staticIaqCharacteristic.addDescriptor(&staticIaqDescriptor_2904);

  bsecService->addCharacteristic(&co2EquivalentCharacteristic);
  co2EquivalentDescriptor.setValue("CO2 Equivalent");
  co2EquivalentDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  co2EquivalentCharacteristic.addDescriptor(&co2EquivalentDescriptor);
  co2EquivalentCharacteristic.addDescriptor(new BLE2902());
  co2EquivalentDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  co2EquivalentDescriptor_2904.setExponent(0);
  co2EquivalentDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_CONCENTRATION_PPM);
  co2EquivalentDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  co2EquivalentCharacteristic.addDescriptor(&co2EquivalentDescriptor_2904);
  
  bsecService->addCharacteristic(&breathVocEquivalentCharacteristic);
  breathVocEquivalentDescriptor.setValue("Breath VOC Equivalent");
  breathVocEquivalentDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  breathVocEquivalentCharacteristic.addDescriptor(&breathVocEquivalentDescriptor);
  breathVocEquivalentCharacteristic.addDescriptor(new BLE2902());
  breathVocEquivalentDescriptor_2904.setFormat(BLE2904::FORMAT_FLOAT32);
  breathVocEquivalentDescriptor_2904.setExponent(0);
  breathVocEquivalentDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_CONCENTRATION_PPM);
  breathVocEquivalentDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  breathVocEquivalentCharacteristic.addDescriptor(&breathVocEquivalentDescriptor_2904);
  
  bsecService->addCharacteristic(&iaqAccuracyCharacteristic);
  iaqAccuracyDescriptor.setValue("IAQ Accuracy");
  iaqAccuracyDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  iaqAccuracyCharacteristic.addDescriptor(&iaqAccuracyDescriptor);
  iaqAccuracyCharacteristic.addDescriptor(new BLE2902());
  iaqAccuracyDescriptor_2904.setFormat(BLE2904::FORMAT_UINT2);
  iaqAccuracyDescriptor_2904.setExponent(0);
  iaqAccuracyDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  iaqAccuracyDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  iaqAccuracyCharacteristic.addDescriptor(&iaqAccuracyDescriptor_2904);
  
  bsecService->addCharacteristic(&staticIaqAccuracyCharacteristic);
  staticIaqAccuracyDescriptor.setValue("Static IAQ Accuracy");
  staticIaqAccuracyDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  staticIaqAccuracyCharacteristic.addDescriptor(&staticIaqAccuracyDescriptor);
  staticIaqAccuracyCharacteristic.addDescriptor(new BLE2902());
  staticIaqAccuracyDescriptor_2904.setFormat(BLE2904::FORMAT_UINT2);
  staticIaqAccuracyDescriptor_2904.setExponent(0);
  staticIaqAccuracyDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  staticIaqAccuracyDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  staticIaqAccuracyCharacteristic.addDescriptor(&staticIaqAccuracyDescriptor_2904);
  
  bsecService->addCharacteristic(&co2AccuracyCharacteristic);
  co2AccuracyDescriptor.setValue("CO2 Accuracy");
  co2AccuracyDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  co2AccuracyCharacteristic.addDescriptor(&co2AccuracyDescriptor);
  co2AccuracyCharacteristic.addDescriptor(new BLE2902());
  co2AccuracyDescriptor_2904.setFormat(BLE2904::FORMAT_UINT2);
  co2AccuracyDescriptor_2904.setExponent(0);
  co2AccuracyDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  co2AccuracyDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  co2AccuracyCharacteristic.addDescriptor(&co2AccuracyDescriptor_2904);
  
  bsecService->addCharacteristic(&breathVocAccuracyCharacteristic);
  breathVocAccuracyDescriptor.setValue("Breath VOC Accuracy");
  breathVocAccuracyDescriptor.setAccessPermissions(ESP_GATT_PERM_READ);
  breathVocAccuracyCharacteristic.addDescriptor(&breathVocAccuracyDescriptor);
  breathVocAccuracyCharacteristic.addDescriptor(new BLE2902()); 
  breathVocAccuracyDescriptor_2904.setFormat(BLE2904::FORMAT_UINT2);
  breathVocAccuracyDescriptor_2904.setExponent(0);
  breathVocAccuracyDescriptor_2904.setUnit(BLE_GATT_UUID_UNIT_UNITLESS);
  breathVocAccuracyDescriptor_2904.setNamespace(BLE_GATT_UUID_NAMESPACE_NONE);
  breathVocAccuracyCharacteristic.addDescriptor(&breathVocAccuracyDescriptor_2904);
  
  bsecServer->getAdvertising()->addServiceUUID(BLEUUID((uint16_t)ESP_GATT_UUID_BATTERY_SERVICE_SVC));
  bsecServer->getAdvertising()->addServiceUUID(BSEC_SERVICE_UUID);          

//  bsecServer->getAdvertising()->setMinInterval((uint16_t)(1285 / 0.625)); //in msec. 20 msec is minimum
//  bsecServer->getAdvertising()->setMaxInterval((uint16_t)(3000 / 0.625)); //in msec. 10240 is maximum
//  bsecServer->getAdvertising()->setMinPreferred((uint16_t)(417.5 / 1.25)); //in msec. 7.5 msec is minimum
//  bsecServer->getAdvertising()->setMaxPreferred((uint16_t)(546.25 / 1.25)); //in msec. 4000 msec is the maximum

  battService->start();
  bsecService->start();

  bsecServer->getAdvertising()->start(); 
     
}

void updateBLE(){
  if (_BLEClientConnected >0) {
    battCharacteristic.setValue(&battery_level,1); //Setting manually battery level
    battCharacteristic.notify();  

    unsigned long srt=(millis()/1000)+sensorRunTime_onBoot;
    millisSinceRunCharacteristic.setValue((uint32_t&) srt);
    tempOffsetCharacteristic.setValue(temp_offset);
    rawTemperatureCharacteristic.setValue(iaqSensor.rawTemperature);
    float press_mmHg = iaqSensor.pressure * 0.007500616827;
    pressureCharacteristic.setValue(press_mmHg);
    rawHumidityCharacteristic.setValue(iaqSensor.rawHumidity);
    gasResistanceCharacteristic.setValue(iaqSensor.gasResistance);
    iaqEstimateCharacteristic.setValue(iaqSensor.iaqEstimate);
//    iaqAccuracyCharacteristic.setValue(ACCURACY(iaqSensor.iaqAccuracy));
    iaqAccuracyCharacteristic.setValue(&iaqSensor.iaqAccuracy,1);
    temperatureCharacteristic.setValue(iaqSensor.temperature);
    humidityCharacteristic.setValue(iaqSensor.humidity);
    staticIaqCharacteristic.setValue(iaqSensor.staticIaq);
    co2EquivalentCharacteristic.setValue(iaqSensor.co2Equivalent);
    breathVocEquivalentCharacteristic.setValue(iaqSensor.breathVocEquivalent); 
    uint8_t stabStat=(uint8_t)iaqSensor.stabStatus; 
    stabStatusCharacteristic.setValue(&stabStat,1);
    uint8_t runInStat=(uint8_t)iaqSensor.runInStatus;
    runInStatusCharacteristic.setValue(&runInStat,1);
//    staticIaqAccuracyCharacteristic.setValue(ACCURACY(iaqSensor.staticIaqAccuracy));
//    co2AccuracyCharacteristic.setValue(ACCURACY(iaqSensor.co2Accuracy));
//    breathVocAccuracyCharacteristic.setValue(ACCURACY(iaqSensor.breathVocAccuracy));
    staticIaqAccuracyCharacteristic.setValue(&iaqSensor.staticIaqAccuracy,1);
    co2AccuracyCharacteristic.setValue(&iaqSensor.co2Accuracy,1);
    breathVocAccuracyCharacteristic.setValue(&iaqSensor.breathVocAccuracy,1);

    tempOffsetCharacteristic.notify();
    millisSinceRunCharacteristic.notify();
    rawTemperatureCharacteristic.notify();
    pressureCharacteristic.notify();
    rawHumidityCharacteristic.notify();
    gasResistanceCharacteristic.notify();
    iaqEstimateCharacteristic.notify();
    iaqAccuracyCharacteristic.notify();
    temperatureCharacteristic.notify();
    humidityCharacteristic.notify();
    staticIaqCharacteristic.notify();
    co2EquivalentCharacteristic.notify();
    breathVocEquivalentCharacteristic.notify();
    stabStatusCharacteristic.notify();
    runInStatusCharacteristic.notify();
    staticIaqAccuracyCharacteristic.notify();
    co2AccuracyCharacteristic.notify();
    breathVocAccuracyCharacteristic.notify();
  
  }
}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      ESP_LOGI(BME,"BSEC error code : %d",iaqSensor.status);      
      delay(3000);
      ESP.restart();
    } else {
      ESP_LOGI(BME,"BSEC warning code : %d",iaqSensor.status);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      ESP_LOGI(BME,"BME680 error code : %d",iaqSensor.bme680Status);
      delay(3000);
      ESP.restart();
    } else {
      ESP_LOGI(BME,"BME680 warning code : %d",iaqSensor.bme680Status);
    }
  }
}

void printValuesScreen2() {
  uint16_t y;
  display.setRotation(1);
  display.setFont(&FreeSansBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  int16_t s2tmpbx, s2tmpby; uint16_t s2tmpbw, s2tmpbh;
  char s2tmp[10];
  sprintf(s2tmp,"%+#.2f",iaqSensor.temperature);  
  display.getTextBounds(s2tmp, 0, 0, &s2tmpbx, &s2tmpby, &s2tmpbw, &s2tmpbh);

  int16_t s2humbx, s2humby; uint16_t s2humbw, s2humbh;
  char s2hum[10];
  sprintf(s2hum,"%#.2f",iaqSensor.humidity);  
  display.getTextBounds(s2hum, 0, 0, &s2humbx, &s2humby, &s2humbw, &s2humbh);

  int16_t s2iaqbx, s2iaqby; uint16_t s2iaqbw, s2iaqbh;
  char s2iaq[10];
  sprintf(s2iaq,"%#.1f",iaqSensor.staticIaq);  
  display.getTextBounds(s2iaq, 0, 0, &s2iaqbx, &s2iaqby, &s2iaqbw, &s2iaqbh);

  int16_t s2vocbx, s2vocby; uint16_t s2vocbw, s2vocbh;
  char s2voc[10];
  sprintf(s2voc,"%#.1f",iaqSensor.breathVocEquivalent);  
  display.getTextBounds(s2voc, 0, 0, &s2vocbx, &s2vocby, &s2vocbw, &s2vocbh);

  unsigned long time_trigger = ((unsigned long )(millis()/1000))+sensorRunTime_onBoot;
  uint8_t seconds = (time_trigger) % 60;
  uint8_t minutes = ((time_trigger) / 60) % 60;
  uint8_t hours = ((time_trigger) / 3600) % 24;
  uint8_t days = ((time_trigger) / 86400);

  display.setFont(&FreeMonoBold9pt7b);

  int16_t s2timbx, s2timby; uint16_t s2timbw, s2timbh;
  char s2tim[32];
  sprintf(s2tim,"%d day%s %02d:%02d:%02d",days,(days>1)?"s":"",hours,minutes,seconds);     
  display.getTextBounds(s2tim, 0, 0, &s2timbx, &s2timbx, &s2timbw, &s2timbh);

  int16_t s2batbx, s2batby; uint16_t s2batbw, s2batbh;
  char s2bat[32];
  sprintf(s2bat,"Battery: %d%%",battery_level);     
  display.getTextBounds(s2bat, 0, 0, &s2batbx, &s2batby, &s2batbw, &s2batbh);

  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    uint16_t x,y;
    display.fillScreen(GxEPD_WHITE);  

    display.setFont(&FreeMonoBold9pt7b);

    display.setCursor(0, 26);
    display.printf("TEMP");
    
    display.setCursor(0, 66);
    display.printf("HUM");

    display.setCursor(0, 97);
    display.printf("IAQ");
    display.setCursor(0, 115);
    display.printf((iaqSensor.iaqAccuracy==0)?"UNR":((iaqSensor.iaqAccuracy==1)?"LOW":((iaqSensor.iaqAccuracy==2)?"MED":((iaqSensor.iaqAccuracy==3)?"HIG":"UNK"))));                  

    display.setCursor(0, 137);
    display.printf("VOC");
    display.setCursor(0, 155);
    display.printf((iaqSensor.breathVocAccuracy==0)?"UNR":((iaqSensor.breathVocAccuracy==1)?"LOW":((iaqSensor.breathVocAccuracy==2)?"MED":((iaqSensor.breathVocAccuracy==3)?"HIG":"UNK"))));                  

    x=(display.width()-s2timbw)/2; 
    display.setCursor(x, 176);
    display.printf("%s",s2tim);
    x=(display.width()-s2batbw)/2;         
    display.setCursor(x, 196);
    display.printf("%s",s2bat);

    display.setFont(&FreeSansBold24pt7b);

    const uint8_t headrow=42;
    x=(display.width()-headrow-s2tmpbw)/2+headrow; 
    display.setCursor(x, 38);
    display.printf("%s",s2tmp);       

    x=(display.width()-headrow-s2humbw)/2+headrow; 
    display.setCursor(x, 78);
    display.printf("%s",s2hum);       

    x=(display.width()-headrow-s2iaqbw)/2+headrow;
    display.setCursor(x, 118);
    display.printf("%s",s2iaq);       
      
    x=(display.width()-headrow-s2vocbw)/2+headrow; 
    display.setCursor(x, 158);
    display.printf("%s",s2voc);       
             
  }
  while (display.nextPage());
  
}

void printHeaders () {
  uint16_t y;
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.getTextBounds(timestamp, 0, 0, &tsbx, &tsby, &tsbw, &tsbh);
  display.getTextBounds(rawtemp, 0, 0, &rtbx, &rtby, &rtbw, &rtbh);
  display.getTextBounds(pressur, 0, 0, &prbx, &prby, &prbw, &prbh);
  display.getTextBounds(rawhumidity, 0, 0, &rhbx, &rhby, &rhbw, &rhbh);
  display.getTextBounds(rawgas, 0, 0, &rgbx, &rgby, &rgbw, &rgbh);  
  display.getTextBounds(iaq, 0, 0, &iabx, &iaby, &iabw, &iabh);    
  display.getTextBounds(iaqaccur, 0, 0, &irbx, &irby, &irbw, &irbh);      
  display.getTextBounds(temp, 0, 0, &tpbx, &tpby, &tpbw, &tpbh);      
  display.getTextBounds(humidit, 0, 0, &hmbx, &hmby, &hmbw, &hmbh);      
  display.getTextBounds(statiaq, 0, 0, &sibx, &siby, &sibw, &sibh);      
  display.getTextBounds(co2, 0, 0, &cobx, &coby, &cobw, &cobh);        
  display.getTextBounds(voc, 0, 0, &vobx, &voby, &vobw, &vobh);          
  //calculate maximum width of the text
  maxwidth=tsbw;
  maxwidth = (rtbw > maxwidth) ? rtbw : maxwidth;
  maxwidth = (prbw > maxwidth) ? prbw : maxwidth;
  maxwidth = (rhbw > maxwidth) ? rhbw : maxwidth;
  maxwidth = (rgbw > maxwidth) ? rgbw : maxwidth;
  maxwidth = (iabw > maxwidth) ? iabw : maxwidth;
  maxwidth = (irbw > maxwidth) ? irbw : maxwidth;
  maxwidth = (tpbw > maxwidth) ? tpbw : maxwidth;
  maxwidth = (hmbw > maxwidth) ? hmbw : maxwidth;  
  maxwidth = (sibw > maxwidth) ? sibw : maxwidth;
  maxwidth = (cobw > maxwidth) ? cobw : maxwidth;
  maxwidth = (vobw > maxwidth) ? vobw : maxwidth;
    
  display.setPartialWindow(0, 0, display.width(), display.height());

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);  
    y=display.height()/lines*2; display.setCursor(0, y);
    display.print(timestamp);          
    y=display.height()/lines*3; display.setCursor(0, y);
    display.print(rawtemp);              
    y=display.height()/lines*4; display.setCursor(0, y);
    display.print(pressur);                  
    y=display.height()/lines*5; display.setCursor(0, y);
    display.print(rawhumidity);                  
    y=display.height()/lines*6; display.setCursor(0, y);
    display.print(rawgas);                  
    y=display.height()/lines*7; display.setCursor(0, y);
    display.print(iaq);                      
    y=display.height()/lines*8; display.setCursor(0, y);
    display.print(iaqaccur);                  
    y=display.height()/lines*9; display.setCursor(0, y);
    display.print(temp);                  
    y=display.height()/lines*10; display.setCursor(0, y);
    display.print(humidit);                  
    y=display.height()/lines*11; display.setCursor(0, y);
    display.print(statiaq);                  
    y=display.height()/lines*12; display.setCursor(0, y);
    display.print(co2);                  
    y=display.height()/lines*13; display.setCursor(0, y);
    display.print(voc);                  
  }
  while (display.nextPage());

}

void printValues() {
  if (show_screen==1) {
    if (updateHeaders) {
      printHeaders();
      updateHeaders=false;
    }
    printValuesScreen1();
  }
  if (show_screen==2) {
    printValuesScreen2();
    updateHeaders=true;
  }
}

void printValuesScreen1() {
  unsigned long time_trigger = ((unsigned long )(millis()/1000))+sensorRunTime_onBoot;
  uint8_t seconds = (time_trigger) % 60;
  uint8_t minutes = ((time_trigger) / 60) % 60;
  uint8_t hours = ((time_trigger) / 3600) % 60;
  uint16_t y;
  
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.getTextBounds(timestamp, 0, 0, &tsbx, &tsby, &tsbw, &tsbh);

  display.setPartialWindow(maxwidth, 0, display.width()-maxwidth, display.height());
//  display.clearScreen();  
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);  
    y=display.height()/lines*2; display.setCursor(maxwidth, y);
    display.printf(" %02d:%02d:%02d",hours,minutes,seconds);          
    y=display.height()/lines*3; display.setCursor(maxwidth, y);
    display.printf(" %+#.2f",iaqSensor.rawTemperature);              
    y=display.height()/lines*4; display.setCursor(maxwidth, y);
    display.printf(" %#.3f",iaqSensor.pressure * 0.007500616827);                  
    y=display.height()/lines*5; display.setCursor(maxwidth, y);
    display.printf(" %#.2f",iaqSensor.rawHumidity);                  
    y=display.height()/lines*6; display.setCursor(maxwidth, y);
    display.printf(" %.0f",iaqSensor.gasResistance);                  
    y=display.height()/lines*7; display.setCursor(maxwidth, y);
    display.printf(" %#.2f",iaqSensor.iaqEstimate);                  
    y=display.height()/lines*8; display.setCursor(maxwidth, y);
    display.printf((iaqSensor.iaqAccuracy==0)?" UNR":((iaqSensor.iaqAccuracy==1)?" LOW":((iaqSensor.iaqAccuracy==2)?" MED":((iaqSensor.iaqAccuracy==3)?" HIGH":" UNK"))));                  
    y=display.height()/lines*9; display.setCursor(maxwidth, y);
    display.printf(" %+#.2f",iaqSensor.temperature);                  
    y=display.height()/lines*10; display.setCursor(maxwidth, y);
    display.printf(" %#.2f",iaqSensor.humidity);                  
    y=display.height()/lines*11; display.setCursor(maxwidth, y);
    display.printf(" %#.2f",iaqSensor.staticIaq);                  
    y=display.height()/lines*12; display.setCursor(maxwidth, y);
    display.printf(" %.0f",iaqSensor.co2Equivalent);                  
    y=display.height()/lines*13; display.setCursor(maxwidth, y);
    display.printf(" %#.2f",iaqSensor.breathVocEquivalent);                  

  }
  while (display.nextPage());    
}

void clearNVS() {
    if (!pref.begin(NVS_NAMESPACE, false)) {
      ESP_LOGE(NVS,"Error init Preferences library in readNVS.");      
      return;
    }
    if (pref.clear()) {
      ESP_LOGW(NVS,"Cleared NVS namespace %s",NVS_NAMESPACE);
    } else {
      ESP_LOGE(NVS,"Error clearing NVS namespace %s",NVS_NAMESPACE);      
    }
    pref.end();    
}

void writeNVS_TOF(float tos) {
    if (!pref.begin(NVS_NAMESPACE, false)) {
      ESP_LOGE(NVS,"Error init Preferences library in writeNVS_TOF.");      
    } 
    pref.putFloat("TOS",tos); 
    pref.end();     
    ESP_LOGI(NVS,"Wrote to NVS new value of temperature offset %f",tos);      
}

unsigned long readNVS(unsigned long srt, void * buf) {
    if (!pref.begin(NVS_NAMESPACE, false)) {
      ESP_LOGE(NVS,"Error init Preferences library in readNVS.");      
      return 0;
    }
    temp_offset=pref.getFloat("TOS",TEMP_OFFSET_DFLT);
    ESP_LOGI(NVS,"Read TemperatureOffset = %f from Preferences.",temp_offset);               
    srt=pref.getULong("SRT",0);
    ESP_LOGI(NVS,"Read SensorRuntTime = %d from Preferences.",srt);               
    if (srt>0) {    
      size_t sensorDataLength = pref.getBytesLength("SDT");
      size_t sensorDataLengthActual = pref.getBytes("SDT", buf, sensorDataLength);
      if (sensorDataLengthActual != sensorDataLength) {
        ESP_LOGE(NVS,"Read from NVS is not OK. Read %d bytes instead of %d.",sensorDataLengthActual,sensorDataLength);      
        pref.end(); 
        return 0;
      }
      ESP_LOGI(NVS,"Read %d bytes of sensor data from Preferences.",sensorDataLengthActual);           
    } else {
      ESP_LOGW(NVS,"No data in NVS to recover from. SensorRuntTime = %d",srt);       
    }
    pref.end();    
    return srt;
}

size_t writeNVS(unsigned long srt, void * buf, size_t sensorDataLength) {
    if (!pref.begin(NVS_NAMESPACE, false)) {
      ESP_LOGE(NVS,"Error init Preferences library in writeNVS.");      
      return 0;
    }
    pref.putULong("SRT",srt);
    size_t sensorDataLengthActual = pref.putBytes("SDT", buf, sensorDataLength);
    pref.end();    
    return (sensorDataLengthActual==sensorDataLength)?sensorDataLength:0;
}

void countDown() {
  int count=3;
  int touch_value;

  updateHeaders=true;
  display.setRotation(1);
  display.setFont(&FreeSansBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  int16_t tmpx, tmpy; uint16_t tmpw1, tmpw2, tmpw3, tmph;
  display.getTextBounds("Clear", 0, 0, &tmpx, &tmpy, &tmpw1, &tmph);
  display.getTextBounds("in", 0, 0, &tmpx, &tmpy, &tmpw2, &tmph);
  display.getTextBounds("3", 0, 0, &tmpx, &tmpy, &tmpw3, &tmph);
  display.setPartialWindow(0, 0, display.width(), display.height());

  do {
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);   
      display.setCursor((display.width()-tmpw1)/2, 78);
      display.printf("Clear");
      display.setCursor((display.width()-tmpw2)/2, 118);
      display.printf("in");
      display.setCursor((display.width()-tmpw3)/2, 158);
      display.printf("%d",count--);                        
    } while (display.nextPage());
    for (int i=0;i<10;i++) {
      delay (100);
      touch_value = touchRead(TOUCH_PIN);
      if (touch_value >= TOUCH_THRESHOLD) {
        break;
      }
    }
    if (touch_value >= TOUCH_THRESHOLD) {
      break;
    }    
    if (count==0) {
      clearNVS();
      ESP.restart();
    }    
  } while (touch_value < TOUCH_THRESHOLD);
}

extern "C" int rom_phy_get_vdd33();
void battRead () {
  int internalBatReading;
  internalBatReading = rom_phy_get_vdd33();
  ESP_LOGI(GET,"Battery is %d",internalBatReading);
    
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  ESP_LOGI(GEN,"Initializing devices");

  display.init(0);
  ESP_LOGI(GEN,"Display Init - DONE");

  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  ESP_LOGI(BME,"Init BME sensor - DONE");

  ESP_LOGI(BME,"BSEC library version %d.%d.%d.%d",iaqSensor.version.major,iaqSensor.version.minor,iaqSensor.version.major_bugfix,iaqSensor.version.minor_bugfix);
  checkIaqSensorStatus();

  uint8_t n_sensorList=12;
  
  bsec_virtual_sensor_t sensorList[n_sensorList] = {
    BSEC_OUTPUT_RAW_TEMPERATURE, //rawTemperature
    BSEC_OUTPUT_RAW_PRESSURE, //pressure
    BSEC_OUTPUT_RAW_HUMIDITY, //rawHumidity
    BSEC_OUTPUT_RAW_GAS, //gasResistance
    BSEC_OUTPUT_IAQ, //iaqEstimate
    BSEC_OUTPUT_STATIC_IAQ, //staticIaq
    BSEC_OUTPUT_CO2_EQUIVALENT, //co2Equivalent
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, //breathVocEquivalent
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE, //temperature
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY, //humidity
    BSEC_OUTPUT_STABILIZATION_STATUS, //stabStatus
    BSEC_OUTPUT_RUN_IN_STATUS //runInStatus
  };

  uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
  sensorRunTime = readNVS(sensorRunTime,workBuffer);
  sensorRunTime_onBoot=sensorRunTime;
  if (sensorRunTime > 0) {
//Serial.println("Dumping NVS BSEC state buffer");
//for (int i=0;i<BSEC_MAX_STATE_BLOB_SIZE;i++){
//  Serial.print(i);((uint8_t) workBuffer[i]<15)?Serial.print(":0x0"):Serial.print(":0x");Serial.print(workBuffer[i],HEX);Serial.print(",");    
//}
//Serial.println();
    ESP_LOGI(BME,"Recovering state of BME.");        
    iaqSensor.setState(workBuffer);
    if (iaqSensor.status != BSEC_OK) {
      ESP_LOGE(BME,"BSEC error/warning code : %d",iaqSensor.status);
    }
  } 
  lastStateSaveTime=0;
  iaqSensor.setTemperatureOffset(temp_offset);  
  iaqSensor.updateSubscription(sensorList, n_sensorList, BSEC_SAMPLE_RATE_LP);

  checkIaqSensorStatus();
  InitBLE();
}

void loop() {

  if (iaqSensor.run()) { // If new data is available
    ESP_LOGI(GEN,"Sensor read START at: %ld",millis());   
    // Print the header
    ESP_LOGI(BME,"Timestamp [s], raw temperature [°C], pressure [mmHg], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent");
    unsigned long time_trigger = ((unsigned long )(millis()/1000))+sensorRunTime_onBoot;    
    ESP_LOGI(BME,"%lu, %.2f, %.2f, %.2f, %.0f, %.2f, %s, %.2f, %.2f, %.2f, %.0f, %.2f",
      time_trigger, 
      iaqSensor.rawTemperature, 
      iaqSensor.pressure * 0.007500616827, 
      iaqSensor.rawHumidity, 
      iaqSensor.gasResistance, 
      iaqSensor.iaqEstimate, 
      ((iaqSensor.iaqAccuracy==0)?" UNR":((iaqSensor.iaqAccuracy==1)?" LOW":((iaqSensor.iaqAccuracy==2)?" MED":((iaqSensor.iaqAccuracy==3)?" HIGH":" UNK")))), 
      iaqSensor.temperature, 
      iaqSensor.humidity,
      iaqSensor.staticIaq, 
      iaqSensor.co2Equivalent,
      iaqSensor.breathVocEquivalent);
    
    printValues();
    updateBLE();
//    battRead();    
    ESP_LOGI(GEN,"Sensor read END at  : %ld",millis());   
  } else {
    checkIaqSensorStatus();
  }

  if (millis()-lastStateSaveTime > SAVE_STATE_PERIOD ) {
    ESP_LOGI(GEN,"NVS save START at: %ld",millis());       
    uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
    iaqSensor.getState(workBuffer);  
    if (iaqSensor.status == BSEC_OK) { 
      sensorRunTime=sensorRunTime_onBoot+((unsigned long) (millis()/1000));
      size_t sensorDataLen = writeNVS(sensorRunTime,workBuffer,BSEC_MAX_STATE_BLOB_SIZE);
      ESP_LOGI(NVS,"Wrote %d bytes of sensor data to Preferences.",sensorDataLen); 
      ESP_LOGI(NVS,"Wrote sensorRunTime as %d.",sensorRunTime); 
    } else {
      checkIaqSensorStatus();    
    }
    lastStateSaveTime=millis(); //Even in case of error retry only on next interval
    ESP_LOGI(GEN,"NVS save END at  : %ld",millis());   
  }
  //In order to prevent some spikes do very simple filtering with average of 5 reads
  int touch_value = (touchRead(TOUCH_PIN)+touchRead(TOUCH_PIN)+touchRead(TOUCH_PIN)+touchRead(TOUCH_PIN)+touchRead(TOUCH_PIN))/5;
  unsigned long mtime=millis();
  if (!touched && touch_value < TOUCH_THRESHOLD)
  {
    touched_time=mtime;
    touched=true;
  } else if (touched && touch_value < TOUCH_THRESHOLD && mtime-touched_time > TOUCH_TIME_LONG) { //if still touched
    countDown();
    printValues();
    touched=false;             
  } else if (touched && touch_value >= TOUCH_THRESHOLD) { //if not touched
    touched=false; 
    if (mtime-touched_time > TOUCH_TIME_SHORT) {
      if (show_screen == 1) {
        show_screen=2;
        printValues();        
      } else {
        show_screen=1;
        printValues();        
      }
      ESP_LOGW(GEN,"It was a short touch. millis=%d, touched_time=%d, touch_value=%d, TOUCH_THRESHOLD=%d",mtime,touched_time,touch_value,TOUCH_THRESHOLD);      
    }
  } 
    
}
