# ble_esp_sensor_connectable
BLE connectable ESP32 with BME680 sensor

Arduino based project for testing the BLE capabilities.

Sample BLE environment sensor build on ESP32 with the BME680 an BSEC library with following capabilities:
- Output of sensor environment data to eINK 1.54 inch screen and BLE.
- Switch between screens (more data with small font <-> less data big font) using touch pin (ESP32 feature).
- Periodic save of BSEC data into NVS for fast BSEC recovery (BSEC library feature)
- Custom GATT Service UUID for sensor environment datas
- Custom GATT Charateristics UUID for each data item
- BLE notifications support with Client Characteristic Configuration 0x2902 descriptor
- BLE Characteristic Presentation Format 0x2904 descriptor for each custom characteristic   

The BLE dta can be read by the esp_sensor_gw python gateway.
