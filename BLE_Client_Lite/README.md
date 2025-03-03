# BLE_Client_Lite
File used for testing functionality of BLE between the ESP32 and the nRF52832. This is intended to be uploaded to the ESP32 (Client) to connect to the nRF52832 (Server). This lessens the load on the nRF52832.

The main UUIDs involve the serviceUUID (used for connecting to separate device), the vent_angle_UUID (To be sent to the nRF52832), and the battery_life_UUID (to be recieved from the nRF52832)

In order to increase flash utilization, go to Tools -> Partition Scheme and change from default to Huge App. This disables the ability to update firmware wirelessly, but significantly decreases the percentage of RAM that the code takes up.
