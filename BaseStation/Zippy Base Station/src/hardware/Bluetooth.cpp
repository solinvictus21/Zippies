
#include <SPI.h>
#include <STBLE.h>

#include "zippies/hardware/Bluetooth.h"

const char* deviceName = "Zippy";
const int deviceNameLength = strlen(deviceName);

//UUIDs for the bluetooth services published by the Zippy; note that they are in little-endian byte order per the BLE spec
//5BF1CEC2-EFC2-44D2-81AE-73FCFD5F7A13
const uint8_t serviceUUID[16] =              { 0x13, 0x7A, 0x5F, 0xFD, 0xFC, 0x73, 0xAE, 0x81, 0xD2, 0x44, 0xC2, 0xEF, 0xC2, 0xCE, 0xF1, 0x5B };
//C9589239-D751-47F7-9C5B-38355F0E9811
const uint8_t transmitUUID[16] =             { 0x11, 0x98, 0x0E, 0x5F, 0x35, 0x38, 0x5B, 0x9C, 0xF7, 0x47, 0x51, 0xD7, 0x39, 0x92, 0x58, 0xC9 };
//7E94E051-995E-4ECB-92C6-12EE64729C4F
const uint8_t lighthouseReceiveUUID[16]      { 0x4F, 0x9C, 0x72, 0x64, 0xEE, 0x12, 0xC6, 0x92, 0xCB, 0x4E, 0x5E, 0x99, 0x51, 0xE0, 0x94, 0x7E };
/*
//53372C2D-2BD5-44C9-990F-A329E5E1F4B5
const uint8_t sensorLeftReceiveUUID[16] =    { 0xB5, 0xF4, 0xE1, 0xE5, 0x29, 0xA3, 0x0F, 0x99, 0xC9, 0x44, 0xD5, 0x2B, 0x2D, 0x2C, 0x37, 0x53 };
//F452B269-4405-42D7-8CD8-0630E778E0D0
const uint8_t sensorRightReceiveUUID[16] =   { 0xD0, 0xE0, 0x78, 0xE7, 0x30, 0x06, 0xD8, 0x8C, 0xD7, 0x42, 0x05, 0x44, 0x69, 0xB2, 0x52, 0xF6 };
//7965B674-B7AE-4E02-B334-872ABBE5999D
const uint8_t computedDataReceiveUUID[16] =  { 0x9D, 0x99, 0xE5, 0xBB, 0x2A, 0x87, 0x34, 0xB3, 0x02, 0x4E, 0xAE, 0xB7, 0x74, 0xB6, 0x65, 0x79 };
*/

//debug output adds extra flash and memory requirements
#define BLE_DEBUG false

#define  ADV_INTERVAL_MIN_MS  50
#define  ADV_INTERVAL_MAX_MS  100

#define LIGHTHOUSE_DATA_LENGTH 20

Bluetooth* currentBluetooth = NULL;

Bluetooth::Bluetooth()
  : serviceHandle(0),
    serviceNameHandle(0),
    apperanceNameHandle(0),
    primaryServiceHandle(0),
    characteristicTransmitHandle(0),
    lighthouseReceiveHandle(0),
    // sensorLeftReceiveHandle(0),
    // sensorRightReceiveHandle(0),
    // computedDataReceiveHandle(0),
    discoveryEnabled(false),
    started(false),
    connectionHandle(0),
    receivedData(NULL),
    receivedDataLength(0)
{
}

bool Bluetooth::start(bool observerMode)
{
  if (started)
    return true;

  if (currentBluetooth != NULL)
    currentBluetooth->stop();
  currentBluetooth = this;

  //init HCI
  HCI_Init();

  //init SPI
  BNRG_SPI_Init();

  //reset BlueNRG SPI interface
  BlueNRG_RST();

  //set our bluetooth address
  uint8_t deviceAddress[] = { 0x4B, 0xFB, 0x50, 0xFB, 0xBF, 0xB8 };
//  uint8_t bdaddr[] = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x02 };
  tBleStatus ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, deviceAddress);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to set our Bluetooth address
//    SerialUSB.println("Bluetooth failed to set address.");
    return false;
  }

  //init GATT; the protocol used to communicate with connected devices
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to init GATT
//    SerialUSB.println("Bluetooth failed to init GATT.");
    return false;
  }

  if (observerMode) {
    if (!startAsObserver())
      return false;
  }
  else {
    if (!startAsPeripheral())
    // if (!startAsBroadcaster())
      return false;
  }

  //+8 dBm output power
  // ret = aci_hal_set_tx_power_level(1, 7);
  //+4 dBm output power
  ret = aci_hal_set_tx_power_level(1, 7);
  if (ret != BLE_STATUS_SUCCESS) {
    // SerialUSB.println("Bluetooth failed to set power level.");
    return false;
  }

  started = true;
  return true;
}

/*
bool Bluetooth::startAsBroadcaster()
{
  //init GAP; the protocol used to advertise the existence of our device
  tBleStatus ret = aci_gap_init_IDB05A1(GAP_BROADCASTER_ROLE_IDB05A1, false, deviceNameLength,
      &serviceHandle, &serviceNameHandle, &apperanceNameHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to init GAP
    // SerialUSB.println("Bluetooth failed to init GAP.");
    return false;
  }

  //start broadcaster mode and broadcast every 1000ms to 5000ms (1000/0.625 = 1600)
  // ret = aci_gap_set_broadcast_mode_IDB05A1(0x00A0, 0x00A0, ADV_NONCONN_IND, STATIC_RANDOM_ADDR,
      // ADVERTISEMENT_PACKET_LENGTH, broadcastData, 0, NULL);
  // if (ret != BLE_STATUS_SUCCESS) {
    //failed to setup broadcast mode
    // SerialUSB.println("Failed to start broadcast mode.");
    // return false;
  // }

  //start broadcaster mode and broadcast every 1000ms to 5000ms (1000/0.625 = 1600)
  ret = aci_gap_update_adv_data(
      ADVERTISEMENT_PACKET_LENGTH,
      broadcastData);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to setup broadcast mode
    SerialUSB.println("Failed to set initial advertisement data.");
    SerialUSB.println(ret, HEX);
    return false;
  }

  // ret = aci_gap_set_non_connectable_IDB05A1(ADV_NONCONN_IND, PUBLIC_ADDR);
  ret = aci_gap_set_broadcast_mode_IDB05A1(
      0x00A0,
      2400,
      ADV_NONCONN_IND,
      PUBLIC_ADDR,
      ADVERTISEMENT_PACKET_LENGTH,
      broadcastData,
      0,
      NULL);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to setup broadcast mode
    // SerialUSB.println("Failed to start broadcast mode.");
    SerialUSB.println(ret, HEX);
    return false;
  }

  // SerialUSB.println("Started broadcast mode.");
  return true;
}

void Bluetooth::sendBroadcastData(float x, float y, float orientation, float linearVelocity, float targetVelocity)
{
  memcpy(broadcastData+6, &x, sizeof(float));
  memcpy(broadcastData+10, &y, sizeof(float));
  memcpy(broadcastData+14, &orientation, sizeof(float));
  memcpy(broadcastData+18, &linearVelocity, sizeof(float));
  memcpy(broadcastData+22, &targetVelocity, sizeof(float));
  aci_gap_update_adv_data(ADVERTISEMENT_PACKET_LENGTH, broadcastData);
}
*/

bool Bluetooth::startAsObserver()
{
  //init GAP; the protocol used to advertise the existence of our device
  tBleStatus ret = aci_gap_init_IDB05A1(
      GAP_OBSERVER_ROLE_IDB05A1,
      PRIVACY_DISABLED,
      deviceNameLength,
      &serviceHandle,
      &serviceNameHandle,
      &apperanceNameHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to init GAP
    SerialUSB.println("Bluetooth failed to init GAP.");
    return false;
  }

  ret = aci_gap_start_observation_procedure_IDB05A1(3200, 1600, PASSIVE_SCAN, PUBLIC_ADDR, true);
  if (ret != BLE_STATUS_SUCCESS) {
    // failed to setup observation mode
    SerialUSB.println("Failed to start observer mode.");
    return false;
  }

  SerialUSB.println("Started observer mode.");
  return true;
}

bool Bluetooth::startAsPeripheral()
{
  //init GAP; the protocol used to advertise the existence of our device
  tBleStatus ret = aci_gap_init_IDB05A1(
      GAP_PERIPHERAL_ROLE_IDB05A1,
      PRIVACY_DISABLED,
      deviceNameLength,
      &serviceHandle,
      &serviceNameHandle,
      &apperanceNameHandle);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to init GAP
//    SerialUSB.println("Bluetooth failed to init GAP.");
    return false;
  }

  //set the name of our device
  ret = aci_gatt_update_char_value(
      serviceHandle,
      serviceNameHandle,
      0,
      deviceNameLength,
      (uint8_t *)deviceName);
  if (ret != BLE_STATUS_SUCCESS) {
    //failed to init our GATT discovery name
//    SerialUSB.println("Bluetooth failed to init GATT discovery name.");
    return false;
  }

  //create the primary service; note that the # of handles we need to reserve is calculated as follows...
  //    service + transmit UUID + transmit value + receive UUID + receive value + CCCD = 6
  //...where CCCD is the Client Configuration Characteristic Descriptor (since char_uuid1 has CHAR_PROP_NOTIFY property)
  ret = aci_gatt_add_serv(
      UUID_TYPE_128,    //we're providing a 128-bit UUID
      serviceUUID,      //the service UUID
      PRIMARY_SERVICE,  //it's the primary service
      24,               //# of handles to reserve
      &primaryServiceHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add primary service.");
    return false;
  }

  //create the transmit characteristic; Bluetooth is in "slave" mode, so the word "transmit" is used here to mean "from the perspective of
  //the other device connected to the Zippy", so we actually receive data via the transmit characteristic
  ret = aci_gatt_add_char(
      primaryServiceHandle,
      UUID_TYPE_128,
      transmitUUID,
      20,
      CHAR_PROP_WRITE_WITHOUT_RESP,
      ATTR_PERMISSION_NONE,
      GATT_NOTIFY_ATTRIBUTE_WRITE,
      16,
      1,
      &characteristicTransmitHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add transmit characteristic.");
    return false;
  }

  //create the lighthouse data receive characteristic; again, the nomenclature is reversed, so the client receives via this characteristic, but it's what
  //the Zippy uses to transmit data
  ret = aci_gatt_add_char(primaryServiceHandle, UUID_TYPE_128, lighthouseReceiveUUID, 20, CHAR_PROP_INDICATE, ATTR_PERMISSION_NONE, 0,
                           16, 1, &lighthouseReceiveHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add left sensor receive characteristic.");
    return false;
  }

  /*
  //create the left sensor receive characteristic
  ret = aci_gatt_add_char(primaryServiceHandle, UUID_TYPE_128, sensorLeftReceiveUUID, 20, CHAR_PROP_INDICATE, ATTR_PERMISSION_NONE, 0,
                           16, 1, &sensorLeftReceiveHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add left sensor receive characteristic.");
    return false;
  }

  //create the right sensor receive characteristic
  ret = aci_gatt_add_char(primaryServiceHandle, UUID_TYPE_128, sensorRightReceiveUUID, 20, CHAR_PROP_INDICATE, ATTR_PERMISSION_NONE, 0,
                           16, 1, &sensorRightReceiveHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add right semsor receive characteristic.");
    return false;
  }

  //create the computed data characteristic
  ret = aci_gatt_add_char(primaryServiceHandle, UUID_TYPE_128, computedDataReceiveUUID, 20, CHAR_PROP_INDICATE, ATTR_PERMISSION_NONE, 0,
                           16, 1, &computedDataReceiveHandle);
  if (ret != BLE_STATUS_SUCCESS) {
//    SerialUSB.println("Bluetooth failed to add computed data characteristic.");
    return false;
  }
  */

  if (!enableDiscovery()) {
//    SerialUSB.println("Bluetooth failed to enable discovery.");
    return false;
  }

//  SerialUSB.println("Successfully initialized Bluetooth.");
  return true;
}

bool Bluetooth::enableDiscovery()
{
  const char local_name[] = {
    AD_TYPE_COMPLETE_LOCAL_NAME,
    'Z', 'i', 'p', 'p', 'y' };
  hci_le_set_scan_resp_data(0, NULL);
  uint8_t serviceUUIDList[17];
  serviceUUIDList[0] = AD_TYPE_128_BIT_SERV_UUID;
  memcpy(serviceUUIDList+1, serviceUUID, sizeof(serviceUUID));
  tBleStatus ret = aci_gap_set_discoverable(
      ADV_IND,
      (ADV_INTERVAL_MIN_MS * 1000) / 625,
      (ADV_INTERVAL_MAX_MS * 1000) / 625,
      STATIC_RANDOM_ADDR,
      NO_WHITE_LIST_USE,
      sizeof(local_name), local_name,
      sizeof(serviceUUIDList), serviceUUIDList,
      0, 0);

  return ret == BLE_STATUS_SUCCESS;
}

/*
tBleStatus Bluetooth::sendLighthouseData(uint8_t* lighthouseDataBuffer)
{
  return aci_gatt_update_char_value(primaryServiceHandle, lighthouseReceiveHandle, 0, LIGHTHOUSE_DATA_LENGTH, lighthouseDataBuffer);
}
*/

bool Bluetooth::sendLighthouseData(float x, float y, float orientation, float linearVelocity, float angularVelocity)
{
  if (!connectionHandle)
    return true;

  uint8_t lighthouseData[LIGHTHOUSE_DATA_LENGTH];
  memcpy(lighthouseData, &x, sizeof(float));
  memcpy(lighthouseData+4, &y, sizeof(float));
  memcpy(lighthouseData+8, &orientation, sizeof(float));
  memcpy(lighthouseData+12, &linearVelocity, sizeof(float));
  memcpy(lighthouseData+16, &angularVelocity, sizeof(float));
  tBleStatus ret = aci_gatt_update_char_value(
      primaryServiceHandle,
      lighthouseReceiveHandle,
      0,
      LIGHTHOUSE_DATA_LENGTH,
      lighthouseData);
  return ret == BLE_STATUS_SUCCESS;
}

/*
tBleStatus Bluetooth::sendSensorLeft(uint8_t* sendBuffer)
{
  return aci_gatt_update_char_value(primaryServiceHandle, sensorLeftReceiveHandle, 0, SENSOR_DATA_LENGTH, sendBuffer);
}

tBleStatus Bluetooth::sendSensorRight(uint8_t* sendBuffer)
{
  return aci_gatt_update_char_value(primaryServiceHandle, sensorRightReceiveHandle, 0, SENSOR_DATA_LENGTH, sendBuffer);
}

tBleStatus Bluetooth::sendComputedData(uint8_t* sendBuffer)
{
  return aci_gatt_update_char_value(primaryServiceHandle, computedDataReceiveHandle, 0, SENSOR_DATA_LENGTH, sendBuffer);
}
*/

void Bluetooth::packetReceived(uint8_t dataLength, uint8_t *data)
{
  //received a bluetooth packet; create a buffer to hold it
  if (receivedData)
    free(receivedData);
  receivedData = (uint8_t*)malloc(dataLength);

  //now copy the data over
  memcpy(receivedData, data, dataLength);
//  for (int i = 0; i < data_length; i++)
//    bluetoothReceiveBuffer[i] = att_data[i];
  receivedDataLength = dataLength;
}

uint8_t Bluetooth::loop()
{
  if (!started)
    return 0;

  if (receivedData) {
    free(receivedData);
    receivedData = NULL;
    receivedDataLength = 0;
  }

  // SerialUSB.println("Processing bluetooth");
  HCI_Process();

  /*
  if (HCI_Queue_Empty()) {
    Enter_LP_Sleep_Mode();
  }
  */

  return receivedDataLength;
}

bool Bluetooth::isConnected()
{
  return connectionHandle != 0;
}

void Bluetooth::stop()
{
  //TODO: research how to properly deallocate all the bluetooth config info allocated in start()
  //ignoring for now, since we currently never actually stop bluetooh
  started = false;
}

void HCI_Event_CB(void *blePacket)
{
  SerialUSB.println("Got bluetooth packet.");
  hci_uart_pckt *hciPacket = (hci_uart_pckt *)blePacket;
  // SerialUSB.print("Got event of type: ");
  // SerialUSB.println(hciPacket->type, HEX);
  if (hciPacket->type != HCI_EVENT_PKT)
    return;

  hci_event_pckt *eventPacket = (hci_event_pckt*)hciPacket->data;
  // SerialUSB.print("Got event of type: ");
  // SerialUSB.println(eventPacket->evt, HEX);
  switch (eventPacket->evt)
  {
    case EVT_LE_META_EVENT:
      {
        //a host bluetooth device connected or is in the process of connecting to us
        evt_le_meta_event* metaEvent = (evt_le_meta_event*)eventPacket->data;

        // SerialUSB.print("Got subevent of type: ");
        // SerialUSB.println(evt->subevent, HEX);
        switch (metaEvent->subevent) {
          case EVT_LE_CONN_COMPLETE:
            {
             SerialUSB.println("Bluetooth device connected.");
              //disable discovery
              aci_gap_set_non_discoverable();
              //capture the connection handle
              evt_le_connection_complete* cc = (evt_le_connection_complete*)metaEvent->data;
              currentBluetooth->connectionHandle = cc->handle;
            }
            break;
          case EVT_LE_ADVERTISING_REPORT:
            {
              le_advertising_info* advEvent = (le_advertising_info*)metaEvent->data;
              if (advEvent->evt_type != 1) {
                SerialUSB.print("Got advertisement of type: ");
                SerialUSB.println(advEvent->evt_type, HEX);
              }
              SerialUSB.print("Got advertisement for address: ");
              for (int i = 0; i < 6; i++) {
                SerialUSB.print(advEvent->bdaddr[i], HEX);
                SerialUSB.print(" ");
              }
              SerialUSB.println();
              /*
              if (advEvent->data_RSSI[1] == 3) {
              SerialUSB.print("Got advertisement of length: ");
              SerialUSB.println(advEvent->data_length);
              static bool displayedOnce = false;
              // if (!displayedOnce && advEvent->data_length == 33) {
                for (int i = 0; i < advEvent->data_length; i++) {
                  SerialUSB.print(advEvent->data_RSSI[i], HEX);
                  SerialUSB.print(" ");
                }
                SerialUSB.println();
                // displayedOnce = true;
              // }
              }
              */
              /*
              switch (advEvent->evt_type) {
                case ADV_IND:
                  break;
                case ADV_DIRECT_IND:
                  break;
                case SCAN_RSP:
                  break;
              }
              break;
              */
            }
        }
      }
      break;

    case EVT_VENDOR:
      {
        evt_blue_aci *blue_evt = (evt_blue_aci *)eventPacket->data;
        // SerialUSB.print("Got vendor event of type: ");
        // SerialUSB.println(blue_evt->ecode, HEX);
        switch (blue_evt->ecode)
        {
          //read request; a little unclear what exactly what this means
          case EVT_BLUE_GATT_READ_PERMIT_REQ:
            {
//              SerialUSB.println("Bluetooth read requested.");
              evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req *)blue_evt->data;
              aci_gatt_allow_read(pr->attr_handle);
            }
            break;

          //check if we received a packet from the host device
          case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
            {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              if (evt->attr_handle == currentBluetooth->characteristicTransmitHandle + 1)
                currentBluetooth->packetReceived(evt->data_length, evt->att_data);
            }
            break;
        }
      }
      break;

    //the currently connected host device disconnected from us
    case EVT_DISCONN_COMPLETE:
      //evt_disconn_complete *evt = (void *)event_pckt->data;
      SerialUSB.println("Bluetooth device disconnected.");
      currentBluetooth->connectionHandle = 0;

      //make the device discoverable again
      currentBluetooth->enableDiscovery();
      break;
  }
}
