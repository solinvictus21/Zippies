
#include <SPI.h>

#include "zippies/hardware/Bluetooth.h"

// Links to documentation for each of the relevant Bluetooth APIs
//     GATT - https://os.mbed.com/teams/ST/code/X_NUCLEO_IDB0XA1/docs/tip/group__GATT__Functions.html
//     GAP  - https://os.mbed.com/teams/ST/code/X_NUCLEO_IDB0XA1/docs/tip/group__GAP__Functions.html
//     HCI  - https://os.mbed.com/teams/ST/code/X_NUCLEO_IDB0XA1/docs/tip/ble__hci_8c.html
//     

const char *deviceName = "Zippy";
const int deviceNameLength = strlen(deviceName);

// UUIDs for the bluetooth services published by the Zippy; note that they are in little-endian byte order per the BLE spec
// 5BF1CEC2-EFC2-44D2-81AE-73FCFD5F7A13
const uint8_t motorControlServiceUUID[16] = {0x13, 0x7A, 0x5F, 0xFD, 0xFC, 0x73, 0xAE, 0x81, 0xD2, 0x44, 0xC2, 0xEF, 0xC2, 0xCE, 0xF1, 0x5B};
// C9589239-D751-47F7-9C5B-38355F0E9811
const uint8_t motorControlCharacteristicUUID[16] = {0x11, 0x98, 0x0E, 0x5F, 0x35, 0x38, 0x5B, 0x9C, 0xF7, 0x47, 0x51, 0xD7, 0x39, 0x92, 0x58, 0xC9};
// 7E94E051-995E-4ECB-92C6-12EE64729C4D
const uint8_t positionCharacteristicUUID[16] = {0x4D, 0x9C, 0x72, 0x64, 0xEE, 0x12, 0xC6, 0x92, 0xCB, 0x4E, 0x5E, 0x99, 0x51, 0xE0, 0x94, 0x7E};

// debug output adds extra flash and memory requirements
// #define BLE_DEBUG false

// for debugging the local bluetooth code
#define BLUETOOTH_DEBUG true

#define ADV_INTERVAL_MIN_MS 50
#define ADV_INTERVAL_MAX_MS 100

Bluetooth *currentBluetooth = NULL;

Bluetooth::Bluetooth()
    : gapServiceHandle(0),
      deviceNameCharacteristicHandle(0),
      appearanceCharacteristicHandle(0),
      motorControlServiceHandle(0),
      motorControlCharacteristicHandle(0),
      positionCharacteristicHandle(0),
      running(false),
      connectionHandle(0)
{
}

bool Bluetooth::start()
{
    if (running)
        return true;

    if (currentBluetooth != NULL)
        currentBluetooth->stop();
    currentBluetooth = this;

    // init HCI
    HCI_Init();

    // init SPI
    BNRG_SPI_Init();

    // reset BlueNRG SPI interface
    BlueNRG_RST();

    // set our bluetooth address
    uint8_t deviceAddress[] = {0x4B, 0xFB, 0x50, 0xFB, 0xBF, 0xB8};
    tBleStatus ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, deviceAddress);
    if (ret != BLE_STATUS_SUCCESS)
    {
        // failed to set our Bluetooth address
#ifdef BLUETOOTH_DEBUG
        SerialUSB.println("Bluetooth failed to set address.");
#endif
        return false;
    }

    // init GATT; the protocol used to communicate with connected devices
    ret = aci_gatt_init();
    if (ret != BLE_STATUS_SUCCESS)
    {
        // failed to init GATT
#ifdef BLUETOOTH_DEBUG
        SerialUSB.println("Bluetooth failed to init GATT.");
#endif
        return false;
    }

    //start as a Bluetooth server to enable clients to connect
    if (!startAsPeripheral())
        return false;

    //enable discovery by Bluetooth clients
    ret = enableDiscovery();
    if (ret != BLE_STATUS_SUCCESS)
    {
#ifdef BLUETOOTH_DEBUG
        SerialUSB.print(ret);
        SerialUSB.println(" - Bluetooth failed to enable discovery.");
#endif
        return false;
    }

    //+8 dBm output power
    ret = aci_hal_set_tx_power_level(1, 7);
    if (ret != BLE_STATUS_SUCCESS)
    {
#ifdef BLUETOOTH_DEBUG
        SerialUSB.println("Bluetooth failed to set power level.");
#endif
        return false;
    }

    currentTimeMicros = 0;
    motorControlTimestamp = 0;
    running = true;
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth started successfully.");
#endif
    return true;
}

bool Bluetooth::startAsPeripheral()
{
  // register the GAP layer with the GATT layer; the protocol used to advertise the existence of our device
  tBleStatus ret = aci_gap_init_IDB05A1(
      GAP_PERIPHERAL_ROLE_IDB05A1,
      PRIVACY_DISABLED,
      deviceNameLength,
      &gapServiceHandle,
      &deviceNameCharacteristicHandle,
      &appearanceCharacteristicHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    // failed to init GAP
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth failed to init GAP.");
#endif
    return false;
  }

  // set the name of our device
  ret = aci_gatt_update_char_value(
      gapServiceHandle,
      deviceNameCharacteristicHandle,
      0,
      deviceNameLength, (uint8_t*)deviceName);
  if (ret != BLE_STATUS_SUCCESS)
  {
    // failed to init our GATT discovery name
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth failed to init GATT discovery name.");
#endif
    return false;
  }

  // create the primary service; note that the # of handles we need to reserve is calculated as follows...
  //     service + characteristic handles + size of transmit values + CCCD = 6
  //...where CCCD is the Client Configuration Characteristic Descriptor (since char_uuid1 has CHAR_PROP_NOTIFY property)
  ret = aci_gatt_add_serv(
      UUID_TYPE_128,               // we're providing a 128-bit UUID
      motorControlServiceUUID,     // the service UUID
      PRIMARY_SERVICE,             // it's the primary service
      33,                          // # of attribute handles to reserve for this service, (+1 for the existing service handle)
      &motorControlServiceHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth failed to add primary service.");
#endif
    return false;
  }

  // create the transmit characteristic; Bluetooth is in "slave" mode, so the word "transmit" is used here to mean "from the perspective of
  // the other device connected to the Zippy", so we actually receive data via the transmit characteristic
  ret = aci_gatt_add_char(
      motorControlServiceHandle,
      UUID_TYPE_128,
      motorControlCharacteristicUUID,
      CHAR_MOTOR_CONTROL_DATA_LENGTH,
      CHAR_PROP_WRITE_WITHOUT_RESP,
      ATTR_PERMISSION_NONE,
      GATT_NOTIFY_ATTRIBUTE_WRITE,
      16,
      0,
      &motorControlCharacteristicHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth failed to add transmit characteristic.");
#endif
    return false;
  }

  // create the lighthouse data receive characteristic; again, the nomenclature is reversed, so the client receives via this characteristic, but it's what
  // the Zippy uses to transmit data
  ret = aci_gatt_add_char(
      motorControlServiceHandle,
      UUID_TYPE_128,
      positionCharacteristicUUID,
      CHAR_POSITION_DATA_LENGTH,
      CHAR_PROP_NOTIFY,
      ATTR_PERMISSION_NONE,
      GATT_DONT_NOTIFY_EVENTS,
      16,
      0,
      &positionCharacteristicHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
#ifdef BLUETOOTH_DEBUG
    SerialUSB.println("Bluetooth failed to add left sensor receive characteristic.");
#endif
    return false;
  }

#ifdef BLUETOOTH_DEBUG
  SerialUSB.println("Successfully setup as peripheral.");
#endif
  return true;
}

tBleStatus Bluetooth::enableDiscovery()
{

    tBleStatus ret = hci_le_set_scan_resp_data(0, NULL);
    if (ret != BLE_STATUS_SUCCESS)
    {
#ifdef BLUETOOTH_DEBUG
        SerialUSB.println("Bluetooth failed to init discover scan response.");
#endif
        return ret;
    }

    uint8_t serviceUUIDList[17];
    serviceUUIDList[0] = AD_TYPE_128_BIT_SERV_UUID;
    memcpy(serviceUUIDList + 1, motorControlServiceUUID, sizeof(motorControlServiceUUID));
    return aci_gap_set_discoverable(
        ADV_IND,
        (ADV_INTERVAL_MIN_MS * 1000) / 625,
        (ADV_INTERVAL_MAX_MS * 1000) / 625,
        STATIC_RANDOM_ADDR,
        NO_WHITE_LIST_USE,
        deviceNameLength, deviceName,
        sizeof(serviceUUIDList), serviceUUIDList,
        0, 0);
}

bool Bluetooth::loop(unsigned long currentTimeMicros, unsigned long deltaTimeMicros)
{
    if (!running)
        return false;

    //need to capture this so it can be attached to data packets received in the HCI_Event_CB callback
    this->currentTimeMicros = currentTimeMicros;
    // process Bluetooth events
    do {
        HCI_Process();
    } while (running && !HCI_Queue_Empty());

    // TODO: determine when we can enter sleep mode and when to get back out of it

    return true;
}

void Bluetooth::packetReceived(uint8_t dataLength, uint8_t *data)
{
    memcpy(motorControlData, data, dataLength);
    motorControlTimestamp = currentTimeMicros;
}

bool Bluetooth::sendLighthouseData(bool lighthouseIsVisible, float x, float y, float orientation, float linearVelocity, float angularVelocity)
{
    if (!connectionHandle)
        return true;

    uint8_t lighthouseData[CHAR_POSITION_DATA_LENGTH];
    lighthouseData[0] = (uint8_t)lighthouseIsVisible;
    //use 16-bit unsigned integers to reduce payload while still providing 1/10mm accuracy
    int16_t data = (int16_t)(x * 10.0);
    memcpy(lighthouseData + 1, &data, sizeof(int16_t));
    data = (int16_t)(y * 10.0);
    memcpy(lighthouseData + 3, &data, sizeof(int16_t));
    data = (int16_t)(orientation * RAD_TO_DEG * 10.0);
    memcpy(lighthouseData + 5, &data, sizeof(int16_t));
    data = (int16_t)(linearVelocity * 10.0);
    memcpy(lighthouseData + 7, &data, sizeof(int16_t));
    data = (int16_t)(angularVelocity * RAD_TO_DEG * 10.0);
    memcpy(lighthouseData + 9, &data, sizeof(int16_t));
    tBleStatus ret = aci_gatt_update_char_value(
        motorControlServiceHandle,
        positionCharacteristicHandle,
        0,
        CHAR_POSITION_DATA_LENGTH,
        lighthouseData);
    return ret == BLE_STATUS_SUCCESS;
}

void Bluetooth::stop()
{
    // TODO: research how to properly deallocate all the bluetooth config info allocated in start()
    connectionHandle = 0;
    running = false;
}

/**
 * @brief The Bluetooth callback for handling further details of communication. It is important to note that
 *        the use of SerialUSB print functions appears to do nothing within the scope of this callback.
 * 
 * @param blePacket 
 */
void HCI_Event_CB(void *blePacket)
{
    hci_uart_pckt* hciPacket = (hci_uart_pckt*)blePacket;
    if (hciPacket->type != HCI_EVENT_PKT)
        return;

    hci_event_pckt* eventPacket = (hci_event_pckt*)hciPacket->data;
    switch (eventPacket->evt)
    {
        case EVT_LE_META_EVENT:
        {
          evt_le_meta_event* metaEvent = (evt_le_meta_event*)eventPacket->data;
          switch (metaEvent->subevent)
          {
              case EVT_LE_CONN_COMPLETE:
              {
                  // a host bluetooth device connected; disable discovery
                  aci_gap_set_non_discoverable();
                  // capture the connection handle
                  evt_le_connection_complete* cc = (evt_le_connection_complete*)metaEvent->data;
                  currentBluetooth->connectionHandle = cc->handle;
              }
              break;
          }
        }
        break;

        case EVT_VENDOR:
        {
            evt_blue_aci* blue_evt = (evt_blue_aci*)eventPacket->data;
            switch (blue_evt->ecode)
            {
                case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
                {
                    // the client sent data to a characteristic
                    evt_gatt_attr_modified_IDB05A1* evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
                    if (evt->attr_handle == currentBluetooth->motorControlCharacteristicHandle + 1)
                        currentBluetooth->packetReceived(evt->data_length, evt->att_data);
                }
                break;
            }
        }
        break;

        // the currently connected host device disconnected from us
        case EVT_DISCONN_COMPLETE:
        {
            // evt_disconn_complete *evt = (void *)event_pckt->data;
            currentBluetooth->connectionHandle = 0;


              // make the device discoverable again
            tBleStatus ret = currentBluetooth->enableDiscovery();
            if (ret != BLE_STATUS_SUCCESS)
            {
                //TODO: it's unclear what to do when we cannot re-enable discovery, so just stop Bluetooth altogether
                currentBluetooth->stop();
                currentBluetooth->errorReason = "Discovery";
            }
        }
        break;

        //remainder of event handling is for detection of errors occuring in the Bluetooth stack
        case EVT_HARDWARE_ERROR:
            currentBluetooth->stop();
            currentBluetooth->errorReason = "Hardware";
        break;

        case EVT_DATA_BUFFER_OVERFLOW:
            // _evt_data_buffer_overflow* overflowEvt = (_evt_data_buffer_overflow*)eventPacket->data;
            currentBluetooth->stop();
            currentBluetooth->errorReason = "Overflow";
        break;

    }
}
