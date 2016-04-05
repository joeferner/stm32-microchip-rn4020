
#ifndef _RN4020_H_
#define _RN4020_H_

#include <platform_config.h>
#include <utils/ringbufferdma.h>

#ifndef RN4020_TIMEOUT
#define RN4020_TIMEOUT 5000
#endif

#ifndef RN4020_LOOKUP_TABLE_SIZE
#define RN4020_LOOKUP_TABLE_SIZE 20
#endif

#define RN4020_RX_BUFFER_SIZE 500

#define RN4020_PRIVATE_UUID_LENGTH_BITS       128
#define RN4020_PRIVATE_UUID_LENGTH_BYTES      (128 / 8)
#define RN4020_PRIVATE_UUID_HEX_STRING_LENGTH (RN4020_PRIVATE_UUID_LENGTH_BYTES * 2)
#define RN4020_MAX_UUID_LEN_BYTES             (128 / 8)

#define RN4020_SERVICE_DEVICE_INFORMATION    0x80000000
#define RN4020_SERVICE_BATTERY               0x40000000
#define RN4020_SERVICE_HEART_RATE            0x20000000
#define RN4020_SERVICE_HEALTH_THERMOMETER    0x10000000
#define RN4020_SERVICE_GLUCOSE               0x08000000
#define RN4020_SERVICE_BLOOD_PRESSURE        0x04000000
#define RN4020_SERVICE_RUNNING_SPEED_CADENCE 0x02000000
#define RN4020_SERVICE_CYCLING_SPEED_CADENCE 0x01000000
#define RN4020_SERVICE_CURRENT_TIME          0x00800000
#define RN4020_SERVICE_NEXT_DST_CHANGE       0x00400000
#define RN4020_SERVICE_REFERENCE_TIME_UPDATE 0x00200000
#define RN4020_SERVICE_LINK_LOSS             0x00100000
#define RN4020_SERVICE_IMMEDIATE_ALERT       0x00080000
#define RN4020_SERVICE_TX_POWER              0x00040000
#define RN4020_SERVICE_ALERT_NOTIFICATION    0x00020000
#define RN4020_SERVICE_PHONE_ALERT_STATUS    0x00010000
#define RN4020_SERVICE_SCAN_PARAMETERS       0x00004000
#define RN4020_SERVICE_USER_DEFINED          0x00000001

#define RN4020_FEATURE_CENTRAL             0x80000000
#define RN4020_FEATURE_REAL_TIME_READ      0x40000000
#define RN4020_FEATURE_AUTO_ADVERTISE      0x20000000
#define RN4020_FEATURE_ENABLE_MLDP         0x10000000
#define RN4020_FEATURE_AUTO_MLDP_DISABLE   0x08000000
#define RN4020_FEATURE_NO_DIRECT_ADVERTISE 0x04000000
#define RN4020_FEATURE_UART_FLOW_CONTROL   0x02000000
#define RN4020_FEATURE_RUN_SCRIPT_PWR_ON   0x01000000
#define RN4020_FEATURE_ENABLE_AUTH         0x00400000
#define RN4020_FEATURE_ENABLE_REMOTE_CMD   0x00200000
#define RN4020_FEATURE_DO_NOT_SAVE_BONDING 0x00100000
#define RN4020_FEATURE_IO_CAP              0x00E00000
#define RN4020_FEATURE_BLOCK_SET_CMD       0x00010000
#define RN4020_FEATURE_ENABLE_OTA          0x00008000
#define RN4020_FEATURE_IOS_MODE            0x00004000
#define RN4020_FEATURE_SERVER_ONLY         0x00002000
#define RN4020_FEATURE_ENABLE_UART_SCRIPT  0x00001000
#define RN4020_FEATURE_AUTO_MLDP           0x00000800
#define RN4020_FEATURE_MLDP_WITHOUT_STATUS 0x00000400

#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_EXTENDED               0b10000000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_AUTH_WRITE             0b01000000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_INDICATE               0b00100000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_NOTIFY                 0b00010000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_WRITE                  0b00001000
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_WRITE_WITHOUT_RESPONSE 0b00000100
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_READ                   0b00000010
#define RN4020_PRIVATE_CHARACTERISTIC_PROPERTY_BROADCAST              0b00000001

#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_NONE     0b00000000
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_ENCR_R   0b00000001
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_AUTH_R   0b00000010
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_ENCR_W   0b00010000
#define RN4020_PRIVATE_CHARACTERISTIC_SECURITY_AUTH_W   0b00100000

#define RN4020_DEV_INFO_MANUFACTURER_NAME  0x2a29
#define RN4020_DEV_INFO_MODEL_NUMBER       0x2a24
#define RN4020_DEV_INFO_SERIAL_NUMBER      0x2a25
#define RN4020_DEV_INFO_HARDWARE_REVISION  0x2a27
#define RN4020_DEV_INFO_FIRMWARE_REVISION  0x2a26
#define RN4020_DEV_INFO_SOFTWARE_REVISION  0x2a28
#define RN4020_DEV_INFO_SYSTEM_ID          0x2a23
#define RN4020_DEV_INFO_REG_CERT_DATA      0x2a2a

#define RN4020_BATTERY_LEVEL_UUID  0x2a19
#define RN4020_BATTERY_MAX_LEVEL   0x64

typedef enum {
  RN4020_STATE_INITIALIZING,
  RN4020_STATE_READY,
  RN4020_STATE_WAITING_FOR_CMD,
  RN4020_STATE_WAITING_FOR_AOK,
  RN4020_STATE_WAITING_FOR_RESET,
  RN4020_STATE_WAITING_FOR_LS
} RN4020_State;

typedef struct {
  uint16_t handle;
  uint8_t characteristicUUID[RN4020_MAX_UUID_LEN_BYTES];
  uint8_t characteristicUUIDLength;
} RN4020_handleLookupItem;

typedef struct {
  UART_HandleTypeDef* uart;
  GPIO_TypeDef* wakeswPort;
  uint16_t wakeswPin;
  GPIO_TypeDef* wakehwPort;
  uint16_t wakehwPin;

  volatile RN4020_State state;
  volatile bool connected;
  RingBufferDmaU8 rxRing;
  uint8_t rxBuffer[RN4020_RX_BUFFER_SIZE];

  RN4020_handleLookupItem handleLookup[RN4020_LOOKUP_TABLE_SIZE];
  uint32_t handleLookupLength;
} RN4020;

__weak void RN4020_onRealTimeRead(RN4020* rn4020, uint16_t characteristicHandle);
__weak void RN4020_onWrite(RN4020* rn4020, uint16_t characteristicHandle, uint8_t* data, uint8_t dataLength);
__weak void RN4020_connectedStateChanged(RN4020* rn4020, bool connected);

HAL_StatusTypeDef RN4020_setup(RN4020* rn4020);
HAL_StatusTypeDef RN4020_resetToFactoryDefaults(RN4020* rn4020);
HAL_StatusTypeDef RN4020_setSupportedServices(RN4020* rn4020, uint32_t services);
HAL_StatusTypeDef RN4020_setSupportedFeatures(RN4020* rn4020, uint32_t features);
HAL_StatusTypeDef RN4020_reset(RN4020* rn4020);
HAL_StatusTypeDef RN4020_setDeviceNameWithMAC(RN4020* rn4020, const char* deviceName);
HAL_StatusTypeDef RN4020_setDeviceName(RN4020* rn4020, const char* deviceName);
HAL_StatusTypeDef RN4020_advertise(RN4020* rn4020);
HAL_StatusTypeDef RN4020_removeBond(RN4020* rn4020);
HAL_StatusTypeDef RN4020_refreshHandleLookup(RN4020* rn4020);
HAL_StatusTypeDef RN4020_clearPrivate(RN4020* rn4020);
bool RN4020_isConnected(RN4020* rn4020);
HAL_StatusTypeDef RN4020_addPrivateService(RN4020* rn4020, const uint8_t* uuid);
HAL_StatusTypeDef RN4020_addPrivateCharacteristic(
  RN4020* rn4020,
  const uint8_t* uuid,
  uint8_t propertyOptions,
  uint8_t size,
  uint8_t securityOptions
);
void RN4020_uuidToString(char* dest, const uint8_t* uuid, uint8_t uuidLength);
RN4020_handleLookupItem* RN4020_lookupHandle(RN4020* rn4020, uint16_t handle);
bool RN4020_isHandleLookupItemUUIDEqual16(RN4020_handleLookupItem* handleLookupItem, uint16_t uuid);
bool RN4020_isHandleLookupItemUUIDEqual128(RN4020_handleLookupItem* handleLookupItem, const uint8_t* uuid);
void RN4020_tick(RN4020* rn4020);
void RN4020_send(RN4020* rn4020, const char* line);

HAL_StatusTypeDef RN4020_writeServerPublicCharacteristic(RN4020* rn4020, uint16_t uuid, const uint8_t* data, uint32_t dataLength);
HAL_StatusTypeDef RN4020_writeServerPrivateCharacteristic(RN4020* rn4020, const uint8_t* uuid, const uint8_t* data, uint32_t dataLength);
HAL_StatusTypeDef RN4020_writeServerCharacteristicHandle(RN4020* rn4020, uint16_t handle, const uint8_t* data, uint32_t dataLength);

/**
 * level 0x00 (0%) - 0x64 (100%)
 */
HAL_StatusTypeDef RN4020_battery_setLevel(RN4020* rn4020, uint8_t level);

#endif
