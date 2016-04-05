
#include "rn4020.h"
#include <utils/utils.h>
#include <utils/time.h>
#include <utils/ringbufferdma.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef RN4020_DEBUG
#define RN4020_DEBUG_OUT(format, ...) printf("%s:%d: RN4020: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define RN4020_DEBUG_OUT(format, ...)
#endif

#define RN4020_MAX_RX_LINE_LENGTH 100

void _RN4020_processLine(RN4020* rn4020, const char* line);
void _RN4020_setState(RN4020* rn4020, RN4020_State newState);
void _RN4020_setConnected(RN4020* rn4020, bool connected);
HAL_StatusTypeDef _RN4020_waitForReadyState(RN4020* rn4020);
HAL_StatusTypeDef _RN4020_runAOKCommand(RN4020* rn4020, const char* cmd);
void _RN4020_parseUUIDString(const char* str, uint8_t strLen, uint8_t* uuid, uint8_t* uuidLen);
bool _RN4020_parseHandleUUIDLine(const char* line, RN4020_handleLookupItem* handleLookupItem);
HAL_StatusTypeDef _RN4020_writeServerCharacteristic(RN4020* rn4020, const uint8_t* uuid, uint8_t uuidLen, const uint8_t* data, uint32_t dataLength);

HAL_StatusTypeDef RN4020_setup(RN4020* rn4020) {
  RingBufferDmaU8_initUSARTRx(&rn4020->rxRing, rn4020->uart, rn4020->rxBuffer, RN4020_RX_BUFFER_SIZE);
  rn4020->connected = false;
  rn4020->handleLookupLength = 0;
  _RN4020_setState(rn4020, RN4020_STATE_INITIALIZING);

  HAL_GPIO_WritePin(rn4020->wakeswPort, rn4020->wakeswPin, GPIO_PIN_RESET);
  sleep_ms(100);
  HAL_GPIO_WritePin(rn4020->wakehwPort, rn4020->wakehwPin, GPIO_PIN_RESET);
  sleep_ms(500);

  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_CMD);
  HAL_GPIO_WritePin(rn4020->wakehwPort, rn4020->wakehwPin, GPIO_PIN_SET);
  sleep_ms(100);
  HAL_GPIO_WritePin(rn4020->wakeswPort, rn4020->wakeswPin, GPIO_PIN_SET);
  returnNonOKHALStatus(_RN4020_waitForReadyState(rn4020));

  return HAL_OK;
}

void RN4020_tick(RN4020* rn4020) {
  char line[RN4020_MAX_RX_LINE_LENGTH];
  if (RingBufferDmaU8_readLine(&rn4020->rxRing, line, sizeof(line)) > 0) {
    strTrimRight(line);
    if (strlen(line) > 0) {
      _RN4020_processLine(rn4020, line);
    }
  }
}

bool RN4020_isConnected(RN4020* rn4020) {
  return rn4020->connected;
}

HAL_StatusTypeDef RN4020_resetToFactoryDefaults(RN4020* rn4020) {
  return _RN4020_runAOKCommand(rn4020, "SF,1");
}

HAL_StatusTypeDef RN4020_setSupportedServices(RN4020* rn4020, uint32_t services) {
  char line[15];
  sprintf(line, "SS,%08lX", services);
  return _RN4020_runAOKCommand(rn4020, line);
}

HAL_StatusTypeDef RN4020_setSupportedFeatures(RN4020* rn4020, uint32_t features) {
  char line[15];
  sprintf(line, "SR,%08lX", features);
  return _RN4020_runAOKCommand(rn4020, line);
}

HAL_StatusTypeDef RN4020_setDeviceNameWithMAC(RN4020* rn4020, const char* deviceName) {
  char line[15];
  sprintf(line, "S-,%s", deviceName);
  return _RN4020_runAOKCommand(rn4020, line);
}

HAL_StatusTypeDef RN4020_setDeviceName(RN4020* rn4020, const char* deviceName) {
  char line[15];
  sprintf(line, "SN,%s", deviceName);
  return _RN4020_runAOKCommand(rn4020, line);
}

HAL_StatusTypeDef RN4020_reset(RN4020* rn4020) {
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_RESET);
  RN4020_send(rn4020, "R,1");
  return _RN4020_waitForReadyState(rn4020);
}

HAL_StatusTypeDef RN4020_advertise(RN4020* rn4020) {
  return _RN4020_runAOKCommand(rn4020, "A");
}

HAL_StatusTypeDef RN4020_removeBond(RN4020* rn4020) {
  return _RN4020_runAOKCommand(rn4020, "U");
}

HAL_StatusTypeDef RN4020_refreshHandleLookup(RN4020* rn4020) {
  rn4020->handleLookupLength = 0;
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_LS);
  RN4020_send(rn4020, "LS");
  return _RN4020_waitForReadyState(rn4020);
}

HAL_StatusTypeDef RN4020_clearPrivate(RN4020* rn4020) {
  return _RN4020_runAOKCommand(rn4020, "PZ");
}

HAL_StatusTypeDef RN4020_addPrivateService(RN4020* rn4020, const uint8_t* uuid) {
  char line[50];
  strcpy(line, "PS,");
  RN4020_uuidToString(line + 3, uuid, RN4020_PRIVATE_UUID_LENGTH_BYTES);
  return _RN4020_runAOKCommand(rn4020, line);
}

HAL_StatusTypeDef RN4020_addPrivateCharacteristic(
  RN4020* rn4020,
  const uint8_t* uuid,
  uint8_t propertyOptions,
  uint8_t size,
  uint8_t securityOptions
) {
  char line[50];
  char* dest;
  strcpy(line, "PC,");
  RN4020_uuidToString(line + 3, uuid, RN4020_PRIVATE_UUID_LENGTH_BYTES);
  dest = line + strlen(line);
  sprintf(dest, ",%02X,%02X", propertyOptions, size);
  if (securityOptions != RN4020_PRIVATE_CHARACTERISTIC_SECURITY_NONE) {
    dest = line + strlen(line);
    sprintf(dest, ",%02X", securityOptions);
  }
  return _RN4020_runAOKCommand(rn4020, line);
}

void RN4020_uuidToString(char* dest, const uint8_t* uuid, uint8_t uuidLength) {
  for (int i = 0; i < uuidLength; i++) {
    sprintf(dest, "%02X", uuid[i]);
    dest += 2;
  }
}

RN4020_handleLookupItem* RN4020_lookupHandle(RN4020* rn4020, uint16_t handle) {
  for (int i = 0; i < rn4020->handleLookupLength; i++) {
    RN4020_handleLookupItem* item = &rn4020->handleLookup[i];
    if (item->handle == handle) {
      return item;
    }
  }
  return NULL;
}

bool RN4020_isHandleLookupItemUUIDEqual16(RN4020_handleLookupItem* handleLookupItem, uint16_t uuid) {
  if (handleLookupItem->characteristicUUIDLength != 2) {
    return false;
  }
  uint16_t lookupItemUUID = ((uint16_t)handleLookupItem->characteristicUUID[0] << 8) | handleLookupItem->characteristicUUID[1];
  return lookupItemUUID == uuid;
}

bool RN4020_isHandleLookupItemUUIDEqual128(RN4020_handleLookupItem* handleLookupItem, const uint8_t* uuid) {
  if (handleLookupItem->characteristicUUIDLength != RN4020_PRIVATE_UUID_LENGTH_BYTES) {
    return false;
  }
  for (int i = 0; i < RN4020_PRIVATE_UUID_LENGTH_BYTES; i++) {
    if (uuid[i] != handleLookupItem->characteristicUUID[i]) {
      return false;
    }
  }
  return true;
}

void _RN4020_processLine(RN4020* rn4020, const char* line) {
  RN4020_DEBUG_OUT("rx: %s\n", line);

  if (strcmp(line, "Connected") == 0) {
    _RN4020_setConnected(rn4020, true);
    return;
  }

  if (strcmp(line, "Connection End") == 0) {
    _RN4020_setConnected(rn4020, false);
    return;
  }

  if (strncmp(line, "RV,", 3) == 0) {
    char handleStr[5];
    strncpy(handleStr, line + 3, 4);
    handleStr[4] = '\0';
    uint16_t handle = strtol(handleStr, NULL, 16);
    RN4020_onRealTimeRead(rn4020, handle);
    return;
  }

  if (strncmp(line, "WV,", 3) == 0) {
    char handleStr[5];
    strncpy(handleStr, line + 3, 4);
    handleStr[4] = '\0';
    uint16_t handle = strtol(handleStr, NULL, 16);
    const char* dataStringPtr = line + 8;
    int dataLength = (strlen(dataStringPtr) - 1) / 2; // WV ends with period so subtract 1
    uint8_t data[RN4020_MAX_RX_LINE_LENGTH];
    for (int i = 0; i < dataLength; i++, dataStringPtr += 2) {
      char temp[3];
      strncpy(temp, dataStringPtr, 2);
      temp[2] = '\0';
      data[i] = strtol(temp, NULL, 16);
    }
    RN4020_onWrite(rn4020, handle, data, dataLength);
    return;
  }

  switch (rn4020->state) {
  case RN4020_STATE_INITIALIZING:
    break;

  case RN4020_STATE_WAITING_FOR_CMD:
    if (strcmp(line, "CMD") == 0) {
      _RN4020_setState(rn4020, RN4020_STATE_READY);
      return;
    }
    break;

  case RN4020_STATE_WAITING_FOR_AOK:
    if (strcmp(line, "AOK") == 0) {
      _RN4020_setState(rn4020, RN4020_STATE_READY);
      return;
    }
    break;

  case RN4020_STATE_WAITING_FOR_RESET:
    if (strcmp(line, "Reboot") == 0) {
      _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_CMD);
      return;
    }
    break;

  case RN4020_STATE_WAITING_FOR_LS:
    if (_RN4020_parseHandleUUIDLine(line, &rn4020->handleLookup[rn4020->handleLookupLength])) {
      rn4020->handleLookupLength++;
      return;
    } else if (strcmp(line, "END") == 0) {
      _RN4020_setState(rn4020, RN4020_STATE_READY);
      return;
    } else if (strlen(line) == 4 || strlen(line) == RN4020_PRIVATE_UUID_HEX_STRING_LENGTH) {
      // service uuid
      return;
    }
    break;

  case RN4020_STATE_READY:
    break;
  }
  RN4020_DEBUG_OUT("unexpected line: %s\n", line);
}

bool _RN4020_parseHandleUUIDLine(const char* line, RN4020_handleLookupItem* handleLookupItem) {
  if (strncmp(line, "  ", 2) != 0) {
    return false;
  }

  const char* startOfUUIDPtr = line + 2;
  const char* firstCommaPtr = strchr(startOfUUIDPtr, ',');
  _RN4020_parseUUIDString(startOfUUIDPtr, firstCommaPtr - startOfUUIDPtr, handleLookupItem->characteristicUUID, &handleLookupItem->characteristicUUIDLength);

  char handleStr[5];
  strncpy(handleStr, firstCommaPtr + 1, 4);
  handleStr[4] = 0;
  handleLookupItem->handle = strtol(handleStr, NULL, 16);

  return true;
}

void _RN4020_setConnected(RN4020* rn4020, bool connected) {
  rn4020->connected = connected;
  RN4020_connectedStateChanged(rn4020, connected);
}

__weak void RN4020_connectedStateChanged(RN4020* rn4020, bool connected) {
  RN4020_DEBUG_OUT("%s\n", connected ? "connected" : "disconnected");
}

__weak void RN4020_onRealTimeRead(RN4020* rn4020, uint16_t characteristicHandle) {
  RN4020_DEBUG_OUT("real time read: 0x%04X\n", characteristicHandle);
}

__weak void RN4020_onWrite(RN4020* rn4020, uint16_t characteristicHandle, uint8_t* data, uint8_t dataLength) {
#ifdef RN4020_DEBUG
  RN4020_DEBUG_OUT("write: 0x%04X: ", characteristicHandle);
  for (int i = 0; i < dataLength; i++) {
    printf("%02X", data[i]);
  }
  printf("\n");
#endif
}

HAL_StatusTypeDef _RN4020_runAOKCommand(RN4020* rn4020, const char* cmd) {
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_AOK);
  RN4020_send(rn4020, cmd);
  return _RN4020_waitForReadyState(rn4020);
}

HAL_StatusTypeDef _RN4020_waitForReadyState(RN4020* rn4020) {
  if (rn4020->state == RN4020_STATE_READY) {
    return HAL_OK;
  }

  uint32_t startTime = HAL_GetTick();
  while (1) {
    if (rn4020->state == RN4020_STATE_READY) {
      return HAL_OK;
    }
    if ((HAL_GetTick() - startTime) > RN4020_TIMEOUT) {
      return HAL_TIMEOUT;
    }
    RN4020_tick(rn4020);
  }
}

void _RN4020_setState(RN4020* rn4020, RN4020_State newState) {
#ifdef RN4020_DEBUG
  switch (newState) {
  case RN4020_STATE_INITIALIZING:
    RN4020_DEBUG_OUT("state: INITIALIZING\n");
    break;
  case RN4020_STATE_READY:
    RN4020_DEBUG_OUT("state: READY\n");
    break;
  case RN4020_STATE_WAITING_FOR_CMD:
    RN4020_DEBUG_OUT("state: WAITING_FOR_CMD\n");
    break;
  case RN4020_STATE_WAITING_FOR_AOK:
    RN4020_DEBUG_OUT("state: WAITING_FOR_AOK\n");
    break;
  case RN4020_STATE_WAITING_FOR_RESET:
    RN4020_DEBUG_OUT("state: WAITING_FOR_RESET\n");
    break;
  case RN4020_STATE_WAITING_FOR_LS:
    RN4020_DEBUG_OUT("state: WAITING_FOR_LS\n");
    break;
  }
#endif
  rn4020->state = newState;
}

void _RN4020_parseUUIDString(const char* str, uint8_t strLen, uint8_t* uuid, uint8_t* uuidLen) {
  char temp[3];
  uint8_t strIndex, destIndex;
  for (strIndex = 0, destIndex = 0; strIndex < strLen; strIndex += 2) {
    temp[0] = str[strIndex];
    temp[1] = str[strIndex + 1];
    temp[2] = '\0';
    uuid[destIndex++] = strtol(temp, NULL, 16);
  }
  *uuidLen = destIndex;
}

void RN4020_send(RN4020* rn4020, const char* line) {
  char newLineCh = '\n';
  RN4020_DEBUG_OUT("tx: %s\n", line);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, strlen(line), RN4020_TIMEOUT);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)&newLineCh, 1, RN4020_TIMEOUT);
}

HAL_StatusTypeDef RN4020_writeServerPublicCharacteristic(RN4020* rn4020, uint16_t uuid, const uint8_t* data, uint32_t dataLength) {
  uint8_t uuidBytes[2];
  uuidBytes[0] = (uuid >> 8) & 0xff;
  uuidBytes[1] = (uuid >> 0) & 0xff;
  return _RN4020_writeServerCharacteristic(rn4020, uuidBytes, 2, data, dataLength);
}

HAL_StatusTypeDef RN4020_writeServerPrivateCharacteristic(RN4020* rn4020, const uint8_t* uuid, const uint8_t* data, uint32_t dataLength) {
  return _RN4020_writeServerCharacteristic(rn4020, uuid, RN4020_PRIVATE_UUID_LENGTH_BYTES, data, dataLength);
}

HAL_StatusTypeDef _RN4020_writeServerCharacteristic(RN4020* rn4020, const uint8_t* uuid, uint8_t uuidLen, const uint8_t* data, uint32_t dataLength) {
  char newLineCh = '\n';
  char uuidStr[RN4020_PRIVATE_UUID_HEX_STRING_LENGTH + 1];
  char line[20];
  RN4020_uuidToString(uuidStr, uuid, uuidLen);
#ifdef RN4020_DEBUG
  RN4020_DEBUG_OUT("tx: SUW,%s,", uuidStr);
  for (int i = 0; i < dataLength; i++) {
    printf("%02X", data[i]);
  }
  printf("\n");
#endif
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_AOK);
  sprintf(line, "SUW,");
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, strlen(line), RN4020_TIMEOUT);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)uuidStr, strlen(uuidStr), RN4020_TIMEOUT);
  strcpy(line, ",");
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, strlen(line), RN4020_TIMEOUT);
  for (uint32_t i = 0; i < dataLength; i++) {
    sprintf(line, "%02X", data[i]);
    HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, 2, RN4020_TIMEOUT);
  }
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)&newLineCh, 1, RN4020_TIMEOUT);
  return _RN4020_waitForReadyState(rn4020);
}

HAL_StatusTypeDef RN4020_writeServerCharacteristicHandle(RN4020* rn4020, uint16_t handle, const uint8_t* data, uint32_t dataLength) {
  char newLineCh = '\n';
  char line[20];
#ifdef RN4020_DEBUG
  RN4020_DEBUG_OUT("tx: SHW,%04X,", handle);
  for (int i = 0; i < dataLength; i++) {
    printf("%02X", data[i]);
  }
  printf("\n");
#endif
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_AOK);
  sprintf(line, "SHW,%04X,", handle);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, strlen(line), RN4020_TIMEOUT);
  for (uint32_t i = 0; i < dataLength; i++) {
    sprintf(line, "%02X", data[i]);
    HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, 2, RN4020_TIMEOUT);
  }
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)&newLineCh, 1, RN4020_TIMEOUT);
  return _RN4020_waitForReadyState(rn4020);
}

HAL_StatusTypeDef RN4020_battery_setLevel(RN4020* rn4020, uint8_t level) {
  return RN4020_writeServerPublicCharacteristic(rn4020, RN4020_BATTERY_LEVEL_UUID, &level, 1);
}

