
#include "rn4020.h"
#include <utils/utils.h>
#include <utils/time.h>
#include <utils/ringbufferdma.h>
#include <stdio.h>
#include <string.h>

#ifdef RN4020_DEBUG
#define RN4020_DEBUG_OUT(format, ...) printf("%s:%d: RN4020: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define RN4020_DEBUG_OUT(format, ...)
#endif

void _RN4020_processLine(RN4020* rn4020, const char* line);
void _RN4020_setState(RN4020* rn4020, RN4020_State newState);
void _RN4020_setConnected(RN4020* rn4020, bool connected);
__weak void RN4020_connectedStateChanged(RN4020* rn4020, bool connected);
HAL_StatusTypeDef _RN4020_waitForReadyState(RN4020* rn4020);
HAL_StatusTypeDef _RN4020_runAOKCommand(RN4020* rn4020, const char* cmd);

HAL_StatusTypeDef RN4020_setup(RN4020* rn4020) {
  RingBufferDmaU8_initUSARTRx(&rn4020->rxRing, rn4020->uart, rn4020->rxBuffer, RN4020_RX_BUFFER_SIZE);
  rn4020->connected = false;
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
  char line[30];
  if (RingBufferDmaU8_readLine(&rn4020->rxRing, line, sizeof(line)) > 0) {
    strTrimRight(line);
    if (strlen(line) > 0) {
      _RN4020_processLine(rn4020, line);
    }
  }
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
  case RN4020_STATE_READY:
    break;
  }
  RN4020_DEBUG_OUT("unexpected line: %s\n", line);
}

void _RN4020_setConnected(RN4020* rn4020, bool connected) {
  rn4020->connected = connected;
  RN4020_connectedStateChanged(rn4020, connected);
}

__weak void RN4020_connectedStateChanged(RN4020* rn4020, bool connected) {
  RN4020_DEBUG_OUT("%s\n", connected ? "connected" : "disconnected");
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
  }
#endif
  rn4020->state = newState;
}

void RN4020_send(RN4020* rn4020, const char* line) {
  char newLineCh = '\n';
  RN4020_DEBUG_OUT("tx: %s\n", line);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)line, strlen(line), RN4020_TIMEOUT);
  HAL_UART_Transmit(rn4020->uart, (uint8_t*)&newLineCh, 1, RN4020_TIMEOUT);
}

HAL_StatusTypeDef RN4020_writeServerPublicCharacteristic(RN4020* rn4020, uint16_t uuid, uint8_t* data, uint32_t dataLength) {
  char newLineCh = '\n';
  char line[15];
  RN4020_DEBUG_OUT("tx: SUW,%04X,%02X...\n", uuid, data[0]);
  _RN4020_setState(rn4020, RN4020_STATE_WAITING_FOR_AOK);
  sprintf(line, "SUW,%04X,", uuid);
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
