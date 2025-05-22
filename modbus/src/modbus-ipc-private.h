#ifndef MODBUS_IPC_PRIVATE_H
#define MODBUS_IPC_PRIVATE_H

#include <stdint.h>

#define _MODBUS_IPC_HEADER_LENGTH     1
#define _MODBUS_IPC_PRESET_REQ_LENGTH 6
#define _MODBUS_IPC_PRESET_RSP_LENGTH 2
#define _MODBUS_IPC_CHECKSUM_LENGTH   2

typedef struct _modbus_ipc{
    char* deviceR, * deviceS;
    int confirmation_to_ignore;
}modbus_ipc_t;

#endif