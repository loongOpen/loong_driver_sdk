#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "modbus-private.h"
#include <assert.h>
#include "modbus-ipc-private.h"
#include "modbus-ipc.h"

static uint8_t const table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40};

static uint8_t const table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5,
    0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B,
    0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
    0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
    0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8,
    0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
    0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A,
    0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
    0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51,
    0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D,
    0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40};

static int _modbus_set_slave(modbus_t* ctx, int slave){
    int max_slave = (ctx->quirks & MODBUS_QUIRK_MAX_SLAVE) ? 255 : 247;
    if(slave >= 0 && slave <= max_slave){
        ctx->slave = slave;
    }else{
        errno = EINVAL;
        return -1;
    }
    return 0;
}

static int _modbus_ipc_build_request_basis(modbus_t* ctx, int function, int addr, int nb, uint8_t* req){
    assert(ctx->slave != -1);
    req[0] = ctx->slave;
    req[1] = function;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;
    return _MODBUS_IPC_PRESET_REQ_LENGTH;
}

static int _modbus_ipc_build_response_basis(sft_t* sft, uint8_t* rsp){
    rsp[0] = sft->slave;
    rsp[1] = sft->function;
    return _MODBUS_IPC_PRESET_RSP_LENGTH;
}

static uint16_t crc16(uint8_t* buffer, uint16_t buffer_length){
    uint8_t crc_hi = 0xff;
    uint8_t crc_lo = 0xff;
    unsigned int i;
    while(buffer_length--){
        i = crc_lo ^ *buffer++;
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }
    return crc_hi << 8 | crc_lo;
}

static int _modbus_ipc_get_response_tid(uint8_t const* req){
    return 0;
}

static int _modbus_ipc_send_msg_pre(uint8_t* req, int req_length){
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc & 0x00ff;
    req[req_length++] = crc >> 8;
    return req_length;
}

static ssize_t _modbus_ipc_send(modbus_t* ctx, uint8_t const* req, int req_length){
    return write(ctx->s, req, req_length);
}

static int _modbus_ipc_receive(modbus_t* ctx, uint8_t* req){
    int rc;
    modbus_ipc_t* ctx_ipc = ctx->backend_data;
    if(ctx_ipc->confirmation_to_ignore){
        _modbus_receive_msg(ctx, req, MSG_CONFIRMATION);
        ctx_ipc->confirmation_to_ignore = FALSE;
        rc = 0;
        if(ctx->debug){
            printf("Confirmation to ignore\n");
        }
    }else{
        rc = _modbus_receive_msg(ctx, req, MSG_INDICATION);
        if(rc == 0){
            ctx_ipc->confirmation_to_ignore = TRUE;
        }
    }
    return rc;
}

static ssize_t _modbus_ipc_recv(modbus_t* ctx, uint8_t* rsp, int rsp_length){
    return read(ctx->r, rsp, rsp_length);
}

static int _modbus_ipc_flush(modbus_t* ctx);

static int _modbus_ipc_pre_check_confirmation(modbus_t* ctx, uint8_t const* req, uint8_t const* rsp, int rsp_length){
    if(req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS){
        if(ctx->debug){
            fprintf(stderr, "The responding slave %d isn't the requested slave %d\n", rsp[0], req[0]);
        }
        errno = EMBBADSLAVE;
        return -1;
    }else{
        return 0;
    }
}

static int _modbus_ipc_check_integrity(modbus_t* ctx, uint8_t* msg, int const msg_length){
    uint16_t crc_calculated;
    uint16_t crc_received;
    int slave = msg[0];
    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 1] << 8) | msg[msg_length - 2];
    if(crc_calculated != crc_received){
        if(ctx->debug){
            fprintf(stderr, "ERROR CRC received 0x%0X != CRC calculated 0x%0X\n", crc_received, crc_calculated);
        }
        if(ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL){
            _modbus_ipc_flush(ctx);
        }
        errno = EMBBADCRC;
        return -1;
    }
    if(slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS){
        if(ctx->debug){
            printf("Request for slave %d ignored (not %d)\n", slave, ctx->slave);
        }
        return 0;
    }
    return msg_length;
}

static int _modbus_ipc_connect(modbus_t* ctx){
    int flags;
    modbus_ipc_t* ctx_ipc = ctx->backend_data;
    if(ctx->debug){
        printf("Opening %s and %s\n", ctx_ipc->deviceR, ctx_ipc->deviceS);
    }
    flags = O_RDONLY | O_NDELAY | O_CLOEXEC;
    ctx->r = open(ctx_ipc->deviceR, flags);
    if(ctx->r < 0){
        if(ctx->debug){
            fprintf(stderr, "ERROR Can't open the device %s (%s)\n", ctx_ipc->deviceR, strerror(errno));
        }
        return -1;
    }
    flags = O_WRONLY | O_CLOEXEC;
    ctx->s = open(ctx_ipc->deviceS, flags);
    if(ctx->s < 0){
        if(ctx->debug){
            fprintf(stderr, "ERROR Can't open the device %s (%s)\n", ctx_ipc->deviceS, strerror(errno));
        }
        return -1;
    }
    return 0;
}

static unsigned int _modbus_ipc_is_connected(modbus_t* ctx){
    return ctx->r >= 0 && ctx->s >= 0;
}

static void _modbus_ipc_close(modbus_t* ctx){
    if(ctx->r >= 0){
        close(ctx->r);
        ctx->r = -1;
    }
    if(ctx->s >= 0){
        close(ctx->s);
        ctx->s = -1;
    }
}

static int _modbus_ipc_flush(modbus_t* ctx){
    return 0;
}

static int _modbus_ipc_select(modbus_t* ctx, fd_set* rset, struct timeval* tv, int length_to_read){
    int s_rc;
    while((s_rc = select(ctx->r + 1, rset, NULL, NULL, tv)) == -1){
        if(errno == EINTR){
            if(ctx->debug){
                fprintf(stderr, "A non blocked signal was caught\n");
            }
            FD_ZERO(rset);
            FD_SET(ctx->r, rset);
        }else{
            return -1;
        }
    }
    if(s_rc == 0){
        errno = ETIMEDOUT;
        return -1;
    }
    return s_rc;
}

static void _modbus_ipc_free(modbus_t* ctx){
    if(ctx->backend_data){
        free(((modbus_ipc_t*)ctx->backend_data)->deviceR);
        free(((modbus_ipc_t*)ctx->backend_data)->deviceS);
        free(ctx->backend_data);
    }
    free(ctx);
}

modbus_backend_t const _modbus_ipc_backend = {
    _MODBUS_BACKEND_TYPE_IPC,
    _MODBUS_IPC_HEADER_LENGTH,
    _MODBUS_IPC_CHECKSUM_LENGTH,
    MODBUS_IPC_MAX_ADU_LENGTH,
    _modbus_set_slave,
    _modbus_ipc_build_request_basis,
    _modbus_ipc_build_response_basis,
    _modbus_ipc_get_response_tid,
    _modbus_ipc_send_msg_pre,
    _modbus_ipc_send,
    _modbus_ipc_receive,
    _modbus_ipc_recv,
    _modbus_ipc_check_integrity,
    _modbus_ipc_pre_check_confirmation,
    _modbus_ipc_connect,
    _modbus_ipc_is_connected,
    _modbus_ipc_close,
    _modbus_ipc_flush,
    _modbus_ipc_select,
    _modbus_ipc_free
};

modbus_t* modbus_new_ipc(char const* deviceR, char const* deviceS){
    modbus_t* ctx;
    modbus_ipc_t* ctx_ipc;
    ctx = (modbus_t*)malloc(sizeof(modbus_t));
    if(ctx == NULL){
        return NULL;
    }
    _modbus_init_common(ctx);
    ctx->backend = &_modbus_ipc_backend;
    ctx->backend_data = (modbus_ipc_t*)malloc(sizeof(modbus_ipc_t));
    if(ctx->backend_data == NULL){
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }
    ctx_ipc = (modbus_ipc_t*)ctx->backend_data;
    ctx_ipc->deviceR = (char*)malloc((strlen(deviceR) + 1) * sizeof(char));
    if(ctx_ipc->deviceR == NULL){
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }
    strcpy(ctx_ipc->deviceR, deviceR);
    ctx_ipc->deviceS = (char*)malloc((strlen(deviceS) + 1) * sizeof(char));
    if(ctx_ipc->deviceS == NULL){
        free(ctx_ipc->deviceR);
        modbus_free(ctx);
        errno = ENOMEM;
        return NULL;
    }
    strcpy(ctx_ipc->deviceS, deviceS);
    ctx_ipc->confirmation_to_ignore = FALSE;
    return ctx;
}