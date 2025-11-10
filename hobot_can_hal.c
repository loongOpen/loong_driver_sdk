#include "hobot_can_hal.h"

int canInit(){
    return 0;
}

void canDeInit(){
}

int canRecvMsgRaw(char const* target, uint8_t* rx_buf, struct pack_info* pack){
    return 0;
}

int canSendMsgRaw(char const* target, uint8_t* tx_buf, struct pack_info* pack){
    return 0;
}

int canRecvMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack){
    return 0;
}

int canSendMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack){
    return 0;
}

int canSendMsgConfig(char const* target, Can_Config_Filter_Type* filter, struct pack_info* pack){
    return 0;
}