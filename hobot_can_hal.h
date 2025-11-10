#include <stdint.h>
#include <linux/can.h>

#ifdef __cplusplus
extern "C"{
#endif

#define CAN_FRAME_LENGTH_MAX 64

enum cantype{
    CANType_Can = 0,
    CANType_Canfd
};

struct canframe{
    uint64_t time_stamp;
    uint32_t canid;
    uint8_t count;
    uint8_t can_type;
    uint8_t can_channel;
    uint8_t len;
    uint8_t data[CAN_FRAME_LENGTH_MAX];
};

#pragma pack(1)
typedef struct{
    uint8_t can_channel;
    uint8_t verbose;
    uint8_t reserved;
    uint8_t filter_num;
    uint32_t* filter_id;
}can_filter_info_t;

typedef struct{
    uint16_t length;
    uint8_t reversed[2];
    can_filter_info_t data;
}Can_Config_Filter_Type;

#pragma pack(0)
struct pack_info{
    uint64_t soc_ts;
    uint32_t data_num;
    uint64_t mcu_ts;
    uint16_t length;
    uint16_t unused;
    uint64_t unused_1;
};

int canInit();
void canDeInit();
int canRecvMsgRaw(char const* target, uint8_t* rx_buf, struct pack_info* pack);
int canSendMsgRaw(char const* target, uint8_t* tx_buf, struct pack_info* pack);
int canRecvMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack);
int canSendMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack);
int canSendMsgConfig(char const* target, Can_Config_Filter_Type* filter, struct pack_info* pack);

#ifdef __cplusplus
}
#endif