/* Copyright 2025 人形机器人（上海）有限公司
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Designed and built with love @zhihu by @cjrcl.
 */

#pragma once

#ifdef __cplusplus
extern "C"{
#endif

enum cantype{
    CANType_Can = 0,
    CANType_Canfd
};

struct canframe{
    unsigned long time_stamp;
    unsigned int canid;
    unsigned char count;
    unsigned char can_type;
    unsigned char can_channel;
    unsigned char len;
    unsigned char data[64];
};

typedef struct __attribute__((__packed__)){
    unsigned char can_channel;
    unsigned char verbose;
    unsigned char reserved;
    unsigned char filter_num;
    unsigned int* filter_id;
}can_filter_info_t;

typedef struct __attribute__((__packed__)){
    unsigned short length;
    unsigned char reversed[2];
    can_filter_info_t data;
}Can_Config_Filter_Type;

struct pack_info{
    unsigned long soc_ts;
    unsigned int data_num;
    unsigned long mcu_ts;
    unsigned short length;
    unsigned short unused;
    unsigned long unused_1;
    unsigned char crc_enabled;
};

int canInit();
int canSendMsgConfig(char const* target, Can_Config_Filter_Type* filter, struct pack_info* packInfo);
int canSendMsgFrame(char const* target, struct canframe* frames, struct pack_info* packInfo);
int canRecvMsgFrame(char const* target, struct canframe* frames, struct pack_info* packInfo);
int canSendMsgRaw(char const* target, unsigned char* rxBuff, struct pack_info* packInfo);
int canRecvMsgRaw(char const* target, unsigned char* txBuff, struct pack_info* packInfo);
void canDeInit();

#ifdef __cplusplus
}
#endif