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

#include "hobot_can_hal.h"
#include <enpht_can.h>
#include <string.h>

#define BSWAP(X) (((unsigned int)(X) & 0xff000000) >> 24 | ((unsigned int)(X) & 0x00ff0000) >> 8 | ((unsigned int)(X) & 0x0000ff00) << 8 | ((unsigned int)(X) & 0x000000ff) << 24)

ViUInt32 cardNum;

int canInit(){
    ViStatus hr = EphCan_USB_AutoConnectToFirst(&cardNum);
    if(hr != 0){
        return hr < 0 ? hr : -hr;
    }
    EphCAN_Reset(cardNum);
    struct EP_CAN_INIT epCanInit;
    memset(&epCanInit, 0, sizeof(epCanInit));
    epCanInit.Bitrate = RATE_1000KHZ;
    ViUInt16 ch = 0;
    while(ch < 8){
        EphCAN_SetWorkMode(cardNum, ch, WM_CONFIG);
        EphCAN_InitCAN(cardNum, ch, &epCanInit);
        EphCAN_SetWorkMode(cardNum, ch, WM_NORMAL);
        ++ch;
    }
    return 0;
}

int canSendMsgConfig(char const* target, Can_Config_Filter_Type* filter, struct pack_info* packInfo){
    return 0;
}

int canSendMsgFrame(char const* target, struct canframe* frames, struct pack_info* packInfo){
    static struct EP_CAN_OBJ objs[64];
    int i = 0;
    while(i < packInfo->data_num){
        objs[i].ID = BSWAP(frames[i].canid);
        objs[i].TimeFlag = 0;
        objs[i].RemoteFlag = frames[i].can_type >> 2;
        objs[i].ExtendedFlag = 0;
        objs[i].DataLen = frames[i].len;
        memcpy(objs[i].Data, frames[i].data, frames[i].len);
        ++i;
    }
    ViStatus hr = EphCAN_Transmit(cardNum, target[3] - '0', packInfo->data_num, objs);
    if(hr != 0){
        return hr < 0 ? hr : -hr;
    }
    return packInfo->data_num;
}

int canRecvMsgFrame(char const* target, struct canframe* frames, struct pack_info* packInfo){
    static struct EP_CAN_OBJ objs[8][32];
    static unsigned char count[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    ViUInt16 ch = target[3] - '0';
    ViUInt32 frameCount;
    ViStatus hr = EphCAN_Receive(cardNum, ch, 32, objs[ch], &frameCount);
    if(hr != 0){
        return hr < 0 ? hr : -hr;
    }
    int i = 0;
    while(i < frameCount){
        frames[i].canid = objs[ch][i].ID;
        frames[i].count = ++count[ch];
        frames[i].can_type = objs[ch][i].RemoteFlag << 2;
        frames[i].can_channel = ch;
        frames[i].len = objs[ch][i].DataLen;
        memcpy(frames[i].data, objs[ch][i].Data, objs[ch][i].DataLen);
        ++i;
    }
    return packInfo->data_num = frameCount;
}

int canSendMsgRaw(char const* target, unsigned char* rxBuff, struct pack_info* packInfo){
    return 0;
}

int canRecvMsgRaw(char const* target, unsigned char* txBuff, struct pack_info* packInfo){
    return 0;
}

void canDeInit(){
    EphCAN_Close(cardNum);
}