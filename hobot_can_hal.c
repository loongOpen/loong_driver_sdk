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

int canInit(){
    return 0;
}

int canSendMsgConfig(char const* target, Can_Config_Filter_Type* filter, struct pack_info* pack){
    return 0;
}

int canSendMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack){
    return 0;
}

int canRecvMsgFrame(char const* target, struct canframe* frame, struct pack_info* pack){
    return 0;
}

int canSendMsgRaw(char const* target, unsigned char* tx_buf, struct pack_info* pack){
    return 0;
}

int canRecvMsgRaw(char const* target, unsigned char* rx_buf, struct pack_info* pack){
    return 0;
}

void canDeInit(){
}