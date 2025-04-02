/* Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司
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

#include "common.h"
#include <modbus-rtu.h>

namespace DriverSDK{
using effectorFunction = void (*)(modbus_t* const, int const);

class RS485{
public:
    int order, baudrate;
    char* device;
    std::map<int, std::string> alias2type;
    long period;
    modbus_t* ctx;
    SwapList* rxSwap, * txSwap;
    pthread_t pth;
    effectorFunction leftRX, rightRX, leftTX, rightTX;
    RS485(int const order, char const* device);
    int config();
    static void* rxtx(void* arg);
    int run();
    ~RS485();
};
}