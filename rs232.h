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

#include "common.h"
#include <map>

namespace DriverSDK{
using imuValidationFunction = bool (*)(unsigned char const*);
using imuParsingFunction = void (*)(SwapList const*, int const);

struct ChainNode{
    int nr;
    ChainNode* previous, * next;
};

class RS232{
public:
    int order, baudrate, fd, frameLength;
    std::map<int, std::string> alias2type;
    char* device, * type;
    unsigned char header0, header1;
    SwapList* rxSwap, * txSwap, * txSwap_;
    pthread_t pth;
    imuValidationFunction valid;
    imuParsingFunction parse;
    RS232(int const order, char const* device);
    RS232(char const* device, int const baudrate, char const* type);
    int config();
    static void cleanup(void* arg);
    static void* recv(void* arg);
    int run();
    ~RS232();
};
}