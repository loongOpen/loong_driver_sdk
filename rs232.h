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

namespace DriverSDK{
using validationFunction = bool (*)(unsigned char const*);
using parseFunction = float (*)(SwapList const*);

struct ChainNode{
    int nr;
    ChainNode* previous, * next;
};

class RS232{
public:
    char* device;
    int fd, baudrate, frameLength;
    unsigned char header0, header1;
    std::atomic<ChainNode*> ptr;
    SwapList* txSwap;
    pthread_t pth;
    validationFunction valid;
    parseFunction rpy0, rpy1, rpy2, gyr0, gyr1, gyr2, acc0, acc1, acc2;
    RS232(char const* device, int const baudrate, char const* type);
    static void cleanup(void* arg);
    static void* recv(void* arg);
    int run();
    ~RS232();
};
}