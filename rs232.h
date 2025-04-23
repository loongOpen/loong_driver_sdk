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

namespace DriverSDK{
struct ChainNode{
    int nr;
    ChainNode* previous, * next;
};

class RS232{
public:
    char* device;
    int baudrate, frameLength;
    unsigned char header0, header1;
    std::atomic<ChainNode*> ptr;
    SwapList* txSwap;
    pthread_t pth;
    RS232(char const* device, int const baudrate, int const frameLength, unsigned char const header0, unsigned char const header1);
    virtual bool valid(unsigned char const* buff) = 0;
    static void cleanup(void* arg);
    static void* recv(void* arg);
    int run();
    ~RS232();
};

class IMU : public RS232{
public:
    IMU(char const* device, int const baudrate, int const frameLength, unsigned char const header0, unsigned char const header1);
    float quadchar2float(unsigned char const* qc);
    bool valid(unsigned char const* buff) override;
    ~IMU();
};
}