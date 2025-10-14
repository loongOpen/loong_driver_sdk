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

#include <vector>
#include <string>
#include <cmath>

namespace DriverSDK{
float const Pi = std::acos(-1);

struct imuStruct{
    float rpy[3], gyr[3], acc[3];
};

struct sensorStruct{
    float F[3], M[3];
    unsigned int statusCode;
};

struct digitTargetStruct{
    unsigned short pos = 0;             // 0 ~ 90: relaxed ~ tense
};

struct digitActualStruct{
    unsigned short pos = 0;             // 0 ~ 90: relaxed ~ tense
};

struct motorTargetStruct{
    float pos = 0.0, vel = 0.0, tor = 0.0, kp = 0.0, kd = 0.0;
    int enabled = 0;                    // -1: clear error; 0: disable; 1: enable; 2: damp
};

struct motorActualStruct{
    float pos = 0.0, vel = 0.0, tor = 0.0;
    short temp = 0;
    unsigned short statusWord = 65535;  // 0: PREOP; 65535: inactive
    unsigned short errorCode = 0;
};

unsigned short single2half(float f);
float half2single(unsigned short u);

class motorSDOClass{
public:
    long value;
    int i;                              // drivers[i]
    short state;                        // -1: error; 0: pending; 1, 2: processing; 3: completed
    unsigned short index;
    unsigned char subindex;
    unsigned char signed_;              // 0: unsigned; 1: signed
    unsigned char bitLength;            // 8, 16 or 32
    unsigned char operation;            // 0: write; 1: read
    motorSDOClass(int i);
    ~motorSDOClass();
};

class DriverSDK{
public:
    static DriverSDK& instance();
    void setCPU(unsigned short const cpu);
    void setCPU(unsigned short const cpu, std::string const& bus);
    void setMaxCurr(std::vector<unsigned short> const& maxCurr);
    int setMode(std::vector<char> const& mode);
    void init(char const* xmlFile);
    int getLeftDigitNr();
    int getRightDigitNr();
    int getTotalMotorNr();
    std::vector<int> getActiveMotors();
    int setCntBias(std::vector<int> const& cntBias);
    int fillSDO(motorSDOClass& data, char const* object);
    void getIMU(imuStruct& data);
    int getSensor(std::vector<sensorStruct>& data);
    int setDigitTarget(std::vector<digitTargetStruct> const& data);
    int getDigitActual(std::vector<digitActualStruct>& data);
    int setMotorTarget(std::vector<motorTargetStruct> const& data);
    int getMotorActual(std::vector<motorActualStruct>& data);
    int getEncoderCount(std::vector<int>& data);
    int sendMotorSDORequest(motorSDOClass const& data);
    int recvMotorSDOResponse(motorSDOClass& data);
    int calibrate(int const i);
    void advance();
    std::string version();
private:
    class impClass;
    impClass& imp;
    DriverSDK();
    ~DriverSDK();
    DriverSDK(DriverSDK const&) = delete;
    DriverSDK& operator=(DriverSDK const&) = delete;
};
}