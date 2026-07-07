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

#include "common.h"
#include "config_xml.h"
#include <unistd.h>
#include <bit>
#include <limits>

namespace DriverSDK{
extern ConfigXML* configXML;

void print(unsigned char const* data, int const length){
    int i = 0;
    while(i < length){
        printf("%02X ", data[i]);
        if((i + 1) % 32 == 0){
            printf("\n");
        }else if((i + 1) % 8 == 0){
            printf(" ");
        }
        ++i;
    }
    printf("\n");
}

unsigned short single2half(float const f){
    unsigned int u = *(unsigned int*)&f;
    unsigned int sign = u & 0x80000000;
    int exp = u & 0x7f800000;
    unsigned int man = u & 0x007fffff;
    unsigned int halfSign = sign >> 16;
    if(exp == 0x7f800000){
        unsigned int nanBit = man == 0 ? 0 : 0x0200;
        return halfSign | 0x7c00 | nanBit | (man >> 13);
    }
    int halfExp = (exp >> 23) - 127 + 15;   // 2 ^ (8 - 1) - 1 == 127; 2 ^ (5 - 1) - 1 == 15
    if(halfExp >= 0x1f){
        return halfSign | 0x7c00;
    }
    if(halfExp <= 0){   // halfExp == 0: subnormal number
        if(14 - halfExp > 24){
            return halfSign;
        }
        man = man | 0x00800000; // bit 24: connotative 1.0
        unsigned int halfMan = man >> (14 - halfExp);
        unsigned int roundBit = 1 << (13 - halfExp);    // bit (14 - halfExp)
        if((man & roundBit) != 0 && (man & (3 * roundBit - 1)) != 0){
            ++halfMan;
        }
        return halfSign | halfMan;
    }
    halfExp = halfExp << 10;
    unsigned int halfMan = man >> 13;   // 23 - 10 == 13
    unsigned int roundBit = 0x00001000; // bit 13
    if((man & roundBit) != 0 && (man & (3 * roundBit - 1)) != 0){
        return (halfSign | halfExp | halfMan) + 1;
    }else{
        return halfSign | halfExp | halfMan;
    }
}

float half2single(unsigned short const u){
    if(u & 0x7fff == 0){
        unsigned int result = (unsigned int)u << 16;
        float f = *(float*)&result;
        return f;
    }
    unsigned int halfSign = u & 0x8000;
    int halfExp = u & 0x7c00;
    unsigned int halfMan = u & 0x03ff;
    unsigned int sign = halfSign << 16;
    if(halfExp == 0x7c00){
        unsigned int result = sign | 0x7f800000 | (halfMan << 13);
        float f = *(float*)&result;
        return f;
    }
    int exp = (halfExp >> 10) - 15 + 127;   // 2 ^ (5 - 1) - 1 == 15; 2 ^ (8 - 1) - 1 == 127
    if(halfExp == 0){   // halfExp == 0: subnormal number
        int e = std::countl_zero(halfMan) - 6;  // 6 leading zeros of 0x03ff
        exp = (exp - e) << 23;
        unsigned int man = (halfMan << (14 + e)) & 0x7fffff;    //  23 - 10 + 1 == 14; std::countl_one(0x7fffff) == 23
        unsigned int result = sign | exp | man;
        float f = *(float*)&result;
        return f;
    }
    exp = exp << 23;
    unsigned int result = sign | exp | (halfMan << 13);
    float f = *(float*)&result;
    return f;
}

int quadchar2int(unsigned char const* qc){
    int i = 0;
    unsigned char* c = (unsigned char*)&i;
    *(c + 0) = *(qc + 3);
    *(c + 1) = *(qc + 2);
    *(c + 2) = *(qc + 1);
    *(c + 3) = *(qc + 0);
    return i;
}

int quadchar2int_(unsigned char const* qc){
    return *(int*)qc;
}

float quadchar2float(unsigned char const* qc){
    float f = 0;
    unsigned char* c = (unsigned char*)&f;
    *(c + 0) = *(qc + 3);
    *(c + 1) = *(qc + 2);
    *(c + 2) = *(qc + 1);
    *(c + 3) = *(qc + 0);
    return f;
}

float quadchar2float_(unsigned char const* qc){
    return *(float*)qc;
}

void adjustCPU(int* const cpu, int const processor){
    if(*cpu <= 0 || *cpu >= sysconf(_SC_NPROCESSORS_ONLN)){
        *cpu = sysconf(_SC_NPROCESSORS_ONLN) - 1;
        if(*cpu > processor && processor > 0){
            *cpu = processor;
        }
    }
}

SwapNode::SwapNode(int const size){
    memPtr = (unsigned char*)calloc(size, 1);
    previous = nullptr;
    next = nullptr;
}

SwapNode::~SwapNode(){
    free(memPtr);
}

SwapList::SwapList(int const size){
    nodePtr.store(new SwapNode(size));
    SwapNode* current = nodePtr.load();
    int i = 1;
    while(i < 3){
        SwapNode* node = new SwapNode(size);
        node->previous = current;
        current->next = node;
        current = current->next;
        ++i;
    }
    nodePtr.load()->previous = current;
    current->next = nodePtr.load();
}

void SwapList::advanceNodePtr(){
    nodePtr.store(nodePtr.load()->next);
}

void SwapList::copyTo(unsigned char* const domainPtr, int const domainSize){
    memcpy(domainPtr, nodePtr.load()->previous->memPtr, domainSize);
}

void SwapList::copyFrom(unsigned char const* domainPtr, int const domainSize){
    SwapNode* node = nodePtr.load();
    memcpy(node->next->memPtr, domainPtr, domainSize);
    nodePtr.store(node->next);
}

SwapList::~SwapList(){
    SwapNode* current = nodePtr.load();
    while(current != nullptr){
        SwapNode* node = current;
        current = current->next;
        node->previous->next = nullptr;
        delete node;
    }
}

DriverParameters::DriverParameters(){
}

int DriverParameters::load(std::string const& type){
    minP  = configXML->readDeviceParameter("CAN", type.c_str(),  "MinP");
    maxP  = configXML->readDeviceParameter("CAN", type.c_str(),  "MaxP");
    minV  = configXML->readDeviceParameter("CAN", type.c_str(),  "MinV");
    maxV  = configXML->readDeviceParameter("CAN", type.c_str(),  "MaxV");
    minKp = configXML->readDeviceParameter("CAN", type.c_str(), "MinKp");
    maxKp = configXML->readDeviceParameter("CAN", type.c_str(), "MaxKp");
    minKd = configXML->readDeviceParameter("CAN", type.c_str(), "MinKd");
    maxKd = configXML->readDeviceParameter("CAN", type.c_str(), "MaxKd");
    minT  = configXML->readDeviceParameter("CAN", type.c_str(),  "MinT");
    maxT  = configXML->readDeviceParameter("CAN", type.c_str(),  "MaxT");
    pUnit       = configXML->readDeviceParameter("CAN", type.c_str(),       "PUnit");
    targetVUnit = configXML->readDeviceParameter("CAN", type.c_str(), "TargetVUnit");
    actualVUnit = configXML->readDeviceParameter("CAN", type.c_str(), "ActualVUnit");
    vOffsetUnit = configXML->readDeviceParameter("CAN", type.c_str(), "VOffsetUnit");
    targetCUnit = configXML->readDeviceParameter("CAN", type.c_str(), "TargetCUnit");
    actualCUnit = configXML->readDeviceParameter("CAN", type.c_str(), "ActualCUnit");
    cOffsetUnit = configXML->readDeviceParameter("CAN", type.c_str(), "COffsetUnit");
    tConstant   = configXML->readDeviceParameter("CAN", type.c_str(),   "TConstant");
    if( minP  == std::numeric_limits<float>::min() ||
        maxP  == std::numeric_limits<float>::min() ||
        minV  == std::numeric_limits<float>::min() ||
        maxV  == std::numeric_limits<float>::min() ||
        minKp == std::numeric_limits<float>::min() ||
        maxKp == std::numeric_limits<float>::min() ||
        minKd == std::numeric_limits<float>::min() ||
        maxKd == std::numeric_limits<float>::min() ||
        minT  == std::numeric_limits<float>::min() ||
        maxT  == std::numeric_limits<float>::min() ||
        pUnit       == std::numeric_limits<float>::min() ||
        targetVUnit == std::numeric_limits<float>::min() ||
        actualVUnit == std::numeric_limits<float>::min() ||
        vOffsetUnit == std::numeric_limits<float>::min() ||
        targetCUnit == std::numeric_limits<float>::min() ||
        actualCUnit == std::numeric_limits<float>::min() ||
        cOffsetUnit == std::numeric_limits<float>::min() ||
        tConstant   == std::numeric_limits<float>::min()){
        printf("no parameter of driver with type %s is set in xml\n", type.c_str());
        return -1;
    }
    return 0;
}

DriverParameters::~DriverParameters(){
}

MotorParameters::MotorParameters(){
    countBias = 0.0;
    polarity = encoderResolution = gearRatioTor = gearRatioPosVel = ratedCurrent = torqueConstant = ratedTorque = maximumTorque = maximumPosition = 1.0;
    minimumPosition = -1.0;
}

#ifndef NIIC
int MotorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int MotorParameters::load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler){
#endif
    polarity          = configXML->readMotorParameter(alias,          "Polarity");
    countBias         = configXML->readMotorParameter(alias,         "CountBias");
    encoderResolution = configXML->readMotorParameter(alias, "EncoderResolution");
    gearRatioTor      = configXML->readMotorParameter(alias,      "GearRatioTor");
    gearRatioPosVel   = configXML->readMotorParameter(alias,   "GearRatioPosVel");
    ratedCurrent      = configXML->readMotorParameter(alias,      "RatedCurrent");
    torqueConstant    = configXML->readMotorParameter(alias,    "TorqueConstant");
    ratedTorque       = configXML->readMotorParameter(alias,       "RatedTorque");
    maximumTorque     = configXML->readMotorParameter(alias,     "MaximumTorque");
    minimumPosition   = configXML->readMotorParameter(alias,   "MinimumPosition");
    maximumPosition   = configXML->readMotorParameter(alias,   "MaximumPosition");
    if( polarity          == std::numeric_limits<float>::min() ||
        countBias         == std::numeric_limits<float>::min() ||
        encoderResolution == std::numeric_limits<float>::min() ||
        gearRatioTor      == std::numeric_limits<float>::min() ||
        gearRatioPosVel   == std::numeric_limits<float>::min() ||
        ratedCurrent      == std::numeric_limits<float>::min() ||
        torqueConstant    == std::numeric_limits<float>::min() ||
        ratedTorque       == std::numeric_limits<float>::min() ||
        maximumTorque     == std::numeric_limits<float>::min() ||
        minimumPosition   == std::numeric_limits<float>::min() ||
        maximumPosition   == std::numeric_limits<float>::min()){
        printf("a motor parameter is incorrectly set in xml\n");
        return -1;
    }
    if(bus == "ECAT"){
        sdoTemplate = SDOMsg{
            sdoHandler,
            0,
            alias,
            0,
            0x0000,
            0x00,
            0,
            0,
            0,
            0
        };
        std::vector<std::string> entry = configXML->entry(configXML->device("ECAT", type.c_str()), "Temperature");
        temperatureSDO = SDOMsg{
            sdoHandler,
            0,
            alias,
            0,
            (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
            (unsigned char )strtoul(entry[2].c_str(), nullptr, 16),
            (unsigned char )strtoul(entry[3].c_str(), nullptr, 10),
            (unsigned char )strtoul(entry[4].c_str(), nullptr, 10),
            (unsigned char )strtoul(entry[5].c_str(), nullptr, 10),
            0
        };
        entry = configXML->entry(configXML->device("ECAT", type.c_str()), "ClearError");
        clearErrorSDO = SDOMsg{
            sdoHandler,
            1,
            alias,
            0,
            (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
            (unsigned char )strtoul(entry[2].c_str(), nullptr, 16),
            (unsigned char )strtoul(entry[3].c_str(), nullptr, 10),
            (unsigned char )strtoul(entry[4].c_str(), nullptr, 10),
            (unsigned char )strtoul(entry[5].c_str(), nullptr, 10),
            0
        };
    }
    return 0;
}

MotorParameters::~MotorParameters(){
}

IMUParameters::IMUParameters(){
}

#ifndef NIIC
int IMUParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int IMUParameters::load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler){
#endif
    return 0;
}

IMUParameters::~IMUParameters(){
}

EffectorParameters::EffectorParameters(){
}

#ifndef NIIC
int EffectorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int EffectorParameters::load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler){
#endif
    return 0;
}

EffectorParameters::~EffectorParameters(){
}

SensorParameters::SensorParameters(){
}

#ifndef NIIC
int SensorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int SensorParameters::load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler){
#endif
    return 0;
}

SensorParameters::~SensorParameters(){
}

TransferrerParameters::TransferrerParameters(){
}

#ifndef NIIC
int TransferrerParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int TransferrerParameters::load(std::string const& bus, int const alias, std::string const& type, ecat::sdo_request* const sdoHandler){
#endif
    return 0;
}

TransferrerParameters::~TransferrerParameters(){
}
}