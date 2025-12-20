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
#include <bit>
#include <limits>

namespace DriverSDK{
extern ConfigXML* configXML;

unsigned short single2half(float f){
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
            halfMan++;
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

float half2single(unsigned short u){
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
        i++;
    }
    nodePtr.load()->previous = current;
    current->next = nodePtr.load();
}

void SwapList::advanceNodePtr(){
    nodePtr.store(nodePtr.load()->next);
}

void SwapList::copyTo(unsigned char* domainPtr, int const domainSize){
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

MotorParameters::MotorParameters(){
    countBias = 0.0;
    polarity = encoderResolution = gearRatioTor = gearRatioPosVel = ratedCurrent = torqueConstant = ratedTorque = maximumTorque = maximumPosition = 1.0;
    minimumPosition = -1.0;
}

#ifndef NIIC
int MotorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int MotorParameters::load(std::string const& bus, int const alias, std::string const& type, int const slave){
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
#ifndef NIIC
            sdoHandler,
#else
            slave,
#endif
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
#ifndef NIIC
            sdoHandler,
#else
            slave,
#endif
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
#ifndef NIIC
            sdoHandler,
#else
            slave,
#endif
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

EffectorParameters::EffectorParameters(){
}

#ifndef NIIC
int EffectorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
#else
int EffectorParameters::load(std::string const& bus, int const alias, std::string const& type, int const slave){
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
int SensorParameters::load(std::string const& bus, int const alias, std::string const& type, int const slave){
#endif
    return 0;
}

SensorParameters::~SensorParameters(){
}
}