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

#include "config_xml.h"
#include "common.h"
#include <limits>
#include <cmath>

namespace DriverSDK{
extern ConfigXML* configXML;

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
    polarity = encoderResolution = gearRatioTor = gearRatioPosVel = ratedCurrent = torqueConstant = maximumTorque = maximumPosition = 1.0;
    minimumPosition = -1.0;
}

int MotorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
    polarity          = configXML->readMotorParameter(alias, "Polarity");
    countBias         = configXML->readMotorParameter(alias, "CountBias");
    encoderResolution = configXML->readMotorParameter(alias, "EncoderResolution");
    gearRatioTor      = configXML->readMotorParameter(alias, "GearRatioTor");
    gearRatioPosVel   = configXML->readMotorParameter(alias, "GearRatioPosVel");
    ratedCurrent      = configXML->readMotorParameter(alias, "RatedCurrent");
    torqueConstant    = configXML->readMotorParameter(alias, "TorqueConstant");
    maximumTorque     = configXML->readMotorParameter(alias, "MaximumTorque");
    minimumPosition   = configXML->readMotorParameter(alias, "MinimumPosition");
    maximumPosition   = configXML->readMotorParameter(alias, "MaximumPosition");
    if( polarity          == std::numeric_limits<float>::min() ||
        countBias         == std::numeric_limits<float>::min() ||
        encoderResolution == std::numeric_limits<float>::min() ||
        gearRatioTor      == std::numeric_limits<float>::min() ||
        gearRatioPosVel   == std::numeric_limits<float>::min() ||
        ratedCurrent      == std::numeric_limits<float>::min() ||
        torqueConstant    == std::numeric_limits<float>::min() ||
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
        std::vector<std::string> entry = configXML->entry(configXML->busDevice("ECAT", type.c_str()), "Temperature");
        temperatureSDO = SDOMsg{
            sdoHandler,
            0,
            alias,
            0,
            (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
            (unsigned char) strtoul(entry[2].c_str(), nullptr, 16),
            (unsigned char) strtoul(entry[3].c_str(), nullptr, 10),
            (unsigned char) strtoul(entry[4].c_str(), nullptr, 10),
            (unsigned char) strtoul(entry[5].c_str(), nullptr, 10),
            0
        };
        entry = configXML->entry(configXML->busDevice("ECAT", type.c_str()), "ClearError");
        clearErrorSDO = SDOMsg{
            sdoHandler,
            1,
            alias,
            0,
            (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
            (unsigned char) strtoul(entry[2].c_str(), nullptr, 16),
            (unsigned char) strtoul(entry[3].c_str(), nullptr, 10),
            (unsigned char) strtoul(entry[4].c_str(), nullptr, 10),
            (unsigned char) strtoul(entry[5].c_str(), nullptr, 10),
            0
        };
    }else if(bus == "CAN"){
        ;
    }else if(bus == "RS485"){
        ;
    }
    return 0;
}

MotorParameters::~MotorParameters(){
}

EffectorParameters::EffectorParameters(){
}

int EffectorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
    return 0;
}

EffectorParameters::~EffectorParameters(){
}

SensorParameters::SensorParameters(){
}

int SensorParameters::load(std::string const& bus, int const alias, std::string const& type, ec_sdo_request_t* const sdoHandler){
    return 0;
}

SensorParameters::~SensorParameters(){
}
}