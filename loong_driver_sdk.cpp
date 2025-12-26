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

#include "version.h"
#include "loong_driver_sdk.h"
#include "config_xml.h"
#include "rs232.h"
#include "rs485.h"
#include "can.h"
#ifndef NIIC
#include "ecat.h"
#else
#include "ecat.hpp"
#endif
#include <unistd.h>
#include <atomic>
#include <sstream>
#include <limits>

namespace DriverSDK{
ConfigXML* configXML;
std::vector<std::map<int, std::string>> rs485alias2type, canAlias2type, ecatAlias2type, rs485emuAlias2type;
std::vector<std::map<int, int>> canAlias2masterID, canAlias2slaveID, ecatAlias2domain;
std::vector<std::vector<int>> ecatDomainDivisions;
int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector, batteryCount;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** legs[2], ** arms[2], ** waist, ** neck;
WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
WrapperPair<ConverterRxData, ConverterTxData, EffectorParameters> converters[2];
WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
std::vector<unsigned short> processorsECAT, processorsCAN;
std::vector<char> operatingMode;
std::vector<unsigned short> maxCurrent;
std::atomic<int> ecatStalled;
std::vector<RS485>* rs485sPtr;

motorSDOClass::motorSDOClass(int i){
    this->i = i;
}

motorSDOClass::~motorSDOClass(){
}

motorREGClass::motorREGClass(int i){
    this->i = i;
}

motorREGClass::~motorREGClass(){
}

class DriverSDK::impClass{
public:
    RS232* imu;
    std::vector<RS485> rs485s;
    std::vector<CANDriver> cans;
    std::vector<Battery> bats;
    std::vector<ECAT> ecats;
    impClass();
    int effectorCheck(std::vector<std::map<int, std::string>> const& alias2type, char const* bus);
    int driverCheck(std::vector<std::map<int, std::string>> const& canAlias2type, std::vector<std::map<int, std::string>> const& ecatAlias2type);
    int init(char const* xmlFile);
    int putDriverSDORequest(SDOMsg const& msg, int const priority = QUE_PRI_LOW);
    int getDriverSDOResponse(SDOMsg& msg);
    int putDriverREGRequest(REGMsg const& msg, int const priority = QUE_PRI_LOW);
    int getDriverREGResponse(REGMsg& msg);
    void rs485Update();
    void canUpdate();
    void ecatUpdate();
    void sdoRequestableUpdate();
    ~impClass();
};

DriverSDK::impClass::impClass(){
    configXML = nullptr;
    dofLeg = dofArm = dofWaist = dofNeck = dofAll = dofLeftEffector = dofRightEffector = dofEffector = 0;
    drivers = nullptr;
    legs[0] = legs[1] = arms[0] = arms[1] = waist = neck = nullptr;
    digits = nullptr;
    CANDriver::alias2masterID_ = nullptr;
    imu = nullptr;
    int i = 0;
    while(i < 6){
        processorsECAT.push_back(sysconf(_SC_NPROCESSORS_ONLN) - 1);
        i++;
    }
    i = 0;
    while(i < 3){
        processorsCAN.push_back(sysconf(_SC_NPROCESSORS_ONLN) - 1);
        i++;
    }
    ecatStalled.store(0);
    rs485sPtr = &rs485s;
    rs485s.reserve(8);
    cans.reserve(8);
    ecats.reserve(6);
}

int DriverSDK::impClass::effectorCheck(std::vector<std::map<int, std::string>> const& alias2type, char const* bus){
    int i = 0;
    while(i < alias2type.size()){
        auto itr = alias2type[i].begin();
        while(itr != alias2type[i].end()){
            if(itr->first == 200 || itr->first == 201){
                int dof = atoi(configXML->typeAttribute(bus, itr->second.c_str(), "dof").c_str());
                if(dof <= 0){
                     printf("invalid effector dof\n");
                     return -1;
                }
                if(itr->first == 200){
                    if(dofLeftEffector > 0){
                        printf("conflicting left effector\n");
                        return -1;
                    }else{
                        dofLeftEffector = dof;
                    }
                }else if(itr->first == 201){
                    if(dofRightEffector > 0){
                        printf("conflicting right effector\n");
                        return -1;
                    }else{
                        dofRightEffector = dof;
                    }
                }
            }
            itr++;
        }
        i++;
    }
    dofEffector = dofLeftEffector + dofRightEffector;
    return 0;
}

int DriverSDK::impClass::driverCheck(std::vector<std::map<int, std::string>> const& canAlias2type, std::vector<std::map<int, std::string>> const& ecatAlias2type){
    bool existing[dofAll];
    int i = 0;
    while(i < dofAll){
        existing[i] = false;
        i++;
    }
    i = 0;
    while(i < canAlias2type.size()){
        auto itr = canAlias2type[i].begin();
        while(itr != canAlias2type[i].end()){
            existing[itr->first - 1] = true;
            itr++;
        }
        i++;
    }
    i = 0;
    while(i < ecatAlias2type.size()){
        auto itr = ecatAlias2type[i].begin();
        while(itr != ecatAlias2type[i].end()){
            if(existing[itr->first - 1]){
                return -1;
            }
            itr++;
        }
        i++;
    }
    return 0;
}

int DriverSDK::impClass::init(char const* xmlFile){
    configXML = new ConfigXML(xmlFile);
    std::vector<std::vector<int>> motorAlias = configXML->motorAlias();
    if(motorAlias.size() != 6){
        printf("there must be 6 limbs: left leg, right leg, left arm, right arm, waist and neck.\nwhereas the size of a limb can be 0.");
        return -1;
    }
    if(motorAlias[0].size() != motorAlias[1].size()){
        printf("the two legs does not mirror each other\n");
        return -1;
    }
    if(motorAlias[2].size() != motorAlias[3].size()){
        printf("the two arms does not mirror each other\n");
        return -1;
    }
    dofLeg   = motorAlias[0].size();
    dofArm   = motorAlias[2].size();
    dofWaist = motorAlias[4].size();
    dofNeck  = motorAlias[5].size();
    dofAll   = 2 * dofLeg + 2 * dofArm + dofWaist + dofNeck;
    if(dofAll > 0){
        drivers = new WrapperPair<DriverRxData, DriverTxData, MotorParameters> [  dofAll];
    }
    if(dofLeg > 0){
        legs[0] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[  dofLeg];
        legs[1] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[  dofLeg];
    }
    if(dofArm > 0){
        arms[0] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[  dofArm];
        arms[1] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[  dofArm];
    }
    if(dofWaist > 0){
        waist   = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofWaist];
    }
    if(dofNeck > 0){
        neck    = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[ dofNeck];
    }
    rs485alias2type    = configXML->alias2type("RS485");
    rs485emuAlias2type = configXML->alias2type("RS485Emu");
    canAlias2type      = configXML->alias2type("CAN");
    ecatAlias2type     = configXML->alias2type("ECAT");
    if(rs485alias2type.size() > 4){
        printf("4 rs485 masters at most\n");
        return -1;
    }
    if(rs485emuAlias2type.size() > 4){
        printf("4 rs485emu masters at most\n");
        return -1;
    }
    if(canAlias2type.size() > 8){
        printf("8 can masters at most\n");
        return -1;
    }
    if(ecatAlias2type.size() > 6){
        printf("6 ecat masters at most\n");
        return -1;
    }
    if(effectorCheck(rs485alias2type, "RS485") != 0 || effectorCheck(ecatAlias2type, "ECAT") != 0){
        printf("invalid effector configuration\n");
        return -1;
    }
    if(driverCheck(canAlias2type, ecatAlias2type) != 0){
        printf("invalid driver configuration\n");
        return -1;
    }
    int i = 0;
    while(i < rs485emuAlias2type.size()){
        if(rs485emuAlias2type[i].size() == 0){
            i++;
            continue;
        }
        if(rs485emuAlias2type[i].size() > 1){
            printf("there should not be more than one slave belonging to a rs485emu master %d\n", i);
            return -1;
        }
        bool found = false;
        int j = 0, alias = rs485emuAlias2type[i].begin()->first, dof = atoi(configXML->typeAttribute("RS485Emu", rs485emuAlias2type[i].begin()->second.c_str(), "dof").c_str());
        while(j < ecatAlias2type.size()){
            auto itr = ecatAlias2type[j].find(alias);
            if(itr != ecatAlias2type[j].end()){
                if(atoi(configXML->typeAttribute("ECAT", itr->second.c_str(), "dof").c_str()) != dof){
                    printf("the dof of ecat converter device with alias %d should match that(%d) of the corresponding rs485emu device\n", alias, dof);
                    return -1;
                }
                found = true;
            }
            j++;
        }
        if(found == false){
            printf("rs485emu device with alias %d should be used together with a ecat to rs485 converter device with the same alias\n", alias);
            return -1;
        }
        i++;
    }
    printf("dofLeftEffector %d, dofRightEffector %d, dofEffector %d\n", dofLeftEffector, dofRightEffector, dofEffector);
    if(dofEffector > 0){
        digits = new WrapperPair<DigitRxData, DigitTxData, EffectorParameters>[dofEffector];
    }
    canAlias2masterID   = configXML->alias2attribute("CAN", "master_id");
    canAlias2slaveID    = configXML->alias2attribute("CAN", "slave_id");
    ecatAlias2domain    = configXML->alias2attribute("ECAT", "domain");
    ecatDomainDivisions = configXML->domainDivisions("ECAT");
    i = 0;
    if(operatingMode.size() == 0){
        while(i < dofAll){
            operatingMode.push_back(8);
            i++;
        }
    }else if(operatingMode.size() != dofAll){
        printf("invalid operatingMode\n");
        return -1;
    };
    i = 0;
    if(maxCurrent.size() == 0){
        while(i < dofAll){
            maxCurrent.push_back(1000);
            i++;
        }
    }else if(maxCurrent.size() != dofAll){
        printf("invalid maxCurrent\n");
        return -1;
    }
    imu = new RS232(configXML->imuAttribute("device").c_str(), configXML->imuBaudrate(), configXML->imuAttribute("type").c_str());
    if(imu->run() < 0){
        printf("imu run failed\n");
        return -1;
    }
    if(imu->pth > 0 && pthread_detach(imu->pth) != 0){
        printf("detaching imu recv thread failed\n");
        return -1;
    }
    int rs485masterCount = rs485alias2type.size();
    i = 0;
    while(i < rs485masterCount){
        rs485s.emplace_back(i, configXML->masterDevice("RS485", i, "device").c_str());
        printf("rs485s[%d] created\n", i);
        i++;
    }
    i = 0;
    while(i < rs485emuAlias2type.size()){
        rs485alias2type.push_back(rs485emuAlias2type[i]);
        rs485s.emplace_back(i + rs485masterCount, configXML->masterDevice("RS485Emu", i, "deviceR").c_str(), configXML->masterDevice("RS485Emu", i, "deviceS").c_str());
        printf("rs485s[%d] (rs485emus[%d]) created\n", i, i - rs485masterCount);
        i++;
    }
    i = 0;
    while(i < rs485s.size()){
        int res = rs485s[i].config();
        if(res == -1){
            printf("rs485s[%d] config failed\n", i);
            return -1;
        }else if(res == 1){
            sleep(1);
            printf("rs485s[%d] config retrying\n", i);
            continue;
        }
        i++;
    }
    i = 0;
    while(i < canAlias2type.size()){
        cans.emplace_back(i, configXML->masterDevice("CAN", i, "device").c_str());
        printf("cans[%d] created\n", i);
        i++;
    }
    i = 0;
    while(i < cans.size()){
        if(cans[i].config() < 0){
            printf("cans[%d] config failed\n", i);
            return -1;
        }
        i++;
    }
    i = 0;
    while(i < ecatAlias2type.size()){
        ecats.emplace_back(i);
        printf("ecats[%d] created\n", i);
        i++;
    }
    i = 0;
    while(i < ecats.size()){
        int res = ecats[i].check();
        if(res == -1){
            printf("ecats[%d] check failed\n", i);
            return -1;
        }else if(res == 1){
            sleep(1);
            printf("ecats[%d] check retrying\n", i);
            continue;
        }
        i++;
    }
    i = 0;
    while(i < ecats.size()){
        if(ecats[i].config() < 0){
            printf("ecats[%d] config failed\n", i);
            return -1;
        }
        i++;
    }
    i = 0;
    while(i < motorAlias.size()){
        printf("limb %d\n", i);
        int j = 0;
        while(j < motorAlias[i].size()){
            int alias = motorAlias[i][j];
            printf("\tjoint %d, alias %d\n", j, alias);
            if(i == 0 || i == 1){
                if(drivers[alias - 1].order == -1){
                    legs[i][j] = nullptr;
                }else{
                    legs[i][j] = &drivers[alias - 1];
                }
            }else if(i == 2 || i == 3){
                if(drivers[alias - 1].order == -1){
                    arms[i - 2][j] = nullptr;
                }else{
                    arms[i - 2][j] = &drivers[alias - 1];
                }
            }else if(i == 4){
                if(drivers[alias - 1].order == -1){
                    waist[j] = nullptr;
                }else{
                    waist[j] = &drivers[alias - 1];
                }
            }else if(i == 5){
                if(drivers[alias - 1].order == -1){
                    neck[j] = nullptr;
                }else{
                    neck[j] = &drivers[alias - 1];
                }
            }
            j++;
        }
        i++;
    }
    i = 0;
    while(i < ecats.size()){
        if(ecats[i].run() < 0){
            printf("ecats[%d] run failed\n", i);
            return -1;
        }
        i++;
    }
    if(CANDriver::run(cans) < 0){
        printf("can run failed\n");
        return -1;
    }
    i = 0;
    while(i < rs485s.size()){
        if(rs485s[i].run() < 0){
            printf("rs485s[%d] run failed\n", i);
            return -1;
        }
        i++;
    }
    return 0;
}

int DriverSDK::impClass::putDriverSDORequest(SDOMsg const& msg, int const priority){
    if(ecats[drivers[msg.alias - 1].order].sdoRequestable){
        SDOMsg* sdoMsg = new SDOMsg();
        *sdoMsg = msg;
        sdoMsg->state = 0;
        ecats[drivers[msg.alias - 1].order].sdoRequestQueue.put(sdoMsg, priority);
        return 0;
    }
    return -1;
}

int DriverSDK::impClass::getDriverSDOResponse(SDOMsg& msg){
    SDOMsg* sdoMsg = ecats[drivers[msg.alias - 1].order].sdoResponseQueue.get_nonblocking();
    if(sdoMsg != nullptr){
        if(sdoMsg->alias == msg.alias && sdoMsg->index == msg.index && sdoMsg->subindex == msg.subindex && sdoMsg->operation == msg.operation){
            msg.state = sdoMsg->state;
            msg.value = sdoMsg->value;
            delete sdoMsg;
            return 0;
        }else{
            sdoMsg->recycled++;
            if(sdoMsg->recycled < dofAll){
                ecats[drivers[msg.alias - 1].order].sdoResponseQueue.put(sdoMsg, QUE_PRI_HIGH);
            }else{
                delete sdoMsg;
            }
        }
    }
    return -1;
}

int DriverSDK::impClass::putDriverREGRequest(REGMsg const& msg, int const priority){
    return -1;
}

int DriverSDK::impClass::getDriverREGResponse(REGMsg& msg){
    return -1;
}

void DriverSDK::impClass::canUpdate(){
    int i = 0;
    while(i < cans.size()){
        if(cans[i].rxSwap != nullptr){
            cans[i].rxSwap->advanceNodePtr();
        }
        i++;
    }
}

void DriverSDK::impClass::rs485Update(){
    int i = 0;
    while(i < rs485s.size()){
        if(rs485s[i].rxSwap != nullptr){
            rs485s[i].rxSwap->advanceNodePtr();
        }
        i++;
    }
}

void DriverSDK::impClass::ecatUpdate(){
    int i = 0;
    while(i < rs485s.size()){
        static unsigned int count = 0xffffffff;
        static unsigned char data[64];
        if(rs485s[i].fdS < 0){
            i++;
            continue;
        }
        int length = read(rs485s[i].fdS, data, 64);
        if(length < 1){
            i++;
            continue;
        }
        count++;
        printf("Count: %8d ", count);
        int alias = rs485s[i].alias2type.begin()->first;
        ConverterDatum& channel = converters[alias - 200].rx->channels[0];
        channel.Index = count;
        channel.ID = alias;
        channel.Length = length;
        memcpy(channel.Data, data, length);
        ecats[converters[alias - 200].order].rxPDOSwaps[converters[alias - 200].domain]->advanceNodePtr();
        channel = converters[alias - 200].rx->channels[0];
        channel.Index = count;
        channel.ID = alias;
        channel.Length = length;
        memcpy(channel.Data, data, length);
        ecats[converters[alias - 200].order].rxPDOSwaps[converters[alias - 200].domain]->advanceNodePtr();
        channel = converters[alias - 200].rx->channels[0];
        channel.Index = count;
        channel.ID = alias;
        channel.Length = length;
        memcpy(channel.Data, data, length);
        ecats[converters[alias - 200].order].rxPDOSwaps[converters[alias - 200].domain]->advanceNodePtr();
        int j = 0;
        while(j < length){
            printf("%02x_", data[j]);
            j++;
        }
        printf("\b\n");
        i++;
    }
    i = 0;
    while(i < ecats.size()){
        int j = 0;
        while(j < ecats[i].domainDivisions.size()){
            if(ecats[i].rxPDOSwaps[j] != nullptr){
                ecats[i].rxPDOSwaps[j]->advanceNodePtr();
            }
            j++;
        }
        i++;
    }
}

void DriverSDK::impClass::sdoRequestableUpdate(){
    static bool initialized = false;
    if(initialized){
        int i = 0;
        while(i < ecats.size()){
            if(ecats[i].sdoRequestQueue.size() < dofAll && ecats[i].sdoResponseQueue.size() < dofAll){
                ecats[i].sdoRequestable = true;
            }else{
                ecats[i].sdoRequestable = false;
            }
            i++;
        }
    }else{
        initialized = true;
        int i = 0;
        while(i < dofAll){
            if(drivers[i].busCode == 0 && drivers[i].tx->StatusWord == 0){
                initialized = false;
            }
            i++;
        }
    }
}

DriverSDK::impClass::~impClass(){
    if(imu != nullptr){
        delete imu;
        imu = nullptr;
    }
    if(digits != nullptr){
        delete[] digits;
        digits = nullptr;
    }
    if(legs[0] != nullptr){
        delete[] legs[0];
        legs[0] = nullptr;
    }
    if(legs[1] != nullptr){
        delete[] legs[1];
        legs[1] = nullptr;
    }
    if(arms[0] != nullptr){
        delete[] arms[0];
        arms[0] = nullptr;
    }
    if(arms[1] != nullptr){
        delete[] arms[1];
        arms[1] = nullptr;
    }
    if(waist != nullptr){
        delete[] waist;
        waist = nullptr;
    }
    if(neck != nullptr){
        delete[] neck;
        neck = nullptr;
    }
    if(drivers != nullptr){
        delete[] drivers;
        drivers = nullptr;
    }
    if(configXML != nullptr){
        delete configXML;
        configXML = nullptr;
    }
}

DriverSDK& DriverSDK::instance(){
    static DriverSDK instance;
    return instance;
}

DriverSDK::DriverSDK(): imp(*new impClass()){
}

void DriverSDK::setCPU(unsigned short const cpu){
    int i = 0;
    while(i < 6){
        processorsECAT[i] = cpu;
        i++;
    }
    i = 0;
    while(i < 3){
        processorsCAN[i] = cpu;
        i++;
    }
}

int DriverSDK::setCPUs(std::vector<unsigned short> const& cpus, std::string const& bus){
    if(bus == "ECAT" && cpus.size() == 6){
        processorsECAT = cpus;
    }else if(bus == "CAN" && cpus.size() == 3){
        processorsCAN = cpus;
    }else{
        return -1;
    }
    return 0;
}

void DriverSDK::setMaxCurr(std::vector<unsigned short> const& maxCurr){
    maxCurrent = maxCurr;
}

int DriverSDK::setMode(std::vector<char> const& mode){
    if(operatingMode.size() == 0){
        operatingMode = mode;
        return 0;
    }else if(operatingMode.size() != mode.size()){
        return -1;
    }
    int i = 0;
    while(i < dofAll){
        operatingMode[i] = mode[i];
        i++;
    }
    return 0;
}

void DriverSDK::init(char const* xmlFile){
    if(imp.init(xmlFile) < 0){
        printf("imp init failed\n");
        exit(-1);
    }
}

int DriverSDK::getLeftDigitNr(){
    return dofLeftEffector;
}

int DriverSDK::getRightDigitNr(){
    return dofRightEffector;
}

int DriverSDK::getTotalMotorNr(){
    return dofAll;
}

std::vector<int> DriverSDK::getActiveMotors(){
    std::vector<int> ret;
    int i = 0;
    while(i < ecatAlias2type.size()){
        auto itr = ecatAlias2type[i].begin();
        while(itr != ecatAlias2type[i].end()){
            if(itr->first <= dofAll){
                ret.push_back(itr->first - 1);
            }
            itr++;
        }
        i++;
    }
    i = 0;
    while(i < canAlias2type.size()){
        auto itr = canAlias2type[i].begin();
        while(itr != canAlias2type[i].end()){
            if(itr->first <= dofAll){
                ret.push_back(itr->first - 1);
            }
            itr++;
        }
        i++;
    }
    return ret;
}

int DriverSDK::setCntBias(std::vector<int> const& cntBias){
    if(cntBias.size() != dofAll){
        return -1;
    }
    int i = 0;
    while(i < dofAll){
        drivers[i].parameters.countBias = cntBias[i];
        i++;
    }
    return 0;
}

int DriverSDK::fillSDO(motorSDOClass& data, char const* object){
    if(configXML == nullptr || data.i < 0 || data.i >= dofAll){
        return -1;
    }
    std::map<int, std::string>::iterator itr;
    int i = 0;
    while(i < ecatAlias2type.size()){
        itr = ecatAlias2type[i].begin();
        while(itr != ecatAlias2type[i].end()){
            if(itr->first == data.i + 1){
                i = ecatAlias2type.size() - 1;
                break;
            }
            itr++;
        }
        i++;
    }
    if(ecatAlias2type.size() == 0 || itr == ecatAlias2type[ecatAlias2type.size() - 1].end()){
        data.value    = 0;
        data.state    = 3;
        data.index    = 0x0000;
        data.subindex = 0x00;
        return 1;
    }
    std::vector<std::string> entry = configXML->entry(configXML->device("ECAT", itr->second.c_str()), object);
    data.value     = 0;
    data.state     = 0;
    data.index     = (unsigned short)strtoul(entry[1].c_str(), nullptr, 16);
    data.subindex  = (unsigned char )strtoul(entry[2].c_str(), nullptr, 16);
    data.signed_   = (unsigned char )strtoul(entry[3].c_str(), nullptr, 10);
    data.bitLength = (unsigned char )strtoul(entry[4].c_str(), nullptr, 10);
    data.operation = (unsigned char )strtoul(entry[5].c_str(), nullptr, 10);
    return 0;
}

void DriverSDK::getIMU(imuStruct& data){
    float f = imp.imu->rpy0(imp.imu->txSwap);
    if(f > -4.0 && f < 4.0){
        data.rpy[0] = f;
    }
    f = imp.imu->rpy1(imp.imu->txSwap);
    if(f > -4.0 && f < 4.0){
        data.rpy[1] = f;
    }
    f = imp.imu->rpy2(imp.imu->txSwap);
    if(f > -4.0 && f < 4.0){
        data.rpy[2] = f;
    }
    f = imp.imu->gyr0(imp.imu->txSwap);
    if(f > -40.0 && f < 40.0){
        data.gyr[0] = f;
    }
    f = imp.imu->gyr1(imp.imu->txSwap);
    if(f > -40.0 && f < 40.0){
        data.gyr[1] = f;
    }
    f = imp.imu->gyr2(imp.imu->txSwap);
    if(f > -40.0 && f < 40.0){
        data.gyr[2] = f;
    }
    data.acc[0] = imp.imu->acc0(imp.imu->txSwap);
    data.acc[1] = imp.imu->acc1(imp.imu->txSwap);
    data.acc[2] = imp.imu->acc2(imp.imu->txSwap);
}

int DriverSDK::getSensor(std::vector<sensorStruct>& data){
    if(data.size() != 2){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < 2){
        if(sensors[i].order < 0){
            data[i].statusCode = 0xffff;
            i++;
            continue;
        }
        data[i].F[0]       = sensors[i].tx->Fx / 10000.0;
        data[i].F[1]       = sensors[i].tx->Fy / 10000.0;
        data[i].F[2]       = sensors[i].tx->Fz / 10000.0;
        data[i].M[0]       = sensors[i].tx->Mx / 10000.0;
        data[i].M[1]       = sensors[i].tx->My / 10000.0;
        data[i].M[2]       = sensors[i].tx->Mz / 10000.0;
        data[i].statusCode = sensors[i].tx->StatusCode;
        i++;
    }
    return 0;
}

int DriverSDK::setDigitTarget(std::vector<digitTargetStruct> const& data){
    if(data.size() != dofEffector){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < dofEffector){
        if(data[i].pos > 90){
            digits[i].rx->TargetPosition = 90;
        }else{
            digits[i].rx->TargetPosition = data[i].pos;
        }
        i++;
    }
    imp.rs485Update();
    return 0;
}

int DriverSDK::getDigitActual(std::vector<digitActualStruct>& data){
    if(data.size() != dofEffector){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < dofEffector){
        data[i].pos = digits[i].tx->ActualPosition;
        i++;
    }
    return 0;
}

int DriverSDK::setMotorTarget(std::vector<motorTargetStruct> const& data){
    if(data.size() != dofAll){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < dofAll){
        if(drivers[i].order < 0){
            i++;
            continue;
        }
        if(drivers[i].busCode == 0){
            float position = data[i].pos;
            if(position < drivers[i].parameters.minimumPosition){
                position = drivers[i].parameters.minimumPosition;
            }else if(position > drivers[i].parameters.maximumPosition){
                position = drivers[i].parameters.maximumPosition;
            }
            position = drivers[i].parameters.polarity * position * drivers[i].parameters.gearRatioPosVel * drivers[i].parameters.encoderResolution / 2.0 / Pi + drivers[i].parameters.countBias;
            drivers[i].rx->TargetPosition = position;
            float velocity = data[i].vel;
            velocity = drivers[i].parameters.polarity * velocity * drivers[i].parameters.gearRatioPosVel * drivers[i].parameters.encoderResolution / 2.0 / Pi;
            drivers[i].rx->TargetVelocity = velocity;
            drivers[i].rx->VelocityOffset = velocity;
            float torque = data[i].tor;
            if(torque > drivers[i].parameters.maximumTorque){
                torque = drivers[i].parameters.maximumTorque;
            }else if(torque < -drivers[i].parameters.maximumTorque){
                torque = -drivers[i].parameters.maximumTorque;
            }
            torque = drivers[i].parameters.polarity * 1000.0 * torque / drivers[i].parameters.torqueConstant / drivers[i].parameters.gearRatioTor / drivers[i].parameters.ratedCurrent;
            if(operatingMode[i] == 8){
                drivers[i].rx->TargetTorque = 0;
                drivers[i].rx->TorqueOffset = torque;
            }else if(operatingMode[i] == 10){
                drivers[i].rx->TargetTorque = torque;
                drivers[i].rx->TorqueOffset = 0;
            }
            drivers[i].enabled = data[i].enabled;
        }else if(drivers[i].busCode == 1){
            float position = data[i].pos;
            if(position < drivers[i].parameters.minimumPosition){
                position = drivers[i].parameters.minimumPosition;
            }else if(position > drivers[i].parameters.maximumPosition){
                position = drivers[i].parameters.maximumPosition;
            }
            position = drivers[i].parameters.polarity * position + drivers[i].parameters.countBias;
            *reinterpret_cast<float*>(&drivers[i].rx->TargetPosition) = position;
            float velocity = data[i].vel;
            velocity = drivers[i].parameters.polarity * velocity;
            *reinterpret_cast<float*>(&drivers[i].rx->TargetVelocity) = velocity;
            float torque = data[i].tor;
            if(torque > drivers[i].parameters.maximumTorque){
                torque = drivers[i].parameters.maximumTorque;
            }else if(torque < -drivers[i].parameters.maximumTorque){
                torque = -drivers[i].parameters.maximumTorque;
            }
            torque = drivers[i].parameters.polarity * torque;
            drivers[i].rx->TorqueOffset = single2half(torque);
            drivers[i].rx->ControlWord = single2half(data[i].kp);
            drivers[i].rx->TargetTorque = single2half(data[i].kd);
            drivers[i].rx->Undefined = data[i].enabled;
            drivers[i].enabled = data[i].enabled;
        }
        i++;
    }
    i = 0;
    while(i < dofAll){
        if(drivers[i].order < 0){
            i++;
            continue;
        }
        if(drivers[i].busCode == 0){
            switch(drivers[i].enabled){
            case 1:
                switch(drivers[i].tx->StatusWord & 0x007f){
                case 0x0031:
                    drivers[i].rx->Mode = operatingMode[i];
                    drivers[i].rx->ControlWord = 0x07;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->Mode = operatingMode[i];
                    drivers[i].rx->ControlWord = 0x07;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->Mode = operatingMode[i];
                    drivers[i].rx->ControlWord = 0x07;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    break;
                case 0x0033:
                    drivers[i].rx->ControlWord = 0x0f;
                    drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->ControlWord = 0x0f;
                    drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->ControlWord = 0x0f;
                    drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    break;
                case 0x0037:
                    drivers[i].rx->Mode = operatingMode[i];
                    break;
                default:
                    drivers[i].rx->ControlWord = 0x06;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->ControlWord = 0x06;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                    drivers[i].rx->ControlWord = 0x06;
                    imp.ecats[drivers[i].order].rxPDOSwaps[drivers[i].domain]->advanceNodePtr();
                }
                break;
            case 0:
                drivers[i].rx->ControlWord = 0x06;
                break;
            case -1:
                drivers[i].rx->ControlWord = 0x86;
                imp.putDriverSDORequest(drivers[i].parameters.clearErrorSDO);
                if(imp.getDriverSDOResponse(drivers[i].parameters.clearErrorSDO) == 0){
                    if(drivers[i].parameters.clearErrorSDO.state < 0){
                        printf("requesting drivers[%d] clearError failed\n", i);
                    }
                }
                break;
            }
        }
        i++;
    }
    imp.ecatUpdate();
    imp.canUpdate();
    return 0;
}

int DriverSDK::getMotorActual(std::vector<motorActualStruct>& data){
    if(data.size() != dofAll){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < dofAll){
        if(drivers[i].order < 0){
            data[i].statusWord = 0xffff;
            i++;
            continue;
        }
        if(drivers[i].busCode == 0){
            imp.putDriverSDORequest(drivers[i].parameters.temperatureSDO);
            if(imp.getDriverSDOResponse(drivers[i].parameters.temperatureSDO) == 0){
                if(drivers[i].parameters.temperatureSDO.state < 0){
                    printf("requesting drivers[%d] temperature failed\n", i);
                }else{
                    data[i].temp = drivers[i].parameters.temperatureSDO.value;
                }
            }
            data[i].pos = 2.0 * Pi * drivers[i].parameters.polarity * (drivers[i].tx->ActualPosition - drivers[i].parameters.countBias) / drivers[i].parameters.encoderResolution / drivers[i].parameters.gearRatioPosVel;
            data[i].vel = 2.0 * Pi * drivers[i].parameters.polarity * drivers[i].tx->ActualVelocity / drivers[i].parameters.encoderResolution / drivers[i].parameters.gearRatioPosVel;
            data[i].tor = drivers[i].parameters.polarity * drivers[i].tx->ActualTorque / 1000.0 * drivers[i].parameters.ratedCurrent * drivers[i].parameters.torqueConstant * drivers[i].parameters.gearRatioTor;
            data[i].statusWord = drivers[i].tx->StatusWord;
            data[i].errorCode = drivers[i].tx->ErrorCode;
        }else if(drivers[i].busCode == 1){
            data[i].pos        = drivers[i].parameters.polarity * (*reinterpret_cast<float*>(&drivers[i].tx->ActualPosition) - drivers[i].parameters.countBias);
            data[i].vel        = drivers[i].parameters.polarity *  *reinterpret_cast<float*>(&drivers[i].tx->ActualVelocity);
            data[i].tor        = drivers[i].parameters.polarity *                half2single( drivers[i].tx->ActualTorque  );
            data[i].temp       =                                                              drivers[i].tx->Undefined;
            data[i].statusWord =                                                              drivers[i].tx->StatusWord;
            data[i].errorCode  =                                                              drivers[i].tx->ErrorCode;
        }
        i++;
    }
    imp.sdoRequestableUpdate();
    return -ecatStalled.load();
}

int DriverSDK::getEncoderCount(std::vector<int>& data){
    if(data.size() != dofAll){
        return std::numeric_limits<int>::min();
    }
    int i = 0;
    while(i < dofAll){
        data[i] = drivers[i].tx->ActualPosition;
        i++;
    }
    return 0;
}

int DriverSDK::sendMotorSDORequest(motorSDOClass const& data){
#ifndef NIIC
    if(drivers[data.i].sdoHandler == nullptr || data.index == 0x0000){
#else
    if(drivers[data.i].slave == -1 || data.index == 0x0000){
#endif
        return 1;
    }
    drivers[data.i].parameters.sdoTemplate.index     = data.index;
    drivers[data.i].parameters.sdoTemplate.subindex  = data.subindex;
    drivers[data.i].parameters.sdoTemplate.signed_   = data.signed_;
    drivers[data.i].parameters.sdoTemplate.bitLength = data.bitLength;
    drivers[data.i].parameters.sdoTemplate.operation = data.operation;
    return imp.putDriverSDORequest(drivers[data.i].parameters.sdoTemplate);
}

int DriverSDK::recvMotorSDOResponse(motorSDOClass& data){
#ifndef NIIC
    if(drivers[data.i].sdoHandler == nullptr || data.index == 0x0000){
#else
    if(drivers[data.i].slave == -1 || data.index == 0x0000){
#endif
        return 1;
    }
    drivers[data.i].parameters.sdoTemplate.index     = data.index;
    drivers[data.i].parameters.sdoTemplate.subindex  = data.subindex;
    drivers[data.i].parameters.sdoTemplate.operation = data.operation;
    int ret = imp.getDriverSDOResponse(drivers[data.i].parameters.sdoTemplate);
    if(ret == 0){
        data.state = drivers[data.i].parameters.sdoTemplate.state;
        if(data.state < 0){
            printf("requesting drivers[%d] 0x%4x 0x%2x failed\n", data.i, data.index, data.subindex);
        }else{
            data.value = drivers[data.i].parameters.sdoTemplate.value;
        }
    }
    return ret;
}

int DriverSDK::sendMotorREGRequest(motorREGClass const& data){
    return 0;
}

int DriverSDK::recvMotorREGResponse(motorREGClass& data){
    return 0;
}

int DriverSDK::calibrate(int const i){
    if(drivers[i].busCode != 0 || drivers[i].order != 0){
        return std::numeric_limits<int>::max();
    }
    motorSDOClass data(i);
    if(fillSDO(data, "ActualPosition") != 0){
        return std::numeric_limits<int>::min();
    }
    long period = configXML->masterAttribute("ECAT", drivers[i].order, "period");
    int tryCount = 0;
    while(sendMotorSDORequest(data) != 0){
        tryCount++;
        if(tryCount > 2 * dofAll * dofAll){
            return std::numeric_limits<int>::min();
        }
        recvMotorSDOResponse(data);
        usleep(period / 1000);
        imp.sdoRequestableUpdate();
    }
    tryCount = 0;
    while(recvMotorSDOResponse(data) != 0){
        tryCount++;
        if(tryCount > 2 * dofAll * dofAll){
            return std::numeric_limits<int>::min();
        }
        usleep(period / 1000);
    }
    if(data.state < 0){
        return std::numeric_limits<int>::min();
    }
    if(configXML->writeMotorParameter(i + 1, "CountBias", data.value) != 0){
        return std::numeric_limits<int>::min();
    }
    if(configXML->readMotorParameter(i + 1, "CountBias") != data.value){
        return std::numeric_limits<int>::min();
    }
    configXML->save();
    drivers[i].parameters.countBias = data.value;
    return data.value;
}

void DriverSDK::advance(){
    imp.ecatUpdate();
    imp.canUpdate();
}

std::string DriverSDK::version(){
    std::stringstream ver;
    ver << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_PATCH;
    return ver.str();
}

DriverSDK::~DriverSDK(){
    delete &imp;
}
}