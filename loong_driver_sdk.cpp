﻿/* Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司
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
#include "serial.h"
#include "rs485.h"
#include "ecat.h"
#include <unistd.h>
#include <sstream>
#include <limits>

namespace DriverSDK{
ConfigXML* configXML;
std::vector<std::map<int, std::string>> rs485Alias2type, ecatAlias2type;
int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
WrapperPair<DriverRxData, DriverTxData, MotorParameters>** legs[2], ** arms[2], ** waist, ** neck;
WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
char operatingMode;
unsigned short processor;
std::vector<unsigned short> maxCurrent;

motorSDOClass::motorSDOClass(int i){
    this->i = i;
}

motorSDOClass::~motorSDOClass(){
}

class DriverSDK::impClass{
public:
    IMU* imu;
    std::vector<RS485> rs485s;
    std::vector<ECAT> ecats;
    impClass();
    int effectorCheck(std::vector<std::map<int, std::string>> alias2type, char const* bus);
    int init(char const* xmlFile);
    int putDriverSDORequest(SDOMsg const& msg, int const priority = QUE_PRI_LOW);
    int getDriverSDOResponse(SDOMsg& msg);
    void rs485Update();
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
    operatingMode = 8;
    processor = sysconf(_SC_NPROCESSORS_ONLN) - 1;
    imu = nullptr;
    rs485s.reserve(4);
    ecats.reserve(4);
}

int DriverSDK::impClass::effectorCheck(std::vector<std::map<int, std::string>> alias2type, char const* bus){
    int i = 0;
    while(i < alias2type.size()){
        auto itr = alias2type[i].begin();
        while(itr != alias2type[i].end()){
            if(itr->first == 200 || itr->first == 201){
                int dof = configXML->dof(bus, itr->second.c_str());
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

int DriverSDK::impClass::init(char const* xmlFile){
    configXML = new ConfigXML(xmlFile);
    std::vector<std::vector<int>> limbAlias = configXML->limbAlias();
    if(limbAlias.size() < 1){
        printf("there should be one limb at least\n");
        return -1;
    }
    if(limbAlias[0].size() != limbAlias[1].size()){
        printf("the two legs does not mirror each other\n");
        return -1;
    }
    if(limbAlias[2].size() != limbAlias[3].size()){
        printf("the two arms does not mirror each other\n");
        return -1;
    }
    dofLeg = limbAlias[0].size();
    dofArm = limbAlias[2].size();
    dofWaist = limbAlias[4].size();
    dofNeck = limbAlias[5].size();
    dofAll = 2 * dofLeg + 2 * dofArm + dofWaist + dofNeck;
    if(dofAll > 0){
        drivers = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>[dofAll];
    }
    if(dofLeg > 0){
        legs[0] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofLeg];
        legs[1] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofLeg];
    }
    if(dofArm > 0){
        arms[0] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofArm];
        arms[1] = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofArm];
    }
    if(dofWaist > 0){
        waist = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofWaist];
    }
    if(dofNeck > 0){
        neck = new WrapperPair<DriverRxData, DriverTxData, MotorParameters>*[dofNeck];
    }
    rs485Alias2type = configXML->alias2type("RS485");
    ecatAlias2type = configXML->alias2type("ECAT");
    if(effectorCheck(rs485Alias2type, "RS485") != 0 || effectorCheck(ecatAlias2type, "ECAT") != 0){
        printf("invalid effector configuration\n");
        return -1;
    }
    printf("dofLeftEffector %d, dofRightEffector %d, dofEffector %d\n", dofLeftEffector, dofRightEffector, dofEffector);
    if(dofEffector > 0){
        digits = new WrapperPair<DigitRxData, DigitTxData, EffectorParameters>[dofEffector];
    }
    int i = 0;
    if(maxCurrent.size() == 0){
        while(i < dofAll){
            maxCurrent.push_back(1000);
            i++;
        }
    }else if(maxCurrent.size() != dofAll){
        printf("invalid maxCurrent\n");
        return -1;
    }
    imu = new IMU(configXML->imuDevice().c_str(), configXML->imuBaudrate(), 50, 0xfa, 0xff);
    if(imu->run() < 0){
        printf("imu run failed\n");
        return -1;
    }
    if(imu->pth > 0 && pthread_detach(imu->pth) != 0){
        printf("detaching imu serialRead thread failed\n");
        return -1;
    }
    i = 0;
    while(i < rs485Alias2type.size()){
        rs485s.emplace_back(i, configXML->device("RS485", i).c_str());
        printf("rs485s[%d] created\n", i);
        i++;
    }
    i = 0;
    while(i < rs485s.size()){
        if(rs485s[i].config() < 0){
            printf("rs485s[%d] config failed\n", i);
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
    while(i < limbAlias.size()){
        printf("limb %d\n", i);
        int j = 0;
        while(j < limbAlias[i].size()){
            int alias = limbAlias[i][j];
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
    while(i < rs485s.size()){
        if(rs485s[i].run() < 0){
            printf("rs485s[%d] run failed\n", i);
            return -1;
        }
        i++;
    }
    i = 0;
    while(i < rs485s.size()){
        if(rs485s[i].pth > 0 && pthread_detach(rs485s[i].pth) != 0){
            printf("detaching rs485s[%d] rxtx thread failed\n", i);
            return -1;
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
    i = 0;
    while(i < ecats.size()){
        if(ecats[i].pth > 0 && pthread_detach(ecats[i].pth) != 0){
            printf("detaching ecats[%d] rxtx thread failed\n", i);
            return -1;
        }
        i++;
    }
    return 0;
}

int DriverSDK::impClass::putDriverSDORequest(SDOMsg const& msg, int const priority){
    if(ecats[drivers[msg.alias - 1].order].sdoRequestable && drivers[msg.alias - 1].tx->StatusWord > 0){
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
    while(i < dofAll){
        if(drivers[i].order < 0){
            i++;
            continue;
        }
        switch(drivers[i].enabled){
        case 1:
            switch(drivers[i].tx->StatusWord & 0x007f){
            case 0x0031:
                drivers[i].rx->Mode = operatingMode;
                drivers[i].rx->ControlWord = 0x07;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->Mode = operatingMode;
                drivers[i].rx->ControlWord = 0x07;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->Mode = operatingMode;
                drivers[i].rx->ControlWord = 0x07;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                break;
            case 0x0033:
                drivers[i].rx->ControlWord = 0x0f;
                drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->ControlWord = 0x0f;
                drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->ControlWord = 0x0f;
                drivers[i].rx->TargetPosition = drivers[i].tx->ActualPosition;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                break;
            case 0x0037:
                break;
            default:
                drivers[i].rx->ControlWord = 0x06;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->ControlWord = 0x06;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
                drivers[i].rx->ControlWord = 0x06;
                ecats[drivers[i].order].rxPDOSwap->advanceNodePtr();
            }
            break;
        case 0:
            drivers[i].rx->ControlWord = 0x06;
            break;
        case -1:
            drivers[i].rx->ControlWord = 0x86;
            putDriverSDORequest(drivers[i].parameters.clearErrorSDO);
            if(getDriverSDOResponse(drivers[i].parameters.clearErrorSDO) == 0){
                if(drivers[i].parameters.clearErrorSDO.state < 0){
                    printf("requesting drivers[%d] clearError failed\n", i);
                }
            }
            break;
        }
        i++;
    }
    i = 0;
    while(i < ecats.size()){
        if(ecats[i].rxPDOSwap != nullptr){
            ecats[i].rxPDOSwap->advanceNodePtr();
        }
        i++;
    }
}

void DriverSDK::impClass::sdoRequestableUpdate(){
    int i = 0;
    while(i < ecats.size()){
        if(ecats[i].sdoRequestQueue.size() < dofAll && ecats[i].sdoResponseQueue.size() < dofAll){
            ecats[i].sdoRequestable = true;
        }else{
            ecats[i].sdoRequestable = false;
        }
        i++;
    }
}

DriverSDK::impClass::~impClass(){
    if(imu != nullptr){
        delete imu;
    }
    if(digits != nullptr){
        delete[] digits;
    }
    if(legs[0] != nullptr){
        delete[] legs[0];
    }
    if(legs[1] != nullptr){
        delete[] legs[1];
    }
    if(arms[0] != nullptr){
        delete[] arms[0];
    }
    if(arms[1] != nullptr){
        delete[] arms[1];
    }
    if(waist != nullptr){
        delete[] waist;
    }
    if(neck != nullptr){
        delete[] neck;
    }
    if(drivers != nullptr){
        delete[] drivers;
    }
    if(configXML != nullptr){
        delete configXML;
    }
}

DriverSDK& DriverSDK::instance(){
    static DriverSDK instance;
    return instance;
}

DriverSDK::DriverSDK(): imp(*new impClass()){
}

void DriverSDK::setCPU(unsigned short const cpu){
    processor = cpu;
}

void DriverSDK::setMode(char const mode){
    operatingMode = mode;
}

void DriverSDK::setMaxCurr(std::vector<unsigned short> const& maxCurr){
    maxCurrent = maxCurr;
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
    while(i < rs485Alias2type.size()){
        auto itr = rs485Alias2type[i].begin();
        while(itr != rs485Alias2type[i].end()){
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
    if(configXML == nullptr || data.i >= dofAll || data.i < 0){
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
        data.value = 0;
        data.state = 3;
        data.index = 0x0000;
        data.subindex = 0x00;
        return 1;
    }
    std::vector<std::string> entry = configXML->entry(configXML->busDevice("ECAT", itr->second.c_str()), object);
    data.value = 0;
    data.state = 0;
    data.index = (unsigned short)strtoul(entry[1].c_str(), nullptr, 16);
    data.subindex = (unsigned char)strtoul(entry[2].c_str(), nullptr, 16);
    data.signed_ = (unsigned char)strtoul(entry[3].c_str(), nullptr, 10);
    data.bitLength = (unsigned char)strtoul(entry[4].c_str(), nullptr, 10);
    data.operation = (unsigned char)strtoul(entry[5].c_str(), nullptr, 10);
    return 0;
}

void DriverSDK::getIMU(imuStruct& data){
    float f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 7) * Pi / 180.0;
    if(f > -4.0 && f < 4.0){
        data.rpy[0] = f;
    }
    f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 11) * Pi / 180.0;
    if(f > -4.0 && f < 4.0){
        data.rpy[1] = f;
    }
    f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 15) * Pi / 180.0;
    if(f > -4.0 && f < 4.0){
        data.rpy[2] = f;
    }
    f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 37);
    if(f > -40.0 && f < 40.0){
        data.gyr[0] = f;
    }
    f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 41);
    if(f > -40.0 && f < 40.0){
        data.gyr[1] = f;
    }
    f = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 45);
    if(f > -40.0 && f < 40.0){
        data.gyr[2] = f;
    }
    data.acc[0] = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 22);
    data.acc[1] = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 26);
    data.acc[2] = imp.imu->quadchar2float(imp.imu->txSwap->nodePtr.load()->memPtr + 30);
}

int DriverSDK::getSensor(std::vector<sensorStruct>& data){
    if(data.size() != 2){
        return -1;
    }
    int i = 0;
    while(i < 2){
        data[i].F[0] = (float)sensors[i].tx->Fx / 10000.0;
        data[i].F[1] = (float)sensors[i].tx->Fy / 10000.0;
        data[i].F[2] = (float)sensors[i].tx->Fz / 10000.0;
        data[i].M[0] = (float)sensors[i].tx->Mx / 10000.0;
        data[i].M[1] = (float)sensors[i].tx->My / 10000.0;
        data[i].M[2] = (float)sensors[i].tx->Mz / 10000.0;
        i++;
    }
    return 0;
}

int DriverSDK::setDigitTarget(std::vector<digitTargetStruct> const& data){
    if(data.size() != dofEffector){
        return -1;
    }
    int i = 0;
    while(i < dofEffector){
        if(data[i].pos > 9000){
            digits[i].rx->TargetPosition = 9000;
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
        return -1;
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
        return -1;
    }
    int i = 0;
    while(i < dofAll){
        if(drivers[i].order < 0){
            i++;
            continue;
        }
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
        torque = drivers[i].parameters.polarity * 1000.0 * torque / drivers[i].parameters.torqueConstant / drivers[i].parameters.gearRatioTor / std::sqrt(2.0) / drivers[i].parameters.ratedCurrent;
        drivers[i].rx->TargetTorque = torque;
        drivers[i].rx->TorqueOffset = torque;
        drivers[i].enabled = data[i].enabled;
        i++;
    }
    imp.ecatUpdate();
    return 0;
}

int DriverSDK::getMotorActual(std::vector<motorActualStruct>& data){
    if(data.size() != dofAll){
        return -1;
    }
    int i = 0;
    while(i < dofAll){
        if(drivers[i].order < 0){
            data[i].statusWord = -1;
            i++;
            continue;
        }
        imp.putDriverSDORequest(drivers[i].parameters.temperatureSDO);
        data[i].pos = 2.0 * Pi * drivers[i].parameters.polarity * (drivers[i].tx->ActualPosition - drivers[i].parameters.countBias) / drivers[i].parameters.encoderResolution / drivers[i].parameters.gearRatioPosVel;
        data[i].vel = 2.0 * Pi * drivers[i].parameters.polarity * drivers[i].tx->ActualVelocity / drivers[i].parameters.encoderResolution / drivers[i].parameters.gearRatioPosVel;
        data[i].tor = drivers[i].parameters.polarity * drivers[i].tx->ActualTorque / 1000.0 * std::sqrt(2.0) * drivers[i].parameters.ratedCurrent * drivers[i].parameters.torqueConstant * drivers[i].parameters.gearRatioTor;
        if(imp.getDriverSDOResponse(drivers[i].parameters.temperatureSDO) == 0){
            if(drivers[i].parameters.temperatureSDO.state < 0){
                printf("requesting drivers[%d] temperature failed\n", i);
            }else{
                data[i].temp = drivers[i].parameters.temperatureSDO.value;
            }
        }
        data[i].statusWord = drivers[i].tx->StatusWord;
        data[i].errorCode = drivers[i].tx->ErrorCode;
        i++;
    }
    imp.sdoRequestableUpdate();
    return 0;
}

int DriverSDK::sendMotorSDORequest(motorSDOClass const& data){
    if(drivers[data.i].sdoHandler == nullptr || data.index == 0x0000){
        return 1;
    }
    drivers[data.i].parameters.sdoTemplate.index = data.index;
    drivers[data.i].parameters.sdoTemplate.subindex = data.subindex;
    drivers[data.i].parameters.sdoTemplate.signed_ = data.signed_;
    drivers[data.i].parameters.sdoTemplate.bitLength = data.bitLength;
    drivers[data.i].parameters.sdoTemplate.operation = data.operation;
    return imp.putDriverSDORequest(drivers[data.i].parameters.sdoTemplate);
}

int DriverSDK::recvMotorSDOResponse(motorSDOClass& data){
    if(drivers[data.i].sdoHandler == nullptr || data.index == 0x0000){
        return 1;
    }
    drivers[data.i].parameters.sdoTemplate.index = data.index;
    drivers[data.i].parameters.sdoTemplate.subindex = data.subindex;
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

int DriverSDK::calibrate(int const i){
    if(drivers[i].order != 0){
        return std::numeric_limits<int>::max();
    }
    motorSDOClass data(i);
    if(fillSDO(data, "ActualPosition") != 0){
        return std::numeric_limits<int>::min();
    }
    long period = configXML->period("ECAT", drivers[i].order);
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

std::string DriverSDK::version(){
    std::stringstream ver;
    ver << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_PATCH;
    return ver.str();
}

DriverSDK::~DriverSDK(){
    delete &imp;
}
}