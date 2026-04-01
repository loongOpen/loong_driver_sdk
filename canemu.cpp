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
#include "canbase.h"

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> canEmuAlias2type;
extern std::vector<std::map<int, std::vector<int>>> canEmuAlias2masterIDs;
extern std::vector<std::map<int, int>> canEmuAlias2slaveID;
extern int dofEffector;

void nullCANEmuTX(int const order, int const masterID, unsigned char* const data, int const length, CANEmu* const canemu){
    printf("unexpected data with master_id %d and length %d on canemus[%d]\n", masterID, length, order);
}

std::map<std::string, DriverParameters*> CANEmu::type2parameters;
unsigned short* CANEmu::alias2status;
DriverParameters** CANEmu::alias2parameters;
int CANEmu::orderSlaveID2alias[8][256];
int CANEmu::orderMasterID2slaveID[8][2048];
canEmuRXFunction CANEmu::rxFuncs[8][256];
canEmuTXFunction CANEmu::txFuncs[8][2048];
int CANEmu::alias2channel[256];

CANEmu::CANEmu(int const order) : CANBase(order, "/dev/null"){
    static bool initialized = false;
    if(!initialized){
        type2parameters.clear();
        alias2status = new unsigned short[dofAll + 1];
        alias2parameters = new DriverParameters*[dofAll + 1];
        int i = 0, j;
        while(i <= dofAll){
            alias2status[i] = 0xffff;
            alias2parameters[i] = nullptr;
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 2048){
                orderMasterID2slaveID[i][j] = -1;
                j++;
            }
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 256){
                orderSlaveID2alias[i][j] = 0;
                j++;
            }
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 256){
                rxFuncs[i][j] = nullRX;
                j++;
            }
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 2048){
                txFuncs[i][j] = nullCANEmuTX;
                j++;
            }
            i++;
        }
        initialized = true;
    }
    rxSwap = txSwap = rxSwap_ = txSwap_ = nullptr;
    MASK = mask = MASK_ = mask_ = 0;
    alias2type = canEmuAlias2type[order];
    alias2masterIDs = canEmuAlias2masterIDs[order];
    alias2slaveID = canEmuAlias2slaveID[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("canemus[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        int alias = itr->first, slaveID = alias2slaveID.find(alias)->second;
        std::vector<int> masterIDs = alias2masterIDs.find(alias)->second;
        std::string const& type = itr->second;
        std::string const category = configXML->typeCategory("CAN", type.c_str());
        if(category == "driver"){
            if(type.starts_with("RealMan")){
                printf("RealMan driver not supported on canemu bus\n");
                exit(-1);
            }
            alias2status[alias] = 0x0000;
            auto itr_ = type2parameters.find(type);
            if(itr_ == type2parameters.end()){
                std::tie(itr_, std::ignore) = type2parameters.insert(std::make_pair(type, new DriverParameters()));
                itr_->second->load(itr_->first);
            }
            alias2parameters[alias] = itr_->second;
        }
        printf("\talias %d, type %s, master_ids", alias, type.c_str());
        int i = 0;
        while(i < masterIDs.size()){
            printf(" %d", masterIDs[i]);
            orderMasterID2slaveID[order][masterIDs[i]] = slaveID;
            if(txFuncs[order][masterIDs[i]] != nullCANEmuTX){
                printf("\ninvalid canemu bus configuration\n");
                exit(-1);
            }
            if(type.starts_with("Encos")){
                txFuncs[order][masterIDs[i]] = encosTX<CANEmu>;
            }else if(type.starts_with("Damiao")){
                txFuncs[order][masterIDs[i]] = damiaoTX<CANEmu>;
            }else if(type.starts_with("RealMan")){
                txFuncs[order][masterIDs[i]] = realManTX<CANEmu>;
            }else if(type == "AGIBOT"){
                txFuncs[order][masterIDs[i]] = agibotTX<CANEmu>;
            }else if(type == "LinkerBot"){
                txFuncs[order][masterIDs[i]] = linkerBotTX<CANEmu>;
            }
            i++;
        }
        printf(", slave_id %d\n", slaveID);
        orderSlaveID2alias[order][slaveID] = alias;
        if(rxFuncs[order][slaveID] != nullRX){
            printf("invalid canemu bus configuration\n");
            exit(-1);
        }
        if(type.starts_with("Encos")){
            rxFuncs[order][slaveID] = encosRX<CANEmu>;
        }else if(type.starts_with("Damiao")){
            rxFuncs[order][slaveID] = damiaoRX<CANEmu>;
        }else if(type.starts_with("RealMan")){
            rxFuncs[order][slaveID] = realManRX<CANEmu>;
        }else if(type == "AGIBOT"){
            rxFuncs[order][slaveID] = agibotRX<CANEmu>;
        }else if(type == "LinkerBot"){
            rxFuncs[order][slaveID] = linkerBotRX<CANEmu>;
        }
        itr++;
    }
    slaveCount = alias2type.size();
}

int CANEmu::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    rxSwap  = new SwapList(dofAll      * sizeof(DriverRxData));
    txSwap  = new SwapList(dofAll      * sizeof(DriverTxData));
    rxSwap_ = new SwapList(dofEffector * sizeof( DigitRxData));
    txSwap_ = new SwapList(dofEffector * sizeof( DigitTxData));
    int k = 0;
    auto itr = alias2slaveID.begin();
    while(itr != alias2slaveID.end()){
        int alias = itr->first, slave = itr->second;
        std::string const& type = alias2type.find(alias)->second;
        std::string const category = configXML->typeCategory("CAN", type.c_str());
        if(category == "driver"){
#ifndef NIIC
            if(drivers[alias - 1].init("CANEmu", 3, order, 0, slave, alias, type, k * sizeof(DriverRxData), k * sizeof(DriverTxData), nullptr, nullptr) != 0){
#else
            if(drivers[alias - 1].init("CANEmu", 3, order, 0, slave, alias, type, k * sizeof(DriverRxData), k * sizeof(DriverTxData)) != 0){
#endif
                printf("\tdrivers[%d] init failed\n", alias - 1);
                return -1;
            }
            if(drivers[alias - 1].config("CANEmu", order, 0, rxSwap, txSwap) != 0){
                printf("\tdrivers[%d] config failed\n", alias - 1);
                return -1;
            }
            MASK |= 1 << slave;
            k++;
        }else if(category == "effector"){
            int i, j;
            if(alias == 200){
                i = 0;
                j = dofLeftEffector;
            }else if(alias == 201){
                i = dofLeftEffector;
                j = dofEffector;
            }else{
                printf("\tinvalid effector alias %d\n", alias);
                return -1;
            }
            while(i < j){
#ifndef NIIC
                if(digits[i].init("CANEmu", 3, order, 0, slave, alias, type, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr, nullptr) != 0){
#else
                if(digits[i].init("CANEmu", 3, order, 0, slave, alias, type, i * sizeof(DigitRxData), i * sizeof(DigitTxData)) != 0){
#endif
                    printf("\tdigits[%d] init failed\n", i);
                    return -1;
                }
                if(digits[i].config("CANEmu", order, 0, rxSwap_, txSwap_) != 0){
                    printf("\tdigits[%d] config failed\n", i);
                    return -1;
                }
                i++;
            }
            MASK_ |= 1 << slave;
        }
        itr++;
    }
    return 0;
}

int CANEmu::run(std::vector<CANEmu>& canemus){
    int i = 0, j = 0;
    while(i < canemus.size()){
        if(canemus[i].alias2type.size() > 0){
            j++;
        }
        i++;
    }
    if(j == 0){
        return 0;
    }
    i = 0;
    while(i < canemus.size()){
        printf("canemus[%d]:\t", i);
        j = 0;
        while(j < 256){
            int a = orderSlaveID2alias[i][j];
            if(a > 0 && a <= dofAll){
                printf("%2d ", a);
            }
            j++;
        }
        printf(" MASK: 0x%08x\n", canemus[i].MASK);
        i++;
    }
    return 0;
}

CANEmu::~CANEmu(){
    std::lock_guard<std::mutex> guard(resourceMutex);
    if(alias2status != nullptr){
        delete[] alias2status;
        alias2status = nullptr;
    }
    if(alias2parameters != nullptr){
        int i = 1;
        while(i <= dofAll){
            if(alias2parameters[i] != nullptr){
                delete alias2parameters[i];
                alias2parameters[i] = nullptr;
            }
            i++;
        }
        delete[] alias2parameters;
        alias2parameters = nullptr;
    }
    if(rxSwap != nullptr){
        delete rxSwap;
        rxSwap = nullptr;
    }
    if(txSwap != nullptr){
        delete txSwap;
        txSwap = nullptr;
    }
    if(rxSwap_ != nullptr){
        delete rxSwap_;
        rxSwap_ = nullptr;
    }
    if(txSwap_ != nullptr){
        delete txSwap_;
        txSwap_ = nullptr;
    }
}
}