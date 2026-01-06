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
#include "rs485.h"
#include "ecat.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <atomic>
#include <sstream>
#include <limits>

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> ecatAlias2type;
extern std::vector<std::map<int, int>> ecatAlias2domain;
extern std::vector<std::vector<int>> ecatDomainDivisions;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
extern WrapperPair<ConverterRxData, ConverterTxData, EffectorParameters> converters[2];
extern WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
extern std::vector<unsigned short> processorsECAT;
extern std::vector<unsigned short> maxCurrent;
extern std::atomic<int> ecatStalled;
extern std::vector<RS485>* rs485sPtr;
WrapperPair<HandRxData, HandTxData, EffectorParameters> hands[2];

ECAT::ECAT(int const order){
    domainSizes = nullptr;
    rxPDOSwaps = nullptr;
    txPDOSwaps = nullptr;
    sdoMsg = nullptr;
    regMsg = nullptr;
    sdoRequestable = false;
    regRequestable = false;
    task = nullptr;
    master = nullptr;
    targetPosition = targetVelocity = targetTorque = controlWord = mode = torqueOffset = velocityOffset = actualPosition = actualVelocity = actualTorque = statusWord = modeDisplay = errorCode = nullptr;
    this->order = order;
    alias2type = ecatAlias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("ecats[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        printf("\talias %d, type %s\n", itr->first, itr->second.c_str());
        itr++;
    }
    eni    = configXML->masterDevice("ECAT", order, "eni");
    dc     = configXML->masterFeature("ECAT", order, "dc");
    period = configXML->masterAttribute("ECAT", order, "period");
    cpu    = configXML->masterAttribute("ECAT", order, "cpu");
    adjustCPU(&cpu, processorsECAT[order]);
    alias2domain = ecatAlias2domain[order];
    domainDivisions = ecatDomainDivisions[order];
    while(init() < 0){
        clean();
    }
}

int ECAT::init(){
    task = new ecat::task(order);
    if(task == nullptr){
        printf("creating task %d failed\n", order);
        return -1;
    }
    master = task->get_master_ptr();
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    task->cpu_affinity(&cpuset, sizeof(cpuset));
    task->priority(90);
    task->load_eni(eni, period);
    domainSizes = new int[domainDivisions.size()];
    rxPDOSwaps = new SwapList*[domainDivisions.size()];
    txPDOSwaps = new SwapList*[domainDivisions.size()];
    int i = 0;
    while(i < domainDivisions.size()){
        domainSizes[i] = 0;
        rxPDOSwaps[i] = nullptr;
        txPDOSwaps[i] = nullptr;
        i++;
    }
    effectorAlias = 199;
    sensorAlias = 219;
    return 0;
}

int ECAT::readAlias(unsigned short const slave, std::string const& category, unsigned short const index, unsigned char const subindex, unsigned char const bitLength){
    if(bitLength == 8){
        while(true){
            try{
                return master->sdo_upload<unsigned char>(slave, {index, subindex}, false);
            }catch(std::exception const& e){
                sleep(1);
            }
        }
    }else if(bitLength == 16){
        while(true){
            try{
                return master->sdo_upload<unsigned short>(slave, {index, subindex}, false);
            }catch(std::exception const& e){
                sleep(1);
            }
        }
    }else if(bitLength == 32){
        while(true){
            try{
                return master->sdo_upload<unsigned int>(slave, {index, subindex}, false);
            }catch(std::exception const& e){
                sleep(1);
            }
        }
    }else if(bitLength == 0){
        if(category == "effector"){
            auto itr = alias2type.end();
            do{
                effectorAlias++;
                itr = alias2type.find(effectorAlias);
            }while(itr == alias2type.end() && effectorAlias < 201);
            return effectorAlias;
        }else if(category == "sensor"){
            auto itr = alias2type.end();
            do{
                sensorAlias++;
                itr = alias2type.find(sensorAlias);
            }while(itr == alias2type.end() && sensorAlias < 221);
            return sensorAlias;
        }
    }
    return 0;
}

int ECAT::requestState(unsigned short const slave, char const* stateString){
    ecat::al_state_type state;
    if(strcmp(stateString, "INIT") == 0) {
        state = ecat::al_state_type::init;
    }else if(strcmp(stateString, "PREOP") == 0){
        state = ecat::al_state_type::preop;
    }else if(strcmp(stateString, "BOOT") == 0){
        state = ecat::al_state_type::boot;
    }else if(strcmp(stateString, "SAFEOP") == 0){
        state = ecat::al_state_type::safeop;
    }else if(strcmp(stateString, "OP") == 0){
        state = ecat::al_state_type::op;
    }else{
        printf("requesting invalid state %s\n", stateString);
        exit(-1);
    }
    try{
        master->request_state(slave, state);
    }catch(std::exception const& e){
        printf("requesting state %s failed for slave %d:%d\n", stateString, order, slave);
        sleep(1);
        return -1;
    }
    return 0;
}

int ECAT::check(){
    if(alias2type.size() == 0){
        return 0;
    }
    auto itr = alias2domain.begin();
    while(itr != alias2domain.end()){
        if(itr->second != 0){
            printf("master %d device with alias %d must be assigned to domain 0\n", order, itr->first);
            return -1;
        }
        itr++;
    }
    ecat::master_state masterState = master->state();
    if(masterState.link_up != 1){
        printf("master %d's link is down\n", order);
        return 1;
    }
    ecat::master_info masterInfo = master->get_info();
    printf("master %d, %ld device(s) in xml, %d slave(s) on bus\n", order, alias2type.size(), masterInfo.slave_count);
    alias2slave.clear();
    int i = 0;
    while(i < masterInfo.slave_count){
        printf("slave %d:%d", order, i);
        ecat::slave_info slaveInfo = master->get_slave_info(i);
        char buffVID[16], buffPC[16];
        sprintf(buffVID, "0x%08x", slaveInfo.id.vendor_id);
        sprintf(buffPC, "0x%08x", slaveInfo.id.product_code);
        printf(", vendor_id %s, product_code %s", buffVID, buffPC);
        tinyxml2::XMLElement* deviceXML = configXML->device("ECAT", buffVID, buffPC);
        if(deviceXML == nullptr){
            printf("\n\tdevice not registered in xml\n");
            i++;
            continue;
        }
        std::string type = configXML->deviceType(deviceXML);
        printf(", type %s", type.c_str());
        std::string category = configXML->typeCategory("ECAT", type.c_str());
        if(category != "driver"){
            printf("\n\tdevice not a driver\n");
            return -1;
        }
        std::vector<std::string> aliasEntry = configXML->entry(deviceXML, "Alias");
        int alias = readAlias(i, category,
            strtoul(aliasEntry[1].c_str(), nullptr, 16),
            strtoul(aliasEntry[2].c_str(), nullptr, 16),
            strtoul(aliasEntry[4].c_str(), nullptr, 10));
        printf(", category %s, alias %d\n", category.c_str(), alias);
        auto itr = alias2type.find(alias);
        if(itr == alias2type.end()){
            printf("\tdevice with the alias not found/enabled in xml\n");
            i++;
            continue;
        }
        if(type != itr->second){
            printf("\tdevice type contradicts that(%s) in xml\n", itr->second.c_str());
            return -1;
        }
        alias2slave.insert(std::make_pair(alias, i));
        i++;
    }
    if(alias2slave.size() != alias2type.size()){
        printf("master %d number of devices %ld contradicts that(%ld) in xml\n", order, alias2slave.size(), alias2type.size());
        clean();
        init();
        return 1;
    }
    return 0;
}

int ECAT::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    targetPosition = new ecat::PdoRegInfo*[alias2slave.size()];
    targetVelocity = new ecat::PdoRegInfo*[alias2slave.size()];
    targetTorque   = new ecat::PdoRegInfo*[alias2slave.size()];
    controlWord    = new ecat::PdoRegInfo*[alias2slave.size()];
    mode           = new ecat::PdoRegInfo*[alias2slave.size()];
    torqueOffset   = new ecat::PdoRegInfo*[alias2slave.size()];
    velocityOffset = new ecat::PdoRegInfo*[alias2slave.size()];
    actualPosition = new ecat::PdoRegInfo*[alias2slave.size()];
    actualVelocity = new ecat::PdoRegInfo*[alias2slave.size()];
    actualTorque   = new ecat::PdoRegInfo*[alias2slave.size()];
    statusWord     = new ecat::PdoRegInfo*[alias2slave.size()];
    modeDisplay    = new ecat::PdoRegInfo*[alias2slave.size()];
    errorCode      = new ecat::PdoRegInfo*[alias2slave.size()];
    int count = 0, rxPDOOffsets[alias2slave.size()], txPDOOffsets[alias2slave.size()];
    auto itr = alias2slave.begin();
    while(itr != alias2slave.end()){
        int alias = itr->first, slave = itr->second;
        std::string type = alias2type.find(alias)->second, category = configXML->typeCategory("ECAT", type.c_str());
        printf("master %d, domain 0, slave %d, alias %d, category %s, type %s\n", order, slave, alias, category.c_str(), type.c_str());
        if(task->profile_no(slave) != 402 || task->slots_count(slave) > 1){
            printf("\tdriver must be CiA402 single-axis\n");
            return -1;
        }
        tinyxml2::XMLElement* deviceXML = configXML->device("ECAT", type.c_str());
        std::vector<std::vector<std::string>> rxPDOs = configXML->pdos(deviceXML, "RxPDOs"), txPDOs = configXML->pdos(deviceXML, "TxPDOs");
        int i = 0, rxPDOCount = 0, txPDOCount = 0;
        while(i < rxPDOs.size()){
            rxPDOCount += rxPDOs[i].size() - 1;
            i++;
        }
        i = 0;
        while(i < txPDOs.size()){
            txPDOCount += txPDOs[i].size() - 1;
            i++;
        }
        std::tuple<unsigned short, unsigned char, unsigned char> pdoEntries[rxPDOCount + txPDOCount];
        int j = 0, k = 0;
        i = 0;
        while(i < rxPDOs.size()){
            j = 1;
            while(j < rxPDOs[i].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, rxPDOs[i][j].c_str());
                pdoEntries[k] = std::make_tuple(
                    (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                    (unsigned char )strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char )strtoul(entry[4].c_str(), nullptr, 10)
                );
                j++;
                k++;
            }
            i++;
        }
        i = 0;
        while(i < txPDOs.size()){
            j = 1;
            while(j < txPDOs[i].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, txPDOs[i][j].c_str());
                pdoEntries[k] = std::make_tuple(
                    (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                    (unsigned char )strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char )strtoul(entry[4].c_str(), nullptr, 10)
                );
                j++;
                k++;
            }
            i++;
        }
        rxPDOOffsets[count] = 0;
        txPDOOffsets[count] = 0;
        k = 0;
        ecat::pdo_cfg* pdoCfg = task->get_slot_rx_config(slave, 0);
        for(ecat::pdo_entry_info const& pdoEntry : pdoCfg->entries){
            unsigned short index = 0;
            unsigned char subindex = 0, bit_length = 0;
            std::tie(index, subindex, bit_length) = pdoEntries[k];
            if(pdoEntry.entry_idx.idx != index || pdoEntry.entry_idx.sub_idx != subindex || pdoEntry.bit_len != bit_length){
                printf("\teni contradicts xml\n");
                return -1;
            }
            rxPDOOffsets[count] += pdoEntry.bit_len;
            k++;
        }
        pdoCfg = task->get_slot_tx_config(slave, 0);
        for(ecat::pdo_entry_info const& pdoEntry : pdoCfg->entries){
            unsigned short index = 0;
            unsigned char subindex = 0, bit_length = 0;
            std::tie(index, subindex, bit_length) = pdoEntries[k];
            if(pdoEntry.entry_idx.idx != index || pdoEntry.entry_idx.sub_idx != subindex || pdoEntry.bit_len != bit_length){
                printf("\teni contradicts xml\n");
                return -1;
            }
            txPDOOffsets[count] += pdoEntry.bit_len;
            k++;
        }
        rxPDOOffsets[count] /= 8;
        txPDOOffsets[count] /= 8;
        unsigned int u32 = 0;
        unsigned short u16 = 0;
        unsigned char u8 = domainDivisions[0] * period / 1000000L;
        while(true){
            try{
                master->sdo_download(slave, {0x60c2, 0x01}, false, u8);
            }catch(std::exception const& e){
                sleep(1);
                continue;
            }
            break;
        }
        std::vector<std::string> timeoutEntry = configXML->entry(configXML->device("ECAT", type.c_str()), "Timeout");
        switch((unsigned char)strtoul(timeoutEntry[4].c_str(), nullptr, 10)){
        case 8:
            u8 = 100;
            while(true){
                try{
                    master->sdo_download(slave, {(unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16)}, false, u8);
                }catch(std::exception const& e){
                    sleep(1);
                    continue;
                }
                break;
            }
            break;
        case 16:
            u16 = 100;
            while(true){
                try{
                    master->sdo_download(slave, {(unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16)}, false, u16);
                }catch(std::exception const& e){
                    sleep(1);
                    continue;
                }
                break;
            }
            break;
        case 32:
            u32 = 100;
            while(true){
                try{
                    master->sdo_download(slave, {(unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16)}, false, u32);
                }catch(std::exception const& e){
                    sleep(1);
                    continue;
                }
                break;
            }
            break;
        }
        u16 = maxCurrent[alias - 1];
        while(true){
            try{
                master->sdo_download(slave, {0x6072, 0x00}, false, u16);
            }catch(std::exception const& e){
                sleep(1);
                continue;
            }
            break;
        }
        count++;
        itr++;
    }
    txPDOOffsets[0] += rxPDOOffsets[0];
    count = 1;
    while(count < alias2slave.size()){
        rxPDOOffsets[count] += txPDOOffsets[count - 1];
        txPDOOffsets[count] += rxPDOOffsets[count];
        count++;
    }
    count--;
    domainSizes[0] = txPDOOffsets[count];
    while(count > 0){
        txPDOOffsets[count] = rxPDOOffsets[count];
        rxPDOOffsets[count] = txPDOOffsets[count - 1];
        count--;
    }
    txPDOOffsets[0] = rxPDOOffsets[0];
    rxPDOOffsets[0] = 0;
    printf("master %d, domain 0, domainSize %d\n", order, domainSizes[0]);
    rxPDOSwaps[0] = new SwapList(domainSizes[0]);
    txPDOSwaps[0] = new SwapList(domainSizes[0]);
    itr = alias2slave.begin();
    while(itr != alias2slave.end()){
        int alias = itr->first, slave = itr->second;
        std::string type = alias2type.find(alias)->second;
        if(drivers[alias - 1].init("ECAT", 0, order, 0, slave, alias, type, rxPDOOffsets[count], txPDOOffsets[count], nullptr) != 0){
            printf("\tdrivers[%d] init failed\n", alias - 1);
            return -1;
        }
        switch(drivers[alias - 1].config("ECAT", order, 0, rxPDOSwaps[0], txPDOSwaps[0])){
        case 2:
            drivers[alias - 1].tx->StatusWord = 0xffff;
            break;
        case 1:
            break;
        case 0:
            break;
        case -1:
            printf("\tdrivers[%d] config failed\n", alias - 1);
            return -1;
            break;
        }
        count++;
        itr++;
    }
    task->set_config_callback([this](){
        int count = 0;
        auto itr = alias2slave.begin();
        while(itr != alias2slave.end()){
            int slave = itr->second;
            while(requestState(slave, "PREOP") < 0);
            ecat::pdo_cfg* pdoCfg = task->get_slot_rx_config(slave, 0);
            task->auto_register_pdo_entry(pdoCfg, slave, ecat::PdoTypeRx);
            pdoCfg = task->get_slot_tx_config(slave, 0);
            task->auto_register_pdo_entry(pdoCfg, slave, ecat::PdoTypeTx);
            tinyxml2::XMLElement* deviceXML = configXML->device("ECAT", alias2type.find(itr->first)->second.c_str());
            std::vector<std::vector<std::string>> rxPDOs = configXML->pdos(deviceXML, "RxPDOs"), txPDOs = configXML->pdos(deviceXML, "TxPDOs");
            int i = 0;
            while(i < rxPDOs.size()){
                int j = 1;
                while(j < rxPDOs[i].size()){
                    std::vector<std::string> entry = configXML->entry(deviceXML, rxPDOs[i][j].c_str());
                    unsigned short index = (unsigned short)strtoul(entry[1].c_str(), nullptr, 16);
                    unsigned char subindex = (unsigned char)strtoul(entry[2].c_str(), nullptr, 16);
                    if(      entry[0] == "TargetPosition"){
                        targetPosition[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] == "TargetVelocity"){
                        targetVelocity[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==   "TargetTorque"){
                          targetTorque[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==    "ControlWord"){
                           controlWord[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==           "Mode"){
                                  mode[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==   "TorqueOffset"){
                          torqueOffset[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] == "VelocityOffset"){
                        velocityOffset[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }
                    j++;
                }
                i++;
            }
            i = 0;
            while(i < txPDOs.size()){
                int j = 1;
                while(j < txPDOs[i].size()){
                    std::vector<std::string> entry = configXML->entry(deviceXML, txPDOs[i][j].c_str());
                    unsigned short index = (unsigned short)strtoul(entry[1].c_str(), nullptr, 16);
                    unsigned char subindex = (unsigned char)strtoul(entry[2].c_str(), nullptr, 16);
                    if(      entry[0] == "ActualPosition"){
                        actualPosition[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] == "ActualVelocity"){
                        actualVelocity[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==   "ActualTorque"){
                          actualTorque[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==     "StatusWord"){
                            statusWord[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==    "ModeDisplay"){
                           modeDisplay[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }else if(entry[0] ==      "ErrorCode"){
                             errorCode[count] = task->get_pdo_by_entry(slave, {index, subindex});
                    }
                    j++;
                }
                i++;
            }
            count++;
            itr++;
        }
    });
    task->set_activation_callback([this](){
        tryCount = 0;
        auto itr = alias2slave.begin();
        while(itr != alias2slave.end()){
            int alias = itr->first, slave = itr->second;
            while(requestState(slave, "OP") < 0);
            ecat::sdo_request* sdoHandler = nullptr;
            try{
                sdoHandler = task->create_sdo(slave, {0x0000, 0x00}, 4, false);
            }catch(std::exception const& e){
                printf("creating sdo request failed\n");
                exit(-1);
            }
            sdoHandler->timeout(500);
            switch(drivers[alias - 1].config("ECAT", order, 0, sdoHandler)){
            case 2:
                drivers[alias - 1].tx->StatusWord = 0xffff;
                break;
            case 1:
                break;
            case 0:
                break;
            case -1:
                printf("drivers[%d] config failed\n", alias - 1);
                exit(-1);
                break;
            }
            itr++;
        }
    });
    task->set_cycle_callback([this](){
        if(sdoMsg == nullptr){
            sdoMsg = sdoRequestQueue.get_nonblocking();
        }else{
            if(tryCount > 500){
                sdoMsg->state = -1;
            }
            if(sdoMsg->state == 0){
                sdoMsg->sdoHandler->index({sdoMsg->index, sdoMsg->subindex}, false);
                sdoMsg->state = 2;
            }else if(sdoMsg->state == 2){
                switch(sdoMsg->sdoHandler->state()){
                case ecat::request_state::usused:
                    if(sdoMsg->operation == 0){
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->bitLength == 8){
                                sdoMsg->sdoHandler->data<unsigned char>(sdoMsg->value);
                            }else if(sdoMsg->bitLength == 16){
                                sdoMsg->sdoHandler->data<unsigned short>(sdoMsg->value);
                            }else if(sdoMsg->bitLength == 32){
                                sdoMsg->sdoHandler->data<unsigned int>(sdoMsg->value);
                            }
                        }else{
                            if(sdoMsg->bitLength == 8){
                                sdoMsg->sdoHandler->data<char>(sdoMsg->value);
                            }else if(sdoMsg->bitLength == 16){
                                sdoMsg->sdoHandler->data<short>(sdoMsg->value);
                            }else if(sdoMsg->bitLength == 32){
                                sdoMsg->sdoHandler->data<int>(sdoMsg->value);
                            }
                        }
                        sdoMsg->sdoHandler->write();
                    }else{
                        sdoMsg->sdoHandler->read();
                    }
                    break;
                case ecat::request_state::success:
                    if(sdoMsg->operation == 0){
                        ;
                    }else{
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->bitLength == 8){
                                sdoMsg->value = sdoMsg->sdoHandler->data<unsigned char>();
                            }else if(sdoMsg->bitLength == 16){
                                sdoMsg->value = sdoMsg->sdoHandler->data<unsigned short>();
                            }else if(sdoMsg->bitLength == 32){
                                sdoMsg->value = sdoMsg->sdoHandler->data<unsigned int>();
                            }
                        }else{
                            if(sdoMsg->bitLength == 8){
                                sdoMsg->value = sdoMsg->sdoHandler->data<char>();
                            }else if(sdoMsg->bitLength == 16){
                                sdoMsg->value = sdoMsg->sdoHandler->data<short>();
                            }else if(sdoMsg->bitLength == 32){
                                sdoMsg->value = sdoMsg->sdoHandler->data<int>();
                            }
                        }
                    }
                    sdoMsg->state = 3;
                    break;
                case ecat::request_state::error:
                    if(sdoMsg->operation == 0){
                        sdoMsg->sdoHandler->write();
                    }else{
                        sdoMsg->sdoHandler->read();
                    }
                case ecat::request_state::busy:
                    tryCount++;
                    break;
                }
            }else if(sdoMsg->state == 3 || sdoMsg->state == -1){
                sdoResponseQueue.put(sdoMsg);
                sdoMsg = nullptr;
                tryCount = 0;
            }
        }
    });
    task->set_receive_callback([this](){
        int count = 0;
        auto itr = alias2slave.begin();
        while(itr != alias2slave.end()){
            int alias = itr->first;
            *(           int*)targetPosition[count]->dataPtr = drivers[alias - 1].rx.previous()->TargetPosition;
            *(           int*)targetVelocity[count]->dataPtr = drivers[alias - 1].rx.previous()->TargetVelocity;
            *(         short*)  targetTorque[count]->dataPtr = drivers[alias - 1].rx.previous()->TargetTorque;
            *(unsigned short*)   controlWord[count]->dataPtr = drivers[alias - 1].rx.previous()->ControlWord;
            *(          char*)          mode[count]->dataPtr = drivers[alias - 1].rx.previous()->Mode;
            *(         short*)  torqueOffset[count]->dataPtr = drivers[alias - 1].rx.previous()->TorqueOffset;
            *(           int*)velocityOffset[count]->dataPtr = drivers[alias - 1].rx.previous()->VelocityOffset;
            drivers[alias - 1].tx.next()->ActualPosition = *(           int*)actualPosition[count]->dataPtr;
            drivers[alias - 1].tx.next()->ActualVelocity = *(           int*)actualVelocity[count]->dataPtr;
            drivers[alias - 1].tx.next()->ActualTorque   = *(         short*)  actualTorque[count]->dataPtr;
            drivers[alias - 1].tx.next()->StatusWord     = *(unsigned short*)    statusWord[count]->dataPtr;
            drivers[alias - 1].tx.next()->ModeDisplay    = *(          char*)   modeDisplay[count]->dataPtr;
            drivers[alias - 1].tx.next()->ErrorCode      = *(unsigned short*)     errorCode[count]->dataPtr;
            count++;
            itr++;
        }
        txPDOSwaps[0]->advanceNodePtr();
    });
    return 0;
}

int ECAT::run(){
    if(alias2type.size() == 0){
        return 0;
    }
    printf("ecats[%d] rxtx on cpu %d\n", order, cpu);
    task->start();
    return 0;
}

void ECAT::clean(){
    if(task != nullptr){
        task->break_();
        sleep(1);
        task->release();
        master = nullptr;
        task = nullptr;
    }
    if(targetPosition != nullptr){
        delete[] targetPosition;
    }
    if(targetVelocity != nullptr){
        delete[] targetVelocity;
    }
    if(targetTorque != nullptr){
        delete[] targetTorque;
    }
    if(controlWord != nullptr){
        delete[] controlWord;
    }
    if(mode != nullptr){
        delete[] mode;
    }
    if(torqueOffset != nullptr){
        delete[] torqueOffset;
    }
    if(velocityOffset != nullptr){
        delete[] velocityOffset;
    }
    if(actualPosition != nullptr){
        delete[] actualPosition;
    }
    if(actualVelocity != nullptr){
        delete[] actualVelocity;
    }
    if(actualTorque != nullptr){
        delete[] actualTorque;
    }
    if(statusWord != nullptr){
        delete[] statusWord;
    }
    if(modeDisplay != nullptr){
        delete[] modeDisplay;
    }
    if(errorCode != nullptr){
        delete[] errorCode;
    }
    if(sdoMsg != nullptr){
        delete sdoMsg;
        sdoMsg = nullptr;
    }
    if(regMsg != nullptr){
        delete regMsg;
        regMsg = nullptr;
    }
    int i = 0;
    while(i < domainDivisions.size()){
        if(rxPDOSwaps[i] != nullptr){
            delete rxPDOSwaps[i];
            rxPDOSwaps[i] = nullptr;
        }
        if(txPDOSwaps[i] != nullptr){
            delete txPDOSwaps[i];
            txPDOSwaps[i] = nullptr;
        }
        i++;
    }
    if(rxPDOSwaps != nullptr){
        delete[] rxPDOSwaps;
        rxPDOSwaps = nullptr;
    }
    if(txPDOSwaps != nullptr){
        delete[] txPDOSwaps;
        txPDOSwaps = nullptr;
    }
    if(domainSizes != nullptr){
        delete[] domainSizes;
        domainSizes = nullptr;
    }
    sleep(1);
}

ECAT::~ECAT(){
    clean();
}
}