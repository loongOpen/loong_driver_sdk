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

#include "config_xml.h"
#include "rs485.h"
#include "ecat.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <atomic>
#include <sstream>
#include <limits>

namespace DriverSDK{
struct ec_ioctl_slave_state_t{
    unsigned short slave_position;
    unsigned char al_state;
};

#define EC_IOCTL_TYPE 0xa4
#define EC_IOW(nr, type) _IOW(EC_IOCTL_TYPE, nr, type)
#define EC_IOCTL_SLAVE_STATE EC_IOW(0x0b, ec_ioctl_slave_state_t)

extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> ecatAlias2type;
extern std::vector<std::map<int, int>> ecatAlias2domain;
extern std::vector<std::vector<int>> ecatDomainDivision;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
extern WrapperPair<ConverterRxData, ConverterTxData, EffectorParameters> converters[2];
extern WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
extern unsigned short processor;
extern std::vector<unsigned short> maxCurrent;
extern std::atomic<bool> ecatStalled;

extern std::vector<RS485>* rs485sPtr;

WrapperPair<HandRxData, HandTxData, EffectorParameters> hands[2];

ECAT::ECAT(int const order){
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
    period = configXML->period("ECAT", order);
    dc = configXML->dc("ECAT", order);
    alias2domain = ecatAlias2domain[order];
    domainDivision = ecatDomainDivision[order];
    domains = nullptr;
    domainPtrs = nullptr;
    domainSizes = nullptr;
    rxPDOSwaps = nullptr;
    txPDOSwaps = nullptr;
    sdoRequestable = false;
    master = nullptr;
    fd = -1;
    pth = 0;
    while(init() < 0){
        clean();
    }
}

int ECAT::init(){
    master = ecrt_request_master(order);
    if(master == nullptr){
        printf("requesting master %d failed\n", order);
        return -1;
    }
    std::stringstream deviceName;
    deviceName << "/dev/EtherCAT" << order;
    if((fd = open(deviceName.str().c_str(), O_RDWR)) < 0){
        printf("opening master device %s failed\n", deviceName.str().c_str());
        return -1;
    }
    domains = new ec_domain_t*[domainDivision.size()];
    domainPtrs = new unsigned char*[domainDivision.size()];
    domainSizes = new int[domainDivision.size()];
    rxPDOSwaps = new SwapList*[domainDivision.size()];
    txPDOSwaps = new SwapList*[domainDivision.size()];
    int i = 0;
    while(i < domainDivision.size()){
        domains[i] = nullptr;
        domainPtrs[i] = nullptr;
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
    unsigned char u8 = 0;
    unsigned short u16 = 0;
    unsigned int u32 = 0, abortCode = 0;
    unsigned long resultSize = 0;
    if(bitLength == 8){
        while(ecrt_master_sdo_upload(master, slave, index, subindex, &u8, sizeof(u8), &resultSize, &abortCode) < 0){
            sleep(1);
        }
        return u8;
    }else if(bitLength == 16){
        while(ecrt_master_sdo_upload(master, slave, index, subindex, (unsigned char*)&u16, sizeof(u16), &resultSize, &abortCode) < 0){
            sleep(1);
        }
        return u16;
    }else if(bitLength == 32){
        while(ecrt_master_sdo_upload(master, slave, index, subindex, (unsigned char*)&u32, sizeof(u32), &resultSize, &abortCode) < 0){
            sleep(1);
        }
        return u32;
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
    unsigned char state = 0x00;
    if(strcmp(stateString, "INIT") == 0) {
        state = 0x01;
    }else if(strcmp(stateString, "PREOP") == 0){
        state = 0x02;
    }else if(strcmp(stateString, "BOOT") == 0){
        state = 0x03;
    }else if(strcmp(stateString, "SAFEOP") == 0){
        state = 0x04;
    }else if(strcmp(stateString, "OP") == 0){
        state = 0x08;
    }else{
        printf("requesting invalid state %s\n", stateString);
        exit(-1);
    }
    ec_ioctl_slave_state_t data;
    data.slave_position = slave;
    data.al_state = state;
    if(ioctl(fd, EC_IOCTL_SLAVE_STATE, &data) < 0){
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
        if(itr->second >= domainDivision.size()){
            printf("master %d device with alias %d is assigned to an unspecified domain\n", order, itr->first);
            return -1;
        }
        itr++;
    }
    ec_master_info_t masterInfo;
    if(ecrt_master(master, &masterInfo) < 0){
        printf("obtaining master %d's info failed\n", order);
        return -1;
    }
    if(masterInfo.link_up != 1){
        printf("master %d's link is down\n", order);
        return 1;
    }
    if(masterInfo.scan_busy == 1){
        printf("master %d is scanning\n", order);
        return 1;
    }
    printf("master %d, %ld device(s) in xml, %d slave(s) on bus\n", order, alias2type.size(), masterInfo.slave_count);
    alias2slave.clear();
    int i = 0;
    while(i < masterInfo.slave_count){
        printf("slave %d:%d", order, i);
        ec_slave_info_t slaveInfo;
        if(ecrt_master_get_slave(master, i, &slaveInfo) < 0){
            printf("\n\tobtaining slave info faild\n");
            return -1;
        }
        char buffVID[16], buffPC[16];
        sprintf(buffVID, "0x%08x", slaveInfo.vendor_id);
        sprintf(buffPC, "0x%08x", slaveInfo.product_code);
        printf(", vendor_id %s, product_code %s", buffVID, buffPC);
        tinyxml2::XMLElement* deviceXML = configXML->busDevice("ECAT", buffVID, buffPC);
        if(deviceXML == nullptr){
            printf("\n\tdevice not registered in xml\n");
            i++;
            continue;
        }
        std::string type = configXML->type(deviceXML);
        printf(", type %s", type.c_str());
        std::string category = configXML->category("ECAT", type.c_str());
        if(category == ""){
            printf("\n\tinvalid device category\n");
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
    int i = 0;
    while(i < domainDivision.size()){
        domains[i] = ecrt_master_create_domain(master);
        if(domains[i] == nullptr){
            printf("creating master %d domain[%d] failed\n", order, i);
            return -1;
        }
        i++;
    }
    auto itr = alias2slave.begin();
    while(itr != alias2slave.end()){
        int alias = itr->first, slave = itr->second, domain = alias2domain.find(alias)->second;
        std::string type = alias2type.find(alias)->second, category = configXML->category("ECAT", type.c_str());
        printf("master %d, domain %d, slave %d, alias %d, category %s, type %s\n", order, domain, slave, alias, category.c_str(), type.c_str());
        while(requestState(slave, "PREOP") < 0);
        tinyxml2::XMLElement* deviceXML = configXML->busDevice("ECAT", type.c_str());
        std::vector<std::vector<std::string>> rxPDOs = configXML->pdos(deviceXML, "RxPDOs");
        std::vector<std::vector<std::string>> txPDOs = configXML->pdos(deviceXML, "TxPDOs");
        int rxPDOCount = 0, txPDOCount = 0;
        i = 0;
        while(i < rxPDOs.size()){
            rxPDOCount += rxPDOs[i].size() - 1;
            i++;
        }
        i = 0;
        while(i < txPDOs.size()){
            txPDOCount += txPDOs[i].size() - 1;
            i++;
        }
        ec_pdo_entry_info_t pdoEntries[rxPDOCount + txPDOCount];
        int j = 0, k = 0;
        i = 0;
        while(i < rxPDOs.size()){
            j = 1;
            while(j < rxPDOs[i].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, rxPDOs[i][j].c_str());
                pdoEntries[k] = ec_pdo_entry_info_t{
                    (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                    (unsigned char) strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char) strtoul(entry[4].c_str(), nullptr, 10)
                };
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
                pdoEntries[k] = ec_pdo_entry_info_t{
                    (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                    (unsigned char) strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char) strtoul(entry[4].c_str(), nullptr, 10)
                };
                j++;
                k++;
            }
            i++;
        }
        ec_pdo_info_t pdoInfos[rxPDOs.size() + txPDOs.size()];
        k = 0;
        ec_pdo_entry_info_t* pdoEntry = pdoEntries;
        i = 0;
        while(i < rxPDOs.size()){
            if(i > 0){
                pdoEntry += rxPDOs[i - 1].size() - 1;
            }
            pdoInfos[k] = ec_pdo_info_t{
                (unsigned short)strtoul(rxPDOs[i][0].c_str(), nullptr, 16),
                (unsigned int)rxPDOs[i].size() - 1,
                pdoEntry
            };
            i++;
            k++;
        }
        i = 0;
        while(i < txPDOs.size()){
            if(i > 0){
                pdoEntry += txPDOs[i - 1].size() - 1;
            }else{
                pdoEntry += rxPDOs[rxPDOs.size() - 1].size() - 1;
            }
            pdoInfos[k] = ec_pdo_info_t{
                (unsigned short)strtoul(txPDOs[i][0].c_str(), nullptr, 16),
                (unsigned int)txPDOs[i].size() - 1,
                pdoEntry
            };
            i++;
            k++;
        }
        ec_sync_info_t syncInfos[5];
        syncInfos[0] = ec_sync_info_t{0, EC_DIR_OUTPUT,                           0,                  nullptr, EC_WD_DISABLE};
        syncInfos[1] = ec_sync_info_t{1,  EC_DIR_INPUT,                           0,                  nullptr, EC_WD_DISABLE};
        syncInfos[2] = ec_sync_info_t{2, EC_DIR_OUTPUT, (unsigned int)rxPDOs.size(),                 pdoInfos,  EC_WD_ENABLE};
        syncInfos[3] = ec_sync_info_t{3,  EC_DIR_INPUT, (unsigned int)txPDOs.size(), pdoInfos + rxPDOs.size(), EC_WD_DISABLE};
        syncInfos[4] = ec_sync_info_t{0xff};
        if(category == "driver"){
            unsigned short index[2 + rxPDOs.size() + txPDOs.size()];
            index[0] = 0x1c12;
            index[1] = 0x1c13;
            k = 0;
            while(k < rxPDOs.size() + txPDOs.size()){
                printf("\t%x, %u, %lu\n", pdoInfos[k].index, pdoInfos[k].n_entries, (unsigned long)pdoInfos[k].entries);
                index[2 + k] = pdoInfos[k].index;
                k++;
            }
            unsigned char u8 = 0;
            unsigned short u16 = 0;
            unsigned int u32 = 0, abortCode = 0;
            i = 0;
            while(i < 2 + rxPDOs.size() + txPDOs.size()){
                if(ecrt_master_sdo_download(master, slave, index[i], 0x00, &u8, sizeof(u8), &abortCode) < 0){
                    sleep(1);
                }
                i++;
            }
            u8 = domainDivision[domain] * period / 1000000L;
            while(ecrt_master_sdo_download(master, slave, 0x60c2, 0x01, &u8, sizeof(u8), &abortCode) < 0){
                sleep(1);
            }
            std::vector<std::string> timeoutEntry = configXML->entry(configXML->busDevice("ECAT", type.c_str()), "Timeout");
            switch((unsigned char)strtoul(timeoutEntry[4].c_str(), nullptr, 10)){
            case 8:
                u8 = 100;
                while(ecrt_master_sdo_download(master, slave, (unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16), &u8, sizeof(u8), &abortCode) < 0){
                    sleep(1);
                }
                break;
            case 16:
                u16 = 100;
                while(ecrt_master_sdo_download(master, slave, (unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16), (unsigned char*)&u16, sizeof(u16), &abortCode) < 0){
                    sleep(1);
                }
                break;
            case 32:
                u32 = 100;
                while(ecrt_master_sdo_download(master, slave, (unsigned short)strtoul(timeoutEntry[1].c_str(), nullptr, 16), (unsigned char)strtoul(timeoutEntry[2].c_str(), nullptr, 16), (unsigned char*)&u32, sizeof(u32), &abortCode) < 0){
                    sleep(1);
                }
                break;
            }
            u16 = maxCurrent[alias - 1];
            while(ecrt_master_sdo_download(master, slave, 0x6072, 0x00, (unsigned char*)&u16, sizeof(u16), &abortCode) < 0){
                sleep(1);
            }
        }
        ec_slave_config_t* slaveConfig = ecrt_master_slave_config(master, 0, slave, configXML->vendorID(deviceXML), configXML->productCode(deviceXML));
        if(slaveConfig == nullptr){
            printf("\tobtaining slave config failed\n");
            return -1;
        }
        k = 0;
        while(k < 5){
            printf("\t%x, %d, %u, %lu, %d\n", syncInfos[k].index, syncInfos[k].dir, syncInfos[k].n_pdos, (unsigned long)syncInfos[k].pdos, syncInfos[k].watchdog_mode);
            k++;
        }
        if(ecrt_slave_config_pdos(slaveConfig, EC_END, syncInfos) < 0){
            printf("\tconfiguring PDOs failed\n");
            return -1;
        }
        unsigned int bitPosition = 0;
        k = 0;
        printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
        int rxPDOOffset = ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domains[domain], &bitPosition);
        if(rxPDOOffset < 0){
            printf("\tregistering RxPDO entry failed\n");
            return -1;
        }
        k++;
        while(k < rxPDOCount){
            printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
            if(ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domains[domain], &bitPosition) < 0){
                printf("\tregistering RxPDO entry failed\n");
                return -1;
            }
            k++;
        }
        printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
        int txPDOOffset = ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domains[domain], &bitPosition);
        if(txPDOOffset < 0){
            printf("\tregistering TxPDO entry failed\n");
            return -1;
        }
        k++;
        while(k < rxPDOCount + txPDOCount){
            printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
            if(ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domains[domain], &bitPosition) < 0){
                printf("\tregistering TxPDO entry failed\n");
                return -1;
            }
            k++;
        }
        printf("\trxPDOOffset %d, txPDOOffset %d\n", rxPDOOffset, txPDOOffset);
        ec_sdo_request_t* sdoHandler = ecrt_slave_config_create_sdo_request(slaveConfig, 0x0000, 0x00, 4);
        if(sdoHandler == nullptr){
            printf("\tcreating SDO request failed\n");
            return -1;
        }
        ecrt_sdo_request_timeout(sdoHandler, 500);
        if(category == "driver"){
            if(drivers[alias - 1].init("ECAT", order, domain, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                printf("\tdrivers[%d] init failed\n", alias - 1);
                return -1;
            }
        }else if(category == "effector"){
            if(alias == 200){
                i = 0;
                j = dofLeftEffector;
            }else if(alias == 201){
                i = dofLeftEffector;
                j = dofEffector;
            }
            k = 0;
            if(type == "Tsinghua"){
                while(i < j){
                    if(digits[i].init("ECAT", order, domain, slave, alias, type,
                        rxPDOOffset +     7 * sizeof(unsigned short) + k * sizeof(DigitRxData),
                        txPDOOffset + 5 * 4 * sizeof(unsigned short) + k * sizeof(DigitTxData), sdoHandler) != 0){
                        printf("\tdigits[%d] init failed\n", i);
                        return -1;
                    }
                    i++;
                    k++;
                }
                if(hands[alias - 200].init("ECAT", order, domain, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                    printf("\thands[%d] init failed\n", alias - 200);
                    return -1;
                }
            }else if(type == "Ruiyan_1dof" || type == "Ruiyan_6dof"){
                if(converters[alias - 200].init("ECAT", order, domain, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                    printf("\tconverters[%d] init failed\n", alias - 200);
                    return -1;
                }
            }else{
                ;
            }
        }else if(category == "sensor"){
            if(type == "LinkTouch"){
                if(sensors[alias - 220].init("ECAT", order, domain, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                    printf("\tsensors[%d] init failed\n", alias - 220);
                    return -1;
                }
            }else{
                ;
            }
        }
        if(dc){
            ecrt_slave_config_dc(slaveConfig, 0x0300, domainDivision[domain] * period, domainDivision[domain] * period / 2, 0, 0);
        }
        itr++;
    }
    if(ecrt_master_activate(master) < 0){
        printf("activating master %d failed\n", order);
        return -1;
    }
    i = 0;
    while(i < domainDivision.size()){
        domainPtrs[i] = ecrt_domain_data(domains[i]);
        if(domainPtrs[i] == nullptr){
            printf("obtaining master %d domains[%d] data failed\n", order, i);
            return -1;
        }
        domainSizes[i] = ecrt_domain_size(domains[i]);
        if(domainSizes[i] < 0){
            printf("obtaining master %d domains[%d] size failed\n", order, i);
            return -1;
        }
        printf("master %d, domain %d, domainPtr %ld, domainSize %d\n", order, i, (unsigned long)domainPtrs[i], domainSizes[i]);
        if(domainSizes[i] == 0){
            i++;
            continue;
        }
        rxPDOSwaps[i] = new SwapList(domainSizes[i]);
        txPDOSwaps[i] = new SwapList(domainSizes[i]);
        int j = 0;
        while(j < dofAll){
            switch(drivers[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i])){
            case 2:
                drivers[j].tx->StatusWord = 0xffff;
                break;
            case 1:
                break;
            case 0:
                break;
            case -1:
                printf("drivers[%d] config failed\n", j);
                return -1;
                break;
            }
            j++;
        }
        j = 0;
        while(j < dofEffector){
            if(digits[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i]) == -1){
                printf("digits[%d] config failed\n", j);
                return -1;
            }
            j++;
        }
        j = 0;
        while(j < 2){
            switch(hands[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i])){
            case 2:
                break;
            case 1:
                break;
            case 0:
                hands[j].rx->CurrentLimitThumb      = 1000;
                hands[j].rx->CurrentLimitThumbBend  = 1000;
                hands[j].rx->CurrentLimitForefinger = 1000;
                hands[j].rx->CurrentLimitMiddle     = 1000;
                hands[j].rx->CurrentLimitRing       = 1000;
                hands[j].rx->CurrentLimitLittle     = 1000;
                hands[j].rx->TargetSpeedThumb       = 100;
                hands[j].rx->TargetSpeedThumbBend   = 100;
                hands[j].rx->TargetSpeedForefinger  = 100;
                hands[j].rx->TargetSpeedMiddle      = 100;
                hands[j].rx->TargetSpeedRing        = 100;
                hands[j].rx->TargetSpeedLittle      = 100;
                rxPDOSwaps[i]->advanceNodePtr();
                hands[j].rx->CurrentLimitThumb      = 1000;
                hands[j].rx->CurrentLimitThumbBend  = 1000;
                hands[j].rx->CurrentLimitForefinger = 1000;
                hands[j].rx->CurrentLimitMiddle     = 1000;
                hands[j].rx->CurrentLimitRing       = 1000;
                hands[j].rx->CurrentLimitLittle     = 1000;
                hands[j].rx->TargetSpeedThumb       = 100;
                hands[j].rx->TargetSpeedThumbBend   = 100;
                hands[j].rx->TargetSpeedForefinger  = 100;
                hands[j].rx->TargetSpeedMiddle      = 100;
                hands[j].rx->TargetSpeedRing        = 100;
                hands[j].rx->TargetSpeedLittle      = 100;
                rxPDOSwaps[i]->advanceNodePtr();
                hands[j].rx->CurrentLimitThumb      = 1000;
                hands[j].rx->CurrentLimitThumbBend  = 1000;
                hands[j].rx->CurrentLimitForefinger = 1000;
                hands[j].rx->CurrentLimitMiddle     = 1000;
                hands[j].rx->CurrentLimitRing       = 1000;
                hands[j].rx->CurrentLimitLittle     = 1000;
                hands[j].rx->TargetSpeedThumb       = 100;
                hands[j].rx->TargetSpeedThumbBend   = 100;
                hands[j].rx->TargetSpeedForefinger  = 100;
                hands[j].rx->TargetSpeedMiddle      = 100;
                hands[j].rx->TargetSpeedRing        = 100;
                hands[j].rx->TargetSpeedLittle      = 100;
                rxPDOSwaps[i]->advanceNodePtr();
                break;
            case -1:
                printf("hands[%d] config failed\n", j);
                return -1;
                break;
            }
            j++;
        }
        j = 0;
        while(j < 2){
            switch(converters[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i])){
            case 2:
                break;
            case 1:
                break;
            case 0:
                break;
            case -1:
                printf("converters[%d] config failed\n", j);
                return -1;
                break;
            }
            j++;
        }
        j = 0;
        while(j < 2){
            switch(sensors[j].config("ECAT", order, i, rxPDOSwaps[i], txPDOSwaps[i])){
            case 2:
                sensors[j].tx->StatusCode = 0xffff;
                break;
            case 1:
                break;
            case 0:
                break;
            case -1:
                printf("sensors[%d] config failed\n", j);
                return -1;
                break;
            }
            j++;
        }
        i++;
    }
    return 0;
}

void* ECAT::rxtx(void* arg){
    ECAT* ecat = (ECAT*)arg;
    int domainCount = ecat->domainDivision.size(), tryCount = 0, slavesResponding = 0, alStates = 0, workingCounters[domainCount] = {0}, wcStates[domainCount] = {0};
    printf("ecats[%d], period %ld, dc %d, domainCount %d, domainDivisions: ", ecat->order, ecat->period, ecat->dc, domainCount);
    int i = 0;
    while(i < domainCount){
        printf("%d ", ecat->domainDivision[i]);
        i++;
    }
    printf("\n");
    unsigned int count = 0xffffffff;
    SDOMsg* sdoMsg = nullptr;
    ec_master_state_t masterState;
    ec_domain_state_t domainStates[domainCount];
    struct timespec currentTime, wakeupTime, step{0, 6 * ecat->period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        if(sdoMsg == nullptr){
            sdoMsg = ecat->sdoRequestQueue.get_nonblocking();
        }else{
            if(tryCount > 500){
                sdoMsg->state = -1;
            }
            if(sdoMsg->state == 0){
                switch(ecrt_sdo_request_state(sdoMsg->sdoHandler)){
                case EC_REQUEST_UNUSED:
                case EC_REQUEST_SUCCESS:
                    ecrt_sdo_request_index(sdoMsg->sdoHandler, sdoMsg->index, sdoMsg->subindex);
                    sdoMsg->state = 1;
                    break;
                case EC_REQUEST_ERROR:
                    ecrt_sdo_request_index(sdoMsg->sdoHandler, sdoMsg->index, sdoMsg->subindex);
                case EC_REQUEST_BUSY:
                    tryCount++;
                    break;
                }
            }else if(sdoMsg->state == 1){
                switch(ecrt_sdo_request_state(sdoMsg->sdoHandler)){
                case EC_REQUEST_UNUSED:
                case EC_REQUEST_SUCCESS:
                    if(sdoMsg->operation == 0){
                        ecrt_sdo_request_write(sdoMsg->sdoHandler);
                    }
                    else if(sdoMsg->operation == 1){
                        ecrt_sdo_request_read(sdoMsg->sdoHandler);
                    }
                    sdoMsg->state = 2;
                    break;
                case EC_REQUEST_ERROR:
                    ecrt_sdo_request_index(sdoMsg->sdoHandler, sdoMsg->index, sdoMsg->subindex);
                case EC_REQUEST_BUSY:
                    tryCount++;
                    break;
                }
            }else if(sdoMsg->state == 2){
                switch(ecrt_sdo_request_state(sdoMsg->sdoHandler)){
                case EC_REQUEST_UNUSED:
                case EC_REQUEST_SUCCESS:
                    if(sdoMsg->bitLength == 8){
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_U8(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_U8(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else if(sdoMsg->signed_ == 1){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S8(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_S8(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }
                    }else if(sdoMsg->bitLength == 16){
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_U16(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_U16(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else if(sdoMsg->signed_ == 1){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S16(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_S16(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }
                    }else if(sdoMsg->bitLength == 32){
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_U32(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_U32(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else if(sdoMsg->signed_ == 1){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S32(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }
                            else if(sdoMsg->operation == 1){
                                sdoMsg->value = EC_READ_S32(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }
                    }
                    sdoMsg->state = 3;
                    break;
                case EC_REQUEST_ERROR:
                    if(sdoMsg->operation == 0){
                        ecrt_sdo_request_write(sdoMsg->sdoHandler);
                    }
                    else if(sdoMsg->operation == 1){
                        ecrt_sdo_request_read(sdoMsg->sdoHandler);
                    }
                case EC_REQUEST_BUSY:
                    tryCount++;
                    break;
                }
            }else if(sdoMsg->state == 3 || sdoMsg->state == -1){
                ecat->sdoResponseQueue.put(sdoMsg);
                sdoMsg = nullptr;
                tryCount = 0;
            }
        }
        if(ecat->dc){
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            ecrt_master_sync_reference_clock_to(ecat->master, TIMESPEC2NS(currentTime));
            ecrt_master_sync_slave_clocks(ecat->master);
        }
        count++;
        i = 0;
        while(i < domainCount){
            if(ecat->rxPDOSwaps[i] != nullptr && count % ecat->domainDivision[i] == 0){
                ecat->rxPDOSwaps[i]->copyTo(ecat->domainPtrs[i], ecat->domainSizes[i]);
                ecrt_domain_queue(ecat->domains[i]);
            }
            i++;
        }
        ecrt_master_send(ecat->master);
        wakeupTime.tv_nsec += ecat->period;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            wakeupTime.tv_sec++;
        }
        bool sleep = true;
        do{
            if(sleep){
                nanosleep(&step, nullptr);
            }
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            if(sleep && (TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime) < 12 * ecat->period / 100)){
                sleep = false;
            }
        }while(TIMESPEC2NS(currentTime) < TIMESPEC2NS(wakeupTime));
        if(ecat->dc){
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            ecrt_master_application_time(ecat->master, TIMESPEC2NS(currentTime));
        }
        ecrt_master_receive(ecat->master);
        ecrt_master_state(ecat->master, &masterState);
        if(masterState.slaves_responding != slavesResponding){
            slavesResponding = masterState.slaves_responding;
            printf("master %d slaves_responding changed to %d\n", ecat->order, slavesResponding);
        }
        if(masterState.al_states != alStates){
            alStates = masterState.al_states;
            printf("master %d al_states changed to 0x%02x\n", ecat->order, alStates);
        }
        i = 0;
        while(i < domainCount){
            if(ecat->txPDOSwaps[i] == nullptr || count % ecat->domainDivision[i] != 0){
                i++;
                continue;
            }
            ecrt_domain_process(ecat->domains[i]);
            ecrt_domain_state(ecat->domains[i], &domainStates[i]);
            if(domainStates[i].working_counter != workingCounters[i]){
                if(ecat->order == 0 && i == 0 && domainStates[i].working_counter < workingCounters[i]){
                    ecatStalled.store(true);
                }
                workingCounters[i] = domainStates[i].working_counter;
                printf("master %d domain %d working_counter changed to %d\n", ecat->order, i, workingCounters[i]);
            }
            if(domainStates[i].wc_state != wcStates[i]){
                if(ecat->order == 0 && i == 0 && domainStates[i].wc_state == EC_WC_COMPLETE){
                    ecatStalled.store(false);
                }
                wcStates[i] = domainStates[i].wc_state;
                printf("master %d domain %d wc_state changed to %d\n", ecat->order, i, wcStates[i]);
            }
            if(domainStates[i].wc_state == EC_WC_COMPLETE){
                ecat->txPDOSwaps[i]->copyFrom(ecat->domainPtrs[i], ecat->domainSizes[i]);
                int j = 0;
                while(j < 2){
                    if(converters[j].order != ecat->order || converters[j].domain != i){
                        j++;
                        continue;
                    }
                    ConverterDatum const& channel = converters[j].tx->channels[0];
                    if(channel.Index == converters[j].enabled){
                        j++;
                        continue;
                    }
                    converters[j].enabled = channel.Index;
                    if(channel.Length < 1){
                        j++;
                        continue;
                    }
                    printf("Index: %8d ", channel.Index);
                    std::vector<RS485> const& rs485s = *rs485sPtr;
                    int k = 0;
                    while(k < rs485s.size()){
                        if(rs485s[k].fdR < 0){
                            k++;
                            continue;
                        }
                        if(rs485s[k].alias2type.begin()->first - 200 != j){
                            k++;
                            continue;
                        }
                        int l = 0;
                        while(l < channel.Length){
                            printf("%02x.", channel.Data[l]);
                            l++;
                        }
                        printf("\b\n");
                        write(rs485s[k].fdR, channel.Data, channel.Length);
                        k++;
                    }
                    j++;
                }
            }
            i++;
        }
    }
    return nullptr;
}

int ECAT::run(){
    if(alias2type.size() == 0){
        return 0;
    }
    int cpu = sysconf(_SC_NPROCESSORS_ONLN) - 1;
    if(cpu > processor){
        cpu = processor;
    }
    if(pthread_create(&pth, nullptr, &rxtx, this) != 0){
        printf("creating ecats[%d] rxtx thread failed\n", order);
        return -1;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if(pthread_setaffinity_np(pth, sizeof(cpu_set_t), &cpuset) != 0){
        printf("setting ecats[%d] rxtx thread cpu affinity failed\n", order);
        return -1;
    }
    printf("ecats[%d] rxtx on cpu %d\n", order, cpu);
    auto itr = alias2slave.begin();
    while(itr != alias2slave.end()){
        int slave = itr->second;
        while(requestState(slave, "OP") < 0);
        itr++;
    }
    return 0;
}

void ECAT::clean(){
    if(pth > 0){
        pthread_cancel(pth);
    }
    if(fd > -1){
        close(fd);
    }
    if(master != nullptr){
        ecrt_release_master(master);
    }
    int i = 0;
    while(i < domainDivision.size()){
        if(rxPDOSwaps[i] != nullptr){
            delete rxPDOSwaps[i];
        }
        if(txPDOSwaps[i] != nullptr){
            delete txPDOSwaps[i];
        }
        i++;
    }
    if(rxPDOSwaps != nullptr){
        delete[] rxPDOSwaps;
    }
    if(txPDOSwaps != nullptr){
        delete[] txPDOSwaps;
    }
    if(domainSizes != nullptr){
        delete[] domainSizes;
    }
    if(domainPtrs != nullptr){
        delete[] domainPtrs;
    }
    if(domains != nullptr){
        delete[] domains;
    }
    sleep(1);
}

ECAT::~ECAT(){
    clean();
}
}