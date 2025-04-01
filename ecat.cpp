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
#include "ecat.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
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
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
extern WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];
extern unsigned short processor;
extern std::vector<unsigned short> maxCurrent;

ECAT::ECAT(int const order){
    this->order = order;
    alias2type = ecatAlias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("master %d\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        printf("\talias %d, type %s\n", itr->first, itr->second.c_str());
        itr++;
    }
    period = configXML->period("ECAT", order);
    dc = configXML->dc("ECAT", order);
    master = nullptr;
    domain = nullptr;
    domainPtr = nullptr;
    domainSize = 0;
    fd = 0;
    rxPDOSwap = nullptr;
    txPDOSwap = nullptr;
    sdoRequestable = false;
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
    return 0;
}

int ECAT::readAlias(unsigned short const slave, unsigned short const index, unsigned char const subindex, unsigned char const bitLength){
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
        if(index == 0x0000){
            if(slave <= 8){
                return 200;
            }else{
                return 201;
            }
        }else if(index == 0x0001){
            if(slave <= 8){
                return 220;
            }else{
                return 221;
            }
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
    printf("master %d, %ld devices in xml, %d slaves on bus\n", order, alias2type.size(), masterInfo.slave_count);
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
        std::vector<std::string> aliasEntry = configXML->entry(deviceXML, "Alias");
        int alias = readAlias(i,
            strtoul(aliasEntry[1].c_str(), nullptr, 16),
            strtoul(aliasEntry[2].c_str(), nullptr, 16),
            strtoul(aliasEntry[4].c_str(), nullptr, 10));
        std::string type = configXML->type(deviceXML);
        printf(", alias %d, type %s\n", alias, type.c_str());
        auto itr = alias2type.find(alias);
        if(itr == alias2type.end()){
            printf("\tdevice alias not found/enabled in xml\n");
            i++;
            continue;
        }
        if(type != itr->second){
            printf("\tdevice type contradicts xml\n");
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
    domain = ecrt_master_create_domain(master);
    if(domain == nullptr){
        printf("master %d domain creation failed\n", order);
        return -1;
    }
    auto itr = alias2slave.begin();
    while(itr != alias2slave.end()){
        int alias = itr->first, slave = itr->second;
        std::string type = alias2type.find(alias)->second;
        std::string category = configXML->category("ECAT", type.c_str());
        printf("slave %d:%d, alias %d, type %s\n", order, slave, alias, type.c_str());
        if(category == ""){
            printf("\tinvalid device category\n");
            return -1;
        }
        while(requestState(slave, "PREOP") < 0);
        tinyxml2::XMLElement* deviceXML = configXML->busDevice("ECAT", type.c_str());
        std::vector<std::vector<std::string>> rxPDOs = configXML->pdos(deviceXML, "RxPDOs");
        std::vector<std::vector<std::string>> txPDOs = configXML->pdos(deviceXML, "TxPDOs");
        int i = 0, j = 0, rxPDOCount = 0, txPDOCount = 0;
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
        int k = 0;
        i = 0;
        while(i < rxPDOs.size()){
            j = 1;
            while(j < rxPDOs[i].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, rxPDOs[i][j].c_str());
                pdoEntries[k] = ec_pdo_entry_info_t{
                    (unsigned short)strtoul(entry[1].c_str(), nullptr, 16),
                    (unsigned char)strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char)strtoul(entry[4].c_str(), nullptr, 10)
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
                    (unsigned char)strtoul(entry[2].c_str(), nullptr, 16),
                    (unsigned char)strtoul(entry[4].c_str(), nullptr, 10)
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
        syncInfos[0] = ec_sync_info_t{0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE};
        syncInfos[1] = ec_sync_info_t{1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE};
        syncInfos[2] = ec_sync_info_t{2, EC_DIR_OUTPUT, (unsigned int)rxPDOs.size(), pdoInfos, EC_WD_ENABLE};
        syncInfos[3] = ec_sync_info_t{3, EC_DIR_INPUT, (unsigned int)txPDOs.size(), pdoInfos + rxPDOs.size(), EC_WD_DISABLE};
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
            u8 = period / 1000000L;
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
        int rxPDOOffset = ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domain, &bitPosition);
        if(rxPDOOffset < 0){
            printf("\tregistering RxPDO entry failed\n");
            return -1;
        }
        k++;
        while(k < rxPDOCount){
            printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
            if(ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domain, &bitPosition) < 0){
                printf("\tregistering RxPDO entry failed\n");
                return -1;
            }
            k++;
        }
        printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
        int txPDOOffset = ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domain, &bitPosition);
        if(txPDOOffset < 0){
            printf("\tregistering TxPDO entry failed\n");
            return -1;
        }
        k++;
        while(k < rxPDOCount + txPDOCount){
            printf("\t%x, %x, %u\n", pdoEntries[k].index, pdoEntries[k].subindex, pdoEntries[k].bit_length);
            if(ecrt_slave_config_reg_pdo_entry(slaveConfig, pdoEntries[k].index, pdoEntries[k].subindex, domain, &bitPosition) < 0){
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
            if(drivers[alias - 1].init("ECAT", order, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                printf("\tdrivers[%d] init failed\n", alias - 1);
                return -1;
            }
        }else if(category == "effector"){
            ;
        }else if(category == "sensor"){
            if(sensors[alias - 220].init("ECAT", order, slave, alias, type, rxPDOOffset, txPDOOffset, sdoHandler) != 0){
                printf("\tsensors[%d] init failed\n", alias - 1);
                return -1;
            }
        }
        if(dc == true){
            ecrt_slave_config_dc(slaveConfig, 0x0300, period, period / 2, 0, 0);
        }
        itr++;
    }
    if(ecrt_master_activate(master) < 0){
        printf("activating master %d failed\n", order);
        return -1;
    }
    domainPtr = ecrt_domain_data(domain);
    if(domainPtr == nullptr){
        printf("obtaining master %d domain data failed\n", order);
        return -1;
    }
    domainSize = ecrt_domain_size(domain);
    if(domainSize <= 0){
        printf("obtaining master %d domain size failed\n", order);
        return -1;
    }
    printf("master %d, domainPtr %ld, domainSize %d\n", order, (unsigned long)domainPtr, domainSize);
    rxPDOSwap = new SwapList(domainSize);
    txPDOSwap = new SwapList(domainSize);
    int i = 0;
    while(i < dofAll){
        switch(drivers[i].config("ECAT", order, rxPDOSwap, txPDOSwap)){
        case 2:
            drivers[i].tx->StatusWord = 0xffff;
            break;
        case 1:
            break;
        case 0:
            break;
        case -1:
            printf("drivers[%d] config failed\n", i);
            return -1;
            break;
        }
        i++;
    }
    i = 0;
    while(i < dofEffector){
        ;
        i++;
    }
    i = 0;
    while(i < 2){
        switch(sensors[i].config("ECAT", order, rxPDOSwap, txPDOSwap)){
        case 2:
            sensors[i].tx->StatusCode = 0xffff;
            break;
        case 1:
            break;
        case 0:
            break;
        case -1:
            printf("sensors[%d] config failed\n", i);
            return -1;
            break;
        }
        i++;
    }
    return 0;
}

void* ECAT::rxtx(void* arg){
    ECAT* ecat = (ECAT*)arg;
    printf("master %d period %ld dc %d\n", ecat->order, ecat->period, ecat->dc);
    SDOMsg* sdoMsg = nullptr;
    ec_master_info_t masterInfo;
    if(ecrt_master(ecat->master, &masterInfo) < 0){
        printf("obtaining master %d's info failed\n", ecat->order);
        exit(-1);
    }
    int slaveNr = masterInfo.slave_count, slavesResponding = 0, alStates = 0, workingCounter = 0, wcState = 0, tryCount = 0;
    ec_master_state_t masterState;
    ec_domain_state_t domainState;
    struct timespec currentTime, wakeupTime, step{0, 6 * ecat->period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    bool sleep = true;
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
                    else{
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
                            }else{
                                sdoMsg->value = EC_READ_U8(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else{
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S8(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else{
                                sdoMsg->value = EC_READ_S8(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }
                    }else if(sdoMsg->bitLength == 16){
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_U16(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else{
                                sdoMsg->value = EC_READ_U16(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else{
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S16(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else{
                                sdoMsg->value = EC_READ_S16(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }
                    }else{
                        if(sdoMsg->signed_ == 0){
                            if(sdoMsg->operation == 0){
                                EC_WRITE_U32(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }else{
                                sdoMsg->value = EC_READ_U32(ecrt_sdo_request_data(sdoMsg->sdoHandler));
                            }
                        }else{
                            if(sdoMsg->operation == 0){
                                EC_WRITE_S32(ecrt_sdo_request_data(sdoMsg->sdoHandler), sdoMsg->value);
                            }
                            else{
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
                    else{
                        ecrt_sdo_request_read(sdoMsg->sdoHandler);
                    }
                case EC_REQUEST_BUSY:
                    tryCount++;
                    break;
                }
            }else{
                ecat->sdoResponseQueue.put(sdoMsg);
                sdoMsg = nullptr;
                tryCount = 0;
            }
        }
        ecat->rxPDOSwap->copyTo(ecat->domainPtr, ecat->domainSize);
        if(ecat->dc == true){
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            ecrt_master_sync_reference_clock_to(ecat->master, TIMESPEC2NS(currentTime));
            ecrt_master_sync_slave_clocks(ecat->master);
        }
        ecrt_domain_queue(ecat->domain);
        ecrt_master_send(ecat->master);
        wakeupTime.tv_nsec += ecat->period;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            wakeupTime.tv_sec++;
        }
        sleep = true;
        do{
            if(sleep){
                nanosleep(&step, nullptr);
            }
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            if(sleep && (TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime) < 12 * ecat->period / 100)){
                sleep = false;
            }
        }while(TIMESPEC2NS(currentTime) < TIMESPEC2NS(wakeupTime));
        if(ecat->dc == true){
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            ecrt_master_application_time(ecat->master, TIMESPEC2NS(currentTime));
        }
        ecrt_master_receive(ecat->master);
        ecrt_domain_process(ecat->domain);
        ecrt_master_state(ecat->master, &masterState);
        ecrt_domain_state(ecat->domain, &domainState);
        if(masterState.slaves_responding == slaveNr && domainState.wc_state == EC_WC_COMPLETE){
            ecat->txPDOSwap->copyFrom(ecat->domainPtr, ecat->domainSize);
        }else{
            if(masterState.slaves_responding != slavesResponding){
                slavesResponding = masterState.slaves_responding;
                printf("master %d slaves_responding changed to %d\n", ecat->order, slavesResponding);
            }
            if(masterState.al_states != alStates){
                alStates = masterState.al_states;
                printf("master %d al_states changed to 0x%02x\n", ecat->order, alStates);
            }
            if(domainState.working_counter != workingCounter){
                workingCounter = domainState.working_counter;
                printf("master %d working_counter changed to %d\n", ecat->order, workingCounter);
            }
            if(domainState.wc_state != wcState){
                wcState = domainState.wc_state;
                printf("master %d wc_state changed to %d\n", ecat->order, wcState);
            }
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
    if(rxPDOSwap != nullptr){
        delete rxPDOSwap;
    }
    if(txPDOSwap != nullptr){
        delete txPDOSwap;
    }
    if(fd > 0){
        close(fd);
    }
    if(master != nullptr){
        ecrt_release_master(master);
    }
    sleep(1);
}

ECAT::~ECAT(){
    clean();
}
}