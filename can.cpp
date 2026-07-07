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
#include "hobot_can_hal.h"
#include "canbase.h"
#include <sys/epoll.h>

namespace DriverSDK{
#define BSWAP(X) (((unsigned int)(X) & 0xff000000) >> 24 | ((unsigned int)(X) & 0x00ff0000) >> 8 | ((unsigned int)(X) & 0x0000ff00) << 8 | ((unsigned int)(X) & 0x000000ff) << 24)

extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> canAlias2type;
extern std::vector<std::map<int, std::vector<int>>> canAlias2masterIDs;
extern std::vector<std::map<int, int>> canAlias2slaveID;
extern int dofEffector, imuCount;
extern std::vector<unsigned short> processorsCAN;
extern std::vector<unsigned short> maxCurrent;

int nullRXCAN(int const order, int const alias, int* const slaveID, unsigned char* const data, int* const rtr){
    return std::numeric_limits<int>::min();
}

void nullTXCAN(int const masterID, unsigned char* const data, int const length, CAN* const can){
    printf("unexpected data with master_id %d and length %d on cans[%d]\n", masterID, length, can->order);
}

pthread_t CAN::rxPth, CAN::txPth, CAN::txPth_;
int CAN::rxCPU, CAN::txCPU, CAN::txCPU_;
std::map<std::string, DriverParameters*> CAN::type2parameters;
unsigned short* CAN::alias2status;
DriverParameters** CAN::alias2parameters;
int CAN::orderSlaveID2alias[8][256];
int CAN::orderMasterID2slaveID[8][2048];
canRXFunction CAN::rxFuncs[8][256];
canTXFunction CAN::txFuncs[8][2048];

CAN::CAN(int const order, char const* device) : CANBase(order, device){
    static bool initialized = false;
    if(!initialized){
        rxPth = txPth = txPth_ = 0;
        rxCPU  = configXML->canAttribute("rx_cpu");
        txCPU  = configXML->canAttribute("tx_cpu");
        txCPU_ = configXML->canAttribute("tx_cpu_");
        adjustCPU(&rxCPU,  processorsCAN[0]);
        adjustCPU(&txCPU,  processorsCAN[1]);
        adjustCPU(&txCPU_, processorsCAN[2]);
        alias2status = new unsigned short[dofAll + 1];
        alias2parameters = new DriverParameters*[dofAll + 1];
        int i = 0, j;
        while(i <= dofAll){
            alias2status[i] = 0xffff;
            alias2parameters[i] = nullptr;
            ++i;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 2048){
                orderMasterID2slaveID[i][j] = -1;
                ++j;
            }
            ++i;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 256){
                orderSlaveID2alias[i][j] = 0;
                ++j;
            }
            ++i;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 256){
                rxFuncs[i][j] = nullRXCAN;
                ++j;
            }
            ++i;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 2048){
                txFuncs[i][j] = nullTXCAN;
                ++j;
            }
            ++i;
        }
        initialized = true;
    }
    rxSwap = txSwap = rxSwap_ = txSwap_ = rxSwap__ = txSwap__ = nullptr;
    MASK = mask = MASK_ = mask_ = MASK__ = mask__ = 0;
    rollingCounter = 0xff;
    alias2type = canAlias2type[order];
    alias2masterIDs = canAlias2masterIDs[order];
    alias2slaveID = canAlias2slaveID[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("cans[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        int alias = itr->first, slaveID = alias2slaveID.find(alias)->second;
        std::vector<int> masterIDs = alias2masterIDs.find(alias)->second;
        std::string const& type = itr->second;
        std::string const category = configXML->typeCategory("CAN", type.c_str());
        if(category == "driver"){
            if(type.starts_with("RealMan") && (slaveID < 1 || slaveID > 7)){
                printf("slave_id of RealMan driver with alias %d must be within [1, 7]\n", alias);
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
            masterIDs[i] &= 0x7ff;
            orderMasterID2slaveID[order][masterIDs[i]] = slaveID;
            if(txFuncs[order][masterIDs[i]] != nullTXCAN){
                printf("\ninvalid can bus configuration\n");
                exit(-1);
            }
            if(type.starts_with("Encos")){
                txFuncs[order][masterIDs[i]] = encosTX<CAN>;
            }else if(type.starts_with("Damiao")){
                txFuncs[order][masterIDs[i]] = damiaoTX<CAN>;
            }else if(type.starts_with("RealMan")){
                txFuncs[order][masterIDs[i]] = realManTX<CAN>;
            }else if(type.starts_with("CANopen")){
                txFuncs[order][masterIDs[i]] = canopenConfigTX<CAN>;
            }else if(type == "AGIBOT"){
                txFuncs[order][masterIDs[i]] = agibotTX<CAN>;
            }else if(type == "LinkerBot"){
                txFuncs[order][masterIDs[i]] = linkerBotTX<CAN>;
            }else if(type == "YESENSE"){
                txFuncs[order][masterIDs[i]] = yesenseTX<CAN>;
            }
            ++i;
        }
        printf(", slave_id %d\n", slaveID);
        orderSlaveID2alias[order][slaveID] = alias;
        if(rxFuncs[order][slaveID] != nullRXCAN){
            printf("invalid can bus configuration\n");
            exit(-1);
        }
        if(type.starts_with("Encos")){
            rxFuncs[order][slaveID] = encosRX<CAN>;
        }else if(type.starts_with("Damiao")){
            rxFuncs[order][slaveID] = damiaoRX<CAN>;
        }else if(type.starts_with("RealMan")){
            rxFuncs[order][slaveID] = realManRX<CAN>;
        }else if(type.starts_with("CANopen")){
            rxFuncs[order][slaveID] = canopenConfigRX<CAN>;
            canopenAliases.push_back(alias);
        }else if(type == "AGIBOT"){
            rxFuncs[order][slaveID] = agibotRX<CAN>;
        }else if(type == "LinkerBot"){
            rxFuncs[order][slaveID] = linkerBotRX<CAN>;
        }else if(type == "YESENSE"){
            rxFuncs[order][slaveID] = yesenseRX<CAN>;
        }
        ++itr;
    }
    slaveCount = alias2type.size();
}

int CAN::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    if(ifaceDown() == -1){
        return -1;
    }
    if(ifaceUp() == -1){
        return -1;
    }
    if(canhal == 0){    // Non D-Robotics CANHAL
        while(true){
            sock = open(0);
            if(sock == -1){
                return -1;
            }else if(sock == -2){
                usleep(500000);
                printf("retesting iface %s\n", device);
                continue;
            }else{
                break;
            }
        }
    }
    rxSwap   = new SwapList(dofAll      * sizeof(DriverRXData));
    txSwap   = new SwapList(dofAll      * sizeof(DriverTXData));
    rxSwap_  = new SwapList(dofEffector * sizeof( DigitRXData));
    txSwap_  = new SwapList(dofEffector * sizeof( DigitTXData));
    rxSwap__ = new SwapList(imuCount    * sizeof(   IMURXData));
    txSwap__ = new SwapList(imuCount    * sizeof(   IMUTXData));
    int k = 0;
    auto itr = alias2slaveID.begin();
    while(itr != alias2slaveID.end()){
        int alias = itr->first, slaveID = itr->second;
        std::string const& type = alias2type.find(alias)->second;
        std::string const category = configXML->typeCategory("CAN", type.c_str());
        if(category == "driver"){
            if(type.starts_with("CANopen")){
#ifndef NIIC
                if(drivers[alias - 1].init("CANopen", 2, order, 0, slaveID, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData), nullptr, nullptr) != 0){
#else
                if(drivers[alias - 1].init("CANopen", 2, order, 0, slaveID, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData)) != 0){
#endif
                    printf("\tdrivers[%d] init failed\n", alias - 1);
                    return -1;
                }
                if(drivers[alias - 1].config("CANopen", order, 0, rxSwap, txSwap) != 0){
                    printf("\tdrivers[%d] config failed\n", alias - 1);
                    return -1;
                }
            }else{
#ifndef NIIC
                if(drivers[alias - 1].init("CAN", 1, order, 0, slaveID, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData), nullptr, nullptr) != 0){
#else
                if(drivers[alias - 1].init("CAN", 1, order, 0, slaveID, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData)) != 0){
#endif
                    printf("\tdrivers[%d] init failed\n", alias - 1);
                    return -1;
                }
                if(drivers[alias - 1].config("CAN", order, 0, rxSwap, txSwap) != 0){
                    printf("\tdrivers[%d] config failed\n", alias - 1);
                    return -1;
                }
            }
            MASK |= 1 << slaveID;
            ++k;
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
                if(digits[i].init("CAN", 1, order, 0, slaveID, alias, type, i * sizeof(DigitRXData), i * sizeof(DigitTXData), nullptr, nullptr) != 0){
#else
                if(digits[i].init("CAN", 1, order, 0, slaveID, alias, type, i * sizeof(DigitRXData), i * sizeof(DigitTXData)) != 0){
#endif
                    printf("\tdigits[%d] init failed\n", i);
                    return -1;
                }
                if(digits[i].config("CAN", order, 0, rxSwap_, txSwap_) != 0){
                    printf("\tdigits[%d] config failed\n", i);
                    return -1;
                }
                ++i;
            }
            MASK_ |= 1 << slaveID % 32;
        }else if(category == "imu"){
            int index = alias - 240;
            if(index < 0 || index > 15){
                printf("\tinvalid imu alias %d\n", alias);
                return -1;
            }
#ifndef NIIC
            if(imus[index].init("CAN", 1, order, 0, slaveID, alias, type, index * sizeof(IMURXData), index * sizeof(IMUTXData), nullptr, nullptr) != 0){
#else
            if(imus[index].init("CAN", 1, order, 0, slaveID, alias, type, index * sizeof(IMURXData), index * sizeof(IMUTXData)) != 0){
#endif
                printf("\timus[%d] init failed\n", index);
                return -1;
            }
            if(imus[index].config("CAN", order, 0, rxSwap__, txSwap__) != 0){
                printf("\timus[%d] config failed\n", index);
                return -1;
            }
            MASK__ |= 1 << slaveID % 32;
        }
        ++itr;
    }
    return 0;
}

void CAN::cleanup(void* arg){
    int* epfd = (int*)arg;
    if(*epfd > -1){
        close(*epfd);
        *epfd = -1;
    }
}

void* CAN::rx(void* arg){
    std::vector<CAN>& cans = *(std::vector<CAN>*)arg;
    unsigned int count = 0xffffffff;
    unsigned char data[128], * data_;
    struct canframe frames[64];
    struct pack_info packInfo;
    packInfo.length = 64;
    struct timespec currentTime, wakeupTime, step{0, 6 * period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        ++step.tv_sec;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        ++count;
        int i = 0;
        while(i < cans.size()){
            if((cans[i].rxSwap != nullptr || cans[i].rxSwap_ != nullptr) && count % cans[i].division == 0){
                if(cans[i].canhal == 1){
                    packInfo.data_num = 0;
                }
                auto itr = cans[i].alias2slaveID.begin();
                while(itr != cans[i].alias2slaveID.end()){
                    int alias = itr->first, slaveID = itr->second, rtr = 0, length = rxFuncs[i][slaveID](i, alias, &slaveID, data, &rtr);
                    if(length > -1){
                        data_ = data;
                    }else if(length == std::numeric_limits<int>::min()){
                        ++itr;
                        continue;
                    }else{
                        data_ = data + 64;
                        length = -length;
                    }
                    if(cans[i].canhal == 0){
                        if(cans[i].canfd == 1 || cans[i].device[0] == '/'){
                            cans[i].sendfd(slaveID, data_, rtr, length);
                        }else{
                            cans[i].send(slaveID, data_, rtr, length);
                        }
                    }else if(cans[i].canhal == 1){
                        int nr = packInfo.data_num;
                        frames[nr].canid = BSWAP(slaveID);
                        frames[nr].count = ++cans[i].rollingCounter;
                        frames[nr].can_type = rtr << 2 | cans[i].canfd << 1;
                        frames[nr].can_channel = cans[i].device[4] == '_' || cans[i].device[4] == '\0' ? cans[i].device[3] - '0' : 10 + cans[i].device[4] - '0';
                        frames[nr].len = length;
                        memcpy(frames[nr].data, data_, length);
                        ++packInfo.data_num;
                    }
                    ++itr;
                }
                if(cans[i].canhal == 1){
                    int ret = canSendMsgFrame(cans[i].device, frames, &packInfo);
                    if(ret < 1){
                        printf("canSendMsgFrame: cans[%d] write ret = %d\n", i, ret);
                    }
                }
            }
            ++i;
        }
        wakeupTime.tv_nsec += period;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            ++wakeupTime.tv_sec;
        }
        bool sleep = true;
        long diff = 0;
        do{
            if(sleep){
                nanosleep(&step, nullptr);
            }
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            diff = TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime);
            if(sleep){
                if(diff < - 3 * period / 4){
                    wakeupTime = currentTime;
                }else if(diff < 9 * period / 100){
                    sleep = false;
                }
            }
        }while(diff > 0);
    }
    return nullptr;
}

void* CAN::tx(void* arg){
    std::vector<CAN>& cans = *(std::vector<CAN>*)arg;
    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if(epfd == -1){
        printf("epoll_create1() error\n");
        exit(-1);
    }
    pthread_cleanup_push(cleanup, &epfd);
    int i = 0, sock2order[128];
    while(i < 128){
        sock2order[i] = -1;
        ++i;
    }
    i = 0;
    while(i < cans.size()){
        if(cans[i].sock > -1){
            sock2order[cans[i].sock] = i;
            struct epoll_event ev;
            ev.events = EPOLLIN;
            ev.data.fd = cans[i].sock;
            if(epoll_ctl(epfd, EPOLL_CTL_ADD, cans[i].sock, &ev) == -1){
                printf("epoll_ctl() EPOLL_CTL_ADD error, errno: %d\n", errno);
                pthread_exit(nullptr);
            }
        }
        ++i;
    }
    printf("epoll_waiting...\n");
    unsigned char data[64];
    struct epoll_event events[8];
    while(true){
        int count = epoll_wait(epfd, events, 8, -1);
        if(count == -1){
            printf("epoll_wait() error\n");
            continue;
        }
        i = 0;
        while(i < count){
            int order = sock2order[events[i].data.fd], masterID, length;
            CAN& can = cans[order];
            if(can.canfd == 1 || can.device[0] == '/'){
                length = can.recvfd(data, 64, &masterID);
            }else{
                length = can.recv(data, 64, &masterID);
            }
            if(length < 1){
                ++i;
                continue;
            }
            masterID &= 0x7ff;
            txFuncs[order][masterID](masterID, data, length, &cans[order]);
            ++i;
        }
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

void CAN::cleanup_(void* arg){
    std::vector<pthread_t> pths = *(std::vector<pthread_t>*)arg;
    int i = 0;
    while(i < pths.size()){
        if(pths[i] > 0){
            pthread_cancel(pths[i]);
            pths[i] = 0;
        }
        ++i;
    }
}

void* CAN::tx__(void* arg){
    CAN& can = *(CAN*)arg;
    struct canframe frames[32];
    struct pack_info packInfo;
    packInfo.length = 32;
#ifdef ENPHT
    struct timespec currentTime, wakeupTime;
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
#endif
    while(true){
        int ret = canRecvMsgFrame(can.device, frames, &packInfo);
        if(ret < 0){
            printf("canRecvMsgFrame: cans[%d] read ret = %d\n", can.order, ret);
            continue;
        }
        int i = 0;
        while(i < packInfo.data_num){
            int masterID = frames[i].canid, length = frames[i].len;
            if(length == 0){
                ++i;
                continue;
            }
            masterID &= 0x7ff;
            txFuncs[can.order][masterID](masterID, frames[i].data, length, &can);
            ++i;
        }
#ifdef ENPHT
        wakeupTime.tv_nsec += can.period * can.division;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            ++wakeupTime.tv_sec;
        }
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        if(TIMESPEC2NS(wakeupTime) > TIMESPEC2NS(currentTime)){
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);
        }else{
            wakeupTime = currentTime;
        }
#endif
    }
    return nullptr;
}

void* CAN::tx_(void* arg){
    std::vector<CAN>& cans = *(std::vector<CAN>*)arg;
    std::vector<pthread_t> pths;
    pthread_cleanup_push(cleanup_, &pths);
    int i = 0;
    while(i < cans.size()){
        if(cans[i].alias2type.size() < 1 || cans[i].canhal != 1){
            ++i;
            continue;
        }
        pthread_t pth;
        if(pthread_create(&pth, nullptr, &tx__, (void*)&cans[i]) != 0){
            printf("creating canhal tx__ thread failed\n");
            pthread_exit(nullptr);
        }
        pths.push_back(pth);
        ++i;
    }
    i = 0;
    while(i < pths.size()){
        pthread_join(pths[i], nullptr);
        ++i;
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

void CAN::transfer(int const alias, long const period, int const division, unsigned short const maxCurr, CANopenData rx){
    int slaveID = 0;
    if(alias != 0){
        slaveID = alias2slaveID.find(alias)->second;
        if((rx.data[2] == 0x14 || rx.data[2] == 0x18) && rx.data[3] == 0x01){
            unsigned short id = *(unsigned short*)(rx.data + 4);
            id += slaveID;
            *(unsigned short*)(rx.data + 4) = id;
        }
        if(rx.data[2] == 0x18 && rx.data[3] == 0x05){
            *(unsigned short*)(rx.data + 4) = division * period / 1000000;
        }
        if(rx.data[1] == 0xc2 && rx.data[2] == 0x60 && rx.data[3] == 0x01){
            *                 (rx.data + 4) = division * period / 1000000;
        }
        if(rx.data[1] == 0x72 && rx.data[2] == 0x60 && rx.data[3] == 0x00){
            *(unsigned short*)(rx.data + 4) = maxCurr;
        }
        if(rx.functionCode == 0){
            *                 (rx.data + 1) = slaveID;
            slaveID = 0;
        }
    }
    slaveID += rx.functionCode;
    if(canhal == 0){
        if(canfd == 1 || device[0] == '/'){
            sendfd(slaveID, rx.data, rx.rtr, rx.length);
        }else{
            send(slaveID, rx.data, rx.rtr, rx.length);
        }
    }else if(canhal == 1){
        struct canframe frames[1];
        frames[0].canid = BSWAP(slaveID);
        frames[0].count = ++rollingCounter;
        frames[0].can_type = rx.rtr << 2 | canfd << 1;
        frames[0].can_channel = device[3] - '0';
        frames[0].len = rx.length;
        memcpy(frames[0].data, rx.data, rx.length);
        struct pack_info packInfo;
        packInfo.length = 1;
        packInfo.data_num = 1;
        int ret = canSendMsgFrame(device, frames, &packInfo);
        if(ret < 1){
            printf("canSendMsgFrame: cans[%d] write ret = %d\n", order, ret);
        }
    }
}

int CAN::canopenConfig(){
    if(canopenAliases.size() == 0){
        return 0;
    }
    int i = 0;
    while(i < canopenAliases.size()){
        int j = 0, alias = canopenAliases[i];
        printf("configuring CANopen driver with alias %d\n", alias);
        std::vector<Correspondence> correspondences;
        while(j < 23){
            correspondences.push_back(Correspondences[j]);
            ++j;
        }
        tinyxml2::XMLElement* deviceXML = configXML->device("CAN", alias2type.find(alias)->second.c_str());
        std::vector<std::vector<std::string>> rxPDOs = configXML->pdos(deviceXML, "RxPDOs"), txPDOs = configXML->pdos(deviceXML, "TxPDOs");
        std::vector<unsigned short> rxIndices, txIndices;
        int m = 0;
        while(m < rxPDOs.size()){
            unsigned short index = strtoul(rxPDOs[m][0].c_str(), nullptr, 16);
            Correspondence correspondence = Correspondences[23];
            *(unsigned short*)(correspondence.rx.data + 1) = index;
            *(unsigned short*)(correspondence.tx.data + 1) = index;
            correspondences.push_back(correspondence);
            rxIndices.push_back(index);
            int n = 1;
            while(n < rxPDOs[m].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, rxPDOs[m][n].c_str());
                correspondence = Correspondences[24];
                *(unsigned short*)(correspondence.rx.data + 1) = index;
                *                 (correspondence.rx.data + 3) = n;
                *(unsigned short*)(correspondence.rx.data + 6) = strtoul(entry[1].c_str(), nullptr, 16);
                *                 (correspondence.rx.data + 5) = strtoul(entry[2].c_str(), nullptr, 16);
                *                 (correspondence.rx.data + 4) = strtoul(entry[4].c_str(), nullptr, 10);
                *(unsigned short*)(correspondence.tx.data + 1) = index;
                *                 (correspondence.tx.data + 3) = n;
                correspondences.push_back(correspondence);
                ++n;
            }
            correspondence = Correspondences[23];
            *(unsigned short*)(correspondence.rx.data + 1) = index;
            *                 (correspondence.rx.data + 4) = rxPDOs[m].size() - 1;
            *(unsigned short*)(correspondence.tx.data + 1) = index;
            correspondences.push_back(correspondence);
            ++m;
        }
        m = 0;
        while(m < txPDOs.size()){
            unsigned short index = strtoul(txPDOs[m][0].c_str(), nullptr, 16);
            Correspondence correspondence = Correspondences[25];
            *(unsigned short*)(correspondence.rx.data + 1) = index;
            *(unsigned short*)(correspondence.tx.data + 1) = index;
            correspondences.push_back(correspondence);
            txIndices.push_back(index);
            int n = 1;
            while(n < txPDOs[m].size()){
                std::vector<std::string> entry = configXML->entry(deviceXML, txPDOs[m][n].c_str());
                correspondence = Correspondences[26];
                *(unsigned short*)(correspondence.rx.data + 1) = index;
                *                 (correspondence.rx.data + 3) = n;
                *(unsigned short*)(correspondence.rx.data + 6) = strtoul(entry[1].c_str(), nullptr, 16);
                *                 (correspondence.rx.data + 5) = strtoul(entry[2].c_str(), nullptr, 16);
                *                 (correspondence.rx.data + 4) = strtoul(entry[4].c_str(), nullptr, 10);
                *(unsigned short*)(correspondence.tx.data + 1) = index;
                *                 (correspondence.tx.data + 3) = n;
                correspondences.push_back(correspondence);
                ++n;
            }
            correspondence = Correspondences[25];
            *(unsigned short*)(correspondence.rx.data + 1) = index;
            *                 (correspondence.rx.data + 4) = txPDOs[m].size() - 1;
            *(unsigned short*)(correspondence.tx.data + 1) = index;
            correspondences.push_back(correspondence);
            ++m;
        }
        m = 0;
        while(m < rxIndices.size()){
            correspondences.push_back(Correspondences[27 + rxIndices[m] - 0x1600]);
            ++m;
        }
        m = 0;
        while(m < txIndices.size()){
            correspondences.push_back(Correspondences[31 + txIndices[m] - 0x1a00]);
            ++m;
        }
        correspondences.push_back(Correspondences[35]);
        j = 0;
        while(j < correspondences.size()){
            do{
                transfer(alias, period, txIndices.size(), maxCurrent[canopenAliases[i] - 1], correspondences[j].rx);
                checkMutex.lock();
                checkSlaveIDs.clear();
                if(correspondences[j].tx.functionCode != 0x000){
                    checkSlaveIDs.push_back(alias2slaveID.find(alias)->second);
                    checkData = correspondences[j].tx;
                }
                checkMutex.unlock();
            }while(!canopenCheck<CAN>(period, 20));
            ++j;
        }
        ++i;
    }
    /* while(true){
        transfer(0, period, 1, 1000, Correspondence0.rx);
        usleep(period / 1000);
        int i = 0;
        while(i < canopenAliases.size()){
            int tryCount = 0;
            do{
                ++tryCount;
                if(tryCount > 4){
                    break;
                }
                transfer(canopenAliases[i], period, 1, maxCurrent[canopenAliases[i] - 1], Correspondence_.rx);
                checkMutex.lock();
                checkData = Correspondence_.tx;
                checkSlaveIDs.clear();
                checkSlaveIDs.push_back(alias2slaveID.find(canopenAliases[i])->second);
                checkMutex.unlock();
            }while(!canopenCheck<CAN>(period, 20));
            if(tryCount > 4){
                break;
            }
            ++i;
        }
        if(i < canopenAliases.size()){
            continue;
        }
        break;
    } */
    transfer(0, period, 1, 1000, Correspondence0.rx);
    i = 0;
    while(i < canopenAliases.size()){
        int alias = canopenAliases[i], slaveID = alias2slaveID.find(alias)->second;
        std::vector<int> masterIDs = alias2masterIDs.find(alias)->second;
        std::string const& type = alias2type.find(alias)->second;
        std::string const subType = type.length() > 8 ? type.substr(8) : "Eyou";
        int j = 0;
        while(j < masterIDs.size()){
            if(orderMasterID2slaveID[order][masterIDs[j]] == slaveID){
                if(subType == "Eyou"){
                    txFuncs[order][masterIDs[j]] = canopenEyouTX<CAN>;
                }else if(subType == "Elmo"){
                    txFuncs[order][masterIDs[j]] = canopenElmoTX<CAN>;
                }else{
                    printf("invalid CANopen driver subType %s\n", subType.c_str());
                    return -1;
                }
            }
            ++j;
        }
        if(subType == "Eyou"){
            rxFuncs[order][slaveID] = canopenEyouRX<CAN>;
        }else if(subType == "Elmo"){
            rxFuncs[order][slaveID] = canopenElmoRX<CAN>;
        }else{
            printf("invalid CANopen driver subType %s\n", subType.c_str());
            return -1;
        }
        ++i;
    }
    return 0;
}

int CAN::run(std::vector<CAN>& cans){
    int i = 0, j = 0;
    while(i < cans.size()){
        if(cans[i].alias2type.size() > 0){
            if(cans[i].canhal == 1){
                ++CANHAL;
            }
            ++j;
        }
        ++i;
    }
    if(j == 0){
        return 0;
    }
    if(CANHAL > 0 && canInit() < 0){
        printf("canhal init failed\n");
        CANHAL = 0;
        return -1;
    }
    bool socketCAN = false;
    if(j > CANHAL){
        socketCAN = true;
    }
    i = 0;
    while(i < cans.size()){
        printf("cans[%d]:\t", i);
        j = 0;
        while(j < 256){
            int a = orderSlaveID2alias[i][j];
            if(j < 32 && a <= dofAll){
                printf("%02d ", a);
            }
            ++j;
        }
        printf(" MASK: 0x%08x\n", cans[i].MASK);
        ++i;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(rxCPU, &cpuset);
    if(pthread_create(&rxPth, nullptr, &rx, (void*)&cans) != 0){
        printf("creating can rx thread failed\n");
        return -1;
    }
    if(pthread_setaffinity_np(rxPth, sizeof(cpuset), &cpuset) != 0){
        printf("setting can rx thread cpu affinity failed\n");
        return -1;
    }
    if(pthread_detach(rxPth) != 0){
        printf("detaching can rx thread failed\n");
        return -1;
    }
    printf("can rx on cpu %d\n", rxCPU);
    if(socketCAN){
        CPU_ZERO(&cpuset);
        CPU_SET(txCPU, &cpuset);
        if(pthread_create(&txPth, nullptr, &tx, (void*)&cans) != 0){
            printf("creating socketcan tx thread failed\n");
            return -1;
        }
        if(pthread_setaffinity_np(txPth, sizeof(cpuset), &cpuset) != 0){
            printf("setting socketcan tx thread cpu affinity failed\n");
            return -1;
        }
        if(pthread_detach(txPth) != 0){
            printf("detaching socketcan tx thread failed\n");
            return -1;
        }
        printf("socketcan tx on cpu %d\n", txCPU);
    }
    if(CANHAL > 0){
        CPU_ZERO(&cpuset);
        CPU_SET(txCPU_, &cpuset);
        if(pthread_create(&txPth_, nullptr, &tx_, (void*)&cans) != 0){
            printf("creating canhal tx_ thread failed\n");
            return -1;
        }
        if(pthread_setaffinity_np(txPth_, sizeof(cpuset), &cpuset) != 0){
            printf("setting canhal tx_ thread cpu affinity failed\n");
            return -1;
        }
        if(pthread_detach(txPth_) != 0){
            printf("detaching canhal tx_ thread failed\n");
            return -1;
        }
        printf("canhal tx_ on cpu %d\n", txCPU_);
    }
    i = 0;
    while(i < cans.size()){
        if(cans[i].canopenConfig() != 0){
            printf("cans[%d] CANopen config failed\n", i);
            return -1;
        }
        ++i;
    }
    return 0;
}

CAN::~CAN(){
    std::lock_guard<std::mutex> guard(resourceMutex);
    if(rxPth > 0){
        pthread_cancel(rxPth);
        rxPth = 0;
    }
    if(txPth > 0){
        pthread_cancel(txPth);
        txPth = 0;
    }
    if(txPth_ > 0){
        pthread_cancel(txPth_);
        txPth_ = 0;
    }
    auto itr = type2parameters.begin();
    while(itr != type2parameters.end()){
        delete itr->second;
        ++itr;
    }
    type2parameters.clear();
    if(alias2status != nullptr){
        delete[] alias2status;
        alias2status = nullptr;
    }
    if(alias2parameters != nullptr){
        delete[] alias2parameters;
        alias2parameters = nullptr;
    }
    if(sock > -1){
        close(sock);
        sock = -1;
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
    if(rxSwap__ != nullptr){
        delete rxSwap__;
        rxSwap__ = nullptr;
    }
    if(txSwap__ != nullptr){
        delete txSwap__;
        txSwap__ = nullptr;
    }
}
}