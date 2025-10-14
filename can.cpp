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
#include "can.h"
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <sys/epoll.h>
#include <limits>

namespace DriverSDK{
#define TXQUEUELEN 100

extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> canAlias2type;
extern std::vector<std::map<int, int>> canAlias2masterID, canAlias2slaveID;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern unsigned short processorCAN;

unsigned short float2para(float const f, float const min, float const max, int const bit){
    return (f - min) * ((1 << bit) - 1.0) / (max - min);
}

float para2float(unsigned short const us, float const min, float const max, int const bit){
    return us * (max - min) / ((1 << bit) - 1.0) + min;
}

int nullRX(int const alias, unsigned char* const data){
    return 0;
}

void nullTX(int const order, int id, unsigned char* const data, int const length, CAN* const can){
}

unsigned char const EncosEnable [3] = {0x71, 0x03, 0xe8};
unsigned char const EncosDisable[3] = {0x6d, 0x00, 0x00};
unsigned char const EncosDamp   [3] = {0x69, 0x00, 0x00};

int encosRX(int const alias, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 2:
        memcpy(data, EncosDamp, 3);
        CAN::alias2status[alias] = 0x0037;
        return 3;
        break;
    case 1:
        switch(CAN::alias2status[alias]){
        case 0x0037:
            break;
        case 0x0031:
            memcpy(data, EncosEnable, 3);
            CAN::alias2status[alias] = 0x0037;
            return 3;
            break;
        }
        break;
    case 0:
        switch(CAN::alias2status[alias]){
        case 0x0037:
            memcpy(data, EncosDisable, 3);
            CAN::alias2status[alias] = 0x0031;
            return 3;
            break;
        case 0x0031:
            break;
        }
        break;
    }
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    unsigned short p = 0, v = 0, t = 0, kp = 0, kd = 0;
    if(CAN::alias2status[alias] != 0x0037){
         p = float2para(0.0,  parameters->minP,  parameters->maxP, 16);
         v = float2para(0.0,  parameters->minV,  parameters->maxV, 12);
        kp = float2para(0.0, parameters->minKp, parameters->maxKp, 12);
        kd = float2para(0.0, parameters->minKd, parameters->maxKd,  9);
         t = float2para(0.0,  parameters->minT,  parameters->maxT, 12);
    }else{
         p = float2para(*reinterpret_cast<float*>(&drivers[alias - 1].rx.previous()->TargetPosition),  parameters->minP,  parameters->maxP, 16);
         v = float2para(*reinterpret_cast<float*>(&drivers[alias - 1].rx.previous()->TargetVelocity),  parameters->minV,  parameters->maxV, 12);
        kp = float2para(              half2single( drivers[alias - 1].rx.previous()->ControlWord   ), parameters->minKp, parameters->maxKp, 12);
        kd = float2para(              half2single( drivers[alias - 1].rx.previous()->TargetTorque  ), parameters->minKd, parameters->maxKd,  9);
         t = float2para(              half2single( drivers[alias - 1].rx.previous()->TorqueOffset  ),  parameters->minT,  parameters->maxT, 12);
    }
    *(data + 0) = kp >> 7;
    *(data + 1) = kp << 1 & 0x00ff | kd >> 8;
    *(data + 2) = kd & 0x00ff;
    *(data + 3) =  p >> 8;
    *(data + 4) =  p & 0x00ff;
    *(data + 5) =  v >> 4;
    *(data + 6) =  v << 4 & 0x00ff |  t >> 8;
    *(data + 7) =  t & 0x00ff;
    return 8;
}

void encosTX(int const order, int id, unsigned char* const data, int const length, CAN* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] & 0x1f;
    data[0] = data[2];
    unsigned short p = *reinterpret_cast<unsigned short*>(data + 0);
    data[2] = data[4];
    unsigned short v = *reinterpret_cast<unsigned short*>(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *reinterpret_cast<unsigned short*>(data + 3);
    int const alias = CAN::orderSlaveID2alias[order][id];
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualPosition) =             para2float(p, parameters->minP, parameters->maxP, 16);
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualVelocity) =             para2float(v, parameters->minV, parameters->maxV, 12);
                               drivers[alias - 1].tx.next()->ActualTorque    = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
                               drivers[alias - 1].tx.next()->Undefined       = (data[7] - 50) / 2;
                               drivers[alias - 1].tx.next()->StatusWord      = err > 0 ? 0x0018 : CAN::alias2status[alias];
                               drivers[alias - 1].tx.next()->ErrorCode       = err;
    can->mask |= 1 << id;
    if(can->mask == can->MASK){
        can->txSwap->advanceNodePtr();
        can->mask = 0;
    }
}

unsigned char const DamiaoEnable [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
unsigned char const DamiaoDisable[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
unsigned char const DamiaoClrErr [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

int damiaoRX(int const alias, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 1:
        switch(CAN::alias2status[alias]){
        case 0x0037:
            break;
        case 0x0031:
            memcpy(data, DamiaoEnable, 8);
            CAN::alias2status[alias] = 0x0037;
            return 8;
            break;
        }
        break;
    case 0:
        switch(CAN::alias2status[alias]){
        case 0x0037:
            memcpy(data, DamiaoDisable, 8);
            CAN::alias2status[alias] = 0x0031;
            return 8;
            break;
        case 0x0031:
            break;
        }
        break;
    case -1:
        memcpy(data, DamiaoClrErr, 8);
        CAN::alias2status[alias] = 0x0031;
        return 8;
        break;
    }
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    unsigned short  p = float2para(*reinterpret_cast<float*>(&drivers[alias - 1].rx.previous()->TargetPosition),  parameters->minP,  parameters->maxP, 16);
    unsigned short  v = float2para(*reinterpret_cast<float*>(&drivers[alias - 1].rx.previous()->TargetVelocity),  parameters->minV,  parameters->maxV, 12);
    unsigned short kp = float2para(              half2single( drivers[alias - 1].rx.previous()->ControlWord   ), parameters->minKp, parameters->maxKp, 12);
    unsigned short kd = float2para(              half2single( drivers[alias - 1].rx.previous()->TargetTorque  ), parameters->minKd, parameters->maxKd, 12);
    unsigned short  t = float2para(              half2single( drivers[alias - 1].rx.previous()->TorqueOffset  ),  parameters->minT,  parameters->maxT, 12);
    *(data + 0) =  p >> 8;
    *(data + 1) =  p & 0x00ff;
    *(data + 2) =  v >> 4;
    *(data + 3) =  v << 4 & 0x00ff | kp >> 8;
    *(data + 4) = kp & 0x00ff;
    *(data + 5) = kd >> 4;
    *(data + 6) = kd << 4 & 0x00ff |  t >> 8;
    *(data + 7) =  t & 0x00ff;
    return 8;
}

void damiaoTX(int const order, int id, unsigned char* const data, int const length, CAN* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] >> 4;
    id = data[0] & 0x0f;
    data[0] = data[2];
    unsigned short p = *reinterpret_cast<unsigned short*>(data + 0);
    data[2] = data[4];
    unsigned short v = *reinterpret_cast<unsigned short*>(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *reinterpret_cast<unsigned short*>(data + 3);
    int const alias = CAN::orderSlaveID2alias[order][id];
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualPosition) =             para2float(p, parameters->minP, parameters->maxP, 16);
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualVelocity) =             para2float(v, parameters->minV, parameters->maxV, 12);
                               drivers[alias - 1].tx.next()->ActualTorque    = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
                               drivers[alias - 1].tx.next()->Undefined       = data[6];
                               drivers[alias - 1].tx.next()->StatusWord      = err > 0 ? 0x0018 : CAN::alias2status[alias];
                               drivers[alias - 1].tx.next()->ErrorCode       = err > 1 ? err : 0x0000;
    can->mask |= 1 << id;
    if(can->mask == can->MASK){
        can->txSwap->advanceNodePtr();
        can->mask = 0;
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
    if( minP  == std::numeric_limits<float>::min() ||
        maxP  == std::numeric_limits<float>::min() ||
        minV  == std::numeric_limits<float>::min() ||
        maxV  == std::numeric_limits<float>::min() ||
        minKp == std::numeric_limits<float>::min() ||
        maxKp == std::numeric_limits<float>::min() ||
        minKd == std::numeric_limits<float>::min() ||
        maxKd == std::numeric_limits<float>::min() ||
        minT  == std::numeric_limits<float>::min() ||
        maxT  == std::numeric_limits<float>::min()){
        printf("a driver parameter is incorrectly set in xml\n");
        return -1;
    }
    return 0;
}

void DriverParameters::print(){
    printf("P[%f, %f], V[%f, %f], Kp[%f, %f], Kd[%f, %f], T[%f, %f]\n", minP, maxP, minV, maxV, minKp, maxKp, minKd, maxKd, minT, maxT);
}

DriverParameters::~DriverParameters(){
}

long CAN::period;
pthread_t CAN::rxPth, CAN::txPth;
std::map<std::string, DriverParameters*> CAN::type2parameters;
int* CAN::alias2masterID_;
unsigned short* CAN::alias2status;
DriverParameters** CAN::alias2parameters;
int CAN::orderSlaveID2alias[8][16];
canRXFunction CAN::rxFuncs[2048][8];
canTXFunction CAN::txFuncs[2048][8];

CAN::CAN(int const order, char const* device){
    rxSwap = nullptr;
    txSwap = nullptr;
    sock = -1;
    MASK = 0;
    mask = 0;
    this->order = order;
    alias2type = canAlias2type[order];
    alias2masterID = canAlias2masterID[order];
    alias2slaveID = canAlias2slaveID[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("cans[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        int alias = itr->first, masterID = alias2masterID.find(alias)->second, slaveID = alias2slaveID.find(alias)->second;
        alias2masterID_[alias] = masterID;
        alias2status[alias] = 0x0031;
        std::string const& type = itr->second;
        auto itr_ = type2parameters.find(type);
        if(itr_ == type2parameters.end()){
            std::tie(itr_, std::ignore) = type2parameters.insert(std::make_pair(type, new DriverParameters()));
            itr_->second->load(itr_->first);
        }
        alias2parameters[alias] = itr_->second;
        orderSlaveID2alias[order][slaveID] = alias;
        printf("\talias %d, type %s, master_id %d, slave_id %d\n\t\t", alias, type.c_str(), masterID, slaveID);
        alias2parameters[alias]->print();
        itr++;
    }
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    baudrate  = configXML->attribute("CAN", order, "baudrate");
    canfd     = configXML->feature("CAN", order, "canfd");
    dbaudrate = configXML->attribute("CAN", order, "dbaudrate");
    division  = configXML->attribute("CAN", order, "division");
}

int CAN::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    ifaceUp();
    sock = open(0);
    if(sock < 0){
        return -1;
    }
    rxSwap = new SwapList(dofAll * sizeof(DriverRxData));
    txSwap = new SwapList(dofAll * sizeof(DriverTxData));
    int i = 0;
    auto itr = alias2slaveID.begin();
    while(itr != alias2slaveID.end()){
        int alias = itr->first, slave = itr->second;
        std::string type = alias2type.find(alias)->second;
        if(drivers[alias - 1].init("CAN", 1, order, 0, slave, alias, type, i * sizeof(DriverRxData), i * sizeof(DriverTxData), nullptr) != 0){
            printf("\tdrivers[%d] init failed\n", alias - 1);
            return -1;
        }
        switch(drivers[alias - 1].config("CAN", order, 0, rxSwap, txSwap)){
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
        itr++;
        i++;
    }
    return 0;
}

int CAN::ifaceIsUp(){
    int ret;
    struct ifreq ifr;
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock < 0){
        printf("creating socket failed\n");
        return -1;
    }
    strcpy(ifr.ifr_name, device);
    ioctl(sock, SIOCGIFFLAGS, &ifr);
    if(ifr.ifr_ifru.ifru_flags & IFF_RUNNING){
        ret = 1;
    }else{
        ret = 0;
    }
    close(sock);
    return ret;
}

int CAN::ifaceUp(){
    char cmd[256];
    int length = 0;
    if(ifaceIsUp() == 1){
        printf("iface %s is up\n", device);
        return 0;
    }
    printf("starting iface %s...\n", device);
    length += snprintf(cmd + length, sizeof(cmd) - length, "sudo ip link set %s up type can bitrate %d", device, baudrate);
    if(canfd > 0){
        length += snprintf(cmd + length, sizeof(cmd) - length, " fd on dbitrate %d", dbaudrate);
    }
    system(cmd);
    sleep(1);
    length = snprintf(cmd, sizeof(cmd), "sudo ifconfig %s txqueuelen %d", device, TXQUEUELEN);
    system(cmd);
    sleep(1);
    printf("iface %s should have started\n", device);
    return 0;
}

int CAN::ifaceDown(){
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "sudo ifconfig %s down", device);
    system(cmd);
    sleep(1);
    return 0;
}

int CAN::open(int const masterID){
    int sock;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_filter rfilter[1];
    if(ifaceIsUp() != 1){
        printf("iface %s is down\n", device);
        return -1;
    }
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock < 0){
        printf("creating socket failed\n");
        return -1;
    }else if(sock >= 128){
        printf("too many sockets\n");
        return -1;
    }
    strcpy(ifr.ifr_name, device);
    ioctl(sock, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0){
        printf("binding failed\n");
        close(sock);
        return -1;
    }
    if(masterID > 0){
        rfilter[0].can_id = masterID;
        rfilter[0].can_mask = CAN_SFF_MASK;
        if(setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) != 0){
            printf("setsockopt CAN_RAW_FILTER failed\n");
            close(sock);
            return -1;
        }
    }
    if(canfd > 0){
        if(setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd, sizeof(canfd)) != 0){
            printf("setsockopt CAN_RAW_FD_FRAMES failed\n");
            close(sock);
            return -1;
        }
    }
    int rcvbufSize = 128 * 1024;
    if(setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbufSize, sizeof(rcvbufSize)) != 0){
        printf("setsockopt SO_RCVBUF failed\n");
        close(sock);
        return -1;
    }
    int sndbufSize = 128 * 1024;
    if(setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbufSize, sizeof(sndbufSize)) != 0){
        printf("setsockopt SO_SNDBUF failed\n");
        close(sock);
        return -1;
    }
    int loopback = 0;
    if(setsockopt(sock, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) != 0){
        printf("setsockopt CAN_RAW_LOOPBACK failed\n");
        close(sock);
        return -1;
    }
    return sock;
}

int CAN::send(int const slaveID, unsigned char const* data, int length){
    int ret;
    struct can_frame frame;
    if(length > sizeof(frame.data)){
        printf("send: cans[%d] send data length cannot be greater than %ld\n", order, sizeof(frame.data));
        return -1;
    }
    frame.can_id = slaveID;
    frame.can_dlc = length;
    memcpy(frame.data, data, length);
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        printf("send: cans[%d] write ret = %d\n", order, ret);
        return -1;
    }
    return frame.can_id;
}

int CAN::recv(unsigned char* const data, int const length, int* const masterID){
    int ret;
    struct can_frame frame;
    ret = read(sock, &frame, sizeof(frame));
    if(ret <= 0){
        printf("recv: cans[%d] read ret = %d\n", order, ret);
        return ret;
    }
    if(masterID != nullptr){
        *masterID = frame.can_id;
    }
    if(length < frame.can_dlc){
        printf("recv: cans[%d] recv data buffer is too small\n", order);
        memcpy(data, frame.data, length);
        return length;
    }else{
        memcpy(data, frame.data, frame.can_dlc);
    }
    return frame.can_dlc;
}

int CAN::sendfd(int const slaveID, unsigned char const* data, int const length){
    int ret;
    struct canfd_frame frame;
    if(length > sizeof(frame.data)){
        printf("sendfd: cans[%d] send data length cannot be greater than %ld\n", order, sizeof(frame.data));
        return -1;
    }
    frame.can_id = slaveID;
    frame.len = length;
    frame.flags = 0x0f;
    memcpy(frame.data, data, length);
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        printf("sendfd: cans[%d] write ret = %d\n", order, ret);
        return -1;
    }
    return frame.can_id;
}

int CAN::recvfd(unsigned char* const data, int const length, int* const masterID){
    int ret;
    struct canfd_frame frame;
    ret = read(sock, &frame, sizeof(frame));
    if(ret <= 0){
        printf("recvfd: cans[%d] read ret = %d\n", order, ret);
        return ret;
    }
    if(masterID != nullptr){
        *masterID = frame.can_id;
    }
    if(length < frame.len){
        printf("recvfd: cans[%d] recv data buffer is too small\n", order);
        memcpy(data, frame.data, length);
        return length;
    }else{
        memcpy(data, frame.data, frame.len);
    }
    return frame.len;
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
    unsigned char data[64];
    struct timespec currentTime, wakeupTime, step{0, 6 * period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        count++;
        int i = 0;
        while(i < cans.size()){
            if(cans[i].rxSwap != nullptr && count % cans[i].division == 0){
                auto itr = cans[i].alias2slaveID.begin();
                while(itr != cans[i].alias2slaveID.end()){
                    int alias = itr->first, length = rxFuncs[alias2masterID_[alias]][i](alias, data);
                    if(cans[i].canfd > 0){
                        cans[i].sendfd(itr->second, data, length);
                    }else{
                        cans[i].send(itr->second, data, length);
                    }
                    itr++;
                }
            }
            i++;
        }
        wakeupTime.tv_nsec += period;
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
            if(sleep && (TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime) < 9 * period / 100)){
                sleep = false;
            }
        }while(TIMESPEC2NS(currentTime) < TIMESPEC2NS(wakeupTime));
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
    int i = 0, sock2order[128];
    while(i < 128){
        sock2order[i] = -1;
        i++;
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
                exit(-1);
            }
        }
        i++;
    }
    unsigned char data[64];
    struct epoll_event events[8];
    pthread_cleanup_push(cleanup, &epfd);
    printf("epoll_waiting...\n");
    while(true){
        int count = epoll_wait(epfd, events, 8, -1);
        if(count == -1){
            printf("epoll_wait() error\n");
            continue;
        }
        i = 0;
        while(i < count){
            int order = sock2order[events[i].data.fd], masterID = 0, length = 0;
            CAN& can = cans[order];
            if(can.canfd > 0){
                length = can.recvfd(data, 64, &masterID);
            }else{
                length = can.recv(data, 64, &masterID);
            }
            if(length <= 0){
                i++;
                continue;
            }
            txFuncs[masterID][order](order, masterID, data, length, &cans[order]);
            i++;
        }
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

int CAN::run(std::vector<CAN>& cans){
    int i = 0, j = 0;
    while(i < cans.size()){
        if(cans[i].alias2type.size() > 0){
             j++;
        }
        i++;
    }
    if(j == 0){
        return 0;
    }
    period = configXML->canPeriod();
    i = 1;
    while(i <= dofAll){
        printf("alias %2d, master_id %2d, status %d\n", i, alias2masterID_[i], alias2status[i]);
        i++;
    }
    i = 0;
    while(i < 8){
        printf("can%d:\t", i);
        j = 0;
        while(j < 16){
            if(orderSlaveID2alias[i][j] > 0){
                cans[i].MASK |= 1 << j;
            }
            printf("%2d ", orderSlaveID2alias[i][j]);
            j++;
        }
        printf(" mask: 0x%04x\n", cans[i].MASK);
        i++;
    }
    i = 0;
    while(i < 2048){
        j = 0;
        while(j < 8){
            rxFuncs[i][j] = nullptr;
            txFuncs[i][j] = nullptr;
            j++;
        }
        i++;
    }
    std::vector<std::tuple<int, std::vector<int>, std::string>> canBus = configXML->canBus();
    i = 0;
    while(i < canBus.size()){
        std::string type;
        int masterID;
        std::vector<int> masters;
        std::tie(masterID, masters, type) = canBus[i];
        printf("master_id %2d, masters", masterID);
        j = 0;
        while(j < masters.size()){
            if(rxFuncs[masterID][masters[j]] != nullptr || txFuncs[masterID][masters[j]] != nullptr){
                printf("\ninvalid can bus configuration\n");
                return -1;
            }
            if(type.starts_with("Encos")){
                rxFuncs[masterID][masters[j]] = encosRX;
                txFuncs[masterID][masters[j]] = encosTX;
            }else if(type.starts_with("Damiao")){
                rxFuncs[masterID][masters[j]] = damiaoRX;
                txFuncs[masterID][masters[j]] = damiaoTX;
            }else{
                rxFuncs[masterID][masters[j]] = nullRX;
                txFuncs[masterID][masters[j]] = nullTX;
            }
            printf(" %d", masters[j]);
            j++;
        }
        printf(", type %s\n", type.c_str());
        i++;
    }
    int cpu = sysconf(_SC_NPROCESSORS_ONLN) - 1;
    if(cpu > processorCAN){
        cpu = processorCAN;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if(pthread_create(&rxPth, nullptr, &rx, (void*)&cans) != 0){
        printf("creating can rx thread failed\n");
        return -1;
    }
    if(pthread_setaffinity_np(rxPth, sizeof(cpu_set_t), &cpuset) != 0){
        printf("setting can rx thread cpu affinity failed\n");
        return -1;
    }
    if(pthread_detach(rxPth) != 0){
        printf("detaching can rx thread failed\n");
        return -1;
    }
    printf("can rx on cpu %d\n", cpu);
    if(pthread_create(&txPth, nullptr, &tx, (void*)&cans) != 0){
        printf("creating can tx thread failed\n");
        return -1;
    }
    if(pthread_setaffinity_np(txPth, sizeof(cpu_set_t), &cpuset) != 0){
        printf("setting can tx thread cpu affinity failed\n");
        return -1;
    }
    if(pthread_detach(txPth) != 0){
        printf("detaching can tx thread failed\n");
        return -1;
    }
    printf("can tx on cpu %d\n", cpu);
    return 0;
}

CAN::~CAN(){
    if(rxPth > 0){
        pthread_cancel(rxPth);
        rxPth = 0;
    }
    if(txPth > 0){
        pthread_cancel(txPth);
        txPth = 0;
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
    if(device != nullptr){
        ifaceDown();
        free(device);
        device = nullptr;
    }
}
}