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
#include "can.h"
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <limits>

namespace DriverSDK{
#define TXQUEUELEN 100
#define CANFD_BRS 0x01
#define CANFD_ESI 0x02
#define CANFD_FDF 0x04
#define BSWAP(X) (((unsigned int)(X) & 0xff000000) >> 24 | ((unsigned int)(X) & 0x00ff0000) >> 8 | ((unsigned int)(X) & 0x0000ff00) << 8 | ((unsigned int)(X) & 0x000000ff) << 24)

extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> canAlias2type;
extern std::vector<std::map<int, int>> canAlias2masterID, canAlias2slaveID;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern std::vector<unsigned short> processorsCAN;

unsigned short float2para(float const f, float const min, float const max, int const bit){
    return (f - min) * ((1 << bit) - 1.0) / (max - min);
}

float para2float(unsigned short const us, float const min, float const max, int const bit){
    return us * (max - min) / ((1 << bit) - 1.0) + min;
}

int nullRX(int const alias, unsigned char* const data){
    return 0;
}

void nullTX(int const order, int id, unsigned char* const data, int const length, CANDriver* const can){
}

unsigned char const EncosEnable [3] = {0x71, 0x03, 0xe8};
unsigned char const EncosDisable[3] = {0x6d, 0x00, 0x00};
unsigned char const EncosDamp   [3] = {0x69, 0x00, 0x00};

int encosRX(int const alias, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 2:
        memcpy(data, EncosDamp, 3);
        CANDriver::alias2status[alias] = 0x0037;
        return 3;
        break;
    case 1:
        switch(CANDriver::alias2status[alias]){
        case 0x0037:
            break;
        case 0x0031:
            memcpy(data, EncosEnable, 3);
            CANDriver::alias2status[alias] = 0x0037;
            return 3;
            break;
        }
        break;
    case 0:
        switch(CANDriver::alias2status[alias]){
        case 0x0037:
            memcpy(data, EncosDisable, 3);
            CANDriver::alias2status[alias] = 0x0031;
            return 3;
            break;
        case 0x0031:
            break;
        }
        break;
    }
    DriverParameters const* parameters = CANDriver::alias2parameters[alias];
    unsigned short p = 0, v = 0, t = 0, kp = 0, kd = 0;
    if(CANDriver::alias2status[alias] != 0x0037){
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

void encosTX(int const order, int id, unsigned char* const data, int const length, CANDriver* const can){
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
    int const alias = CANDriver::orderSlaveID2alias[order][id];
    DriverParameters const* parameters = CANDriver::alias2parameters[alias];
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualPosition) =             para2float(p, parameters->minP, parameters->maxP, 16);
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualVelocity) =             para2float(v, parameters->minV, parameters->maxV, 12);
                               drivers[alias - 1].tx.next()->ActualTorque    = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
                               drivers[alias - 1].tx.next()->Undefined       = (data[7] - 50) / 2;
                               drivers[alias - 1].tx.next()->StatusWord      = err > 0 ? 0x0018 : CANDriver::alias2status[alias];
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
        switch(CANDriver::alias2status[alias]){
        case 0x0037:
            break;
        case 0x0031:
            memcpy(data, DamiaoEnable, 8);
            CANDriver::alias2status[alias] = 0x0037;
            return 8;
            break;
        }
        break;
    case 0:
        switch(CANDriver::alias2status[alias]){
        case 0x0037:
            memcpy(data, DamiaoDisable, 8);
            CANDriver::alias2status[alias] = 0x0031;
            return 8;
            break;
        case 0x0031:
            break;
        }
        break;
    case -1:
        memcpy(data, DamiaoClrErr, 8);
        CANDriver::alias2status[alias] = 0x0031;
        return 8;
        break;
    }
    DriverParameters const* parameters = CANDriver::alias2parameters[alias];
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

void damiaoTX(int const order, int id, unsigned char* const data, int const length, CANDriver* const can){
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
    int const alias = CANDriver::orderSlaveID2alias[order][id];
    DriverParameters const* parameters = CANDriver::alias2parameters[alias];
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualPosition) =             para2float(p, parameters->minP, parameters->maxP, 16);
    *reinterpret_cast<float*>(&drivers[alias - 1].tx.next()->ActualVelocity) =             para2float(v, parameters->minV, parameters->maxV, 12);
                               drivers[alias - 1].tx.next()->ActualTorque    = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
                               drivers[alias - 1].tx.next()->Undefined       = data[6];
                               drivers[alias - 1].tx.next()->StatusWord      = err > 1 ? 0x0018 : CANDriver::alias2status[alias];
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
int CAN::CANHAL;

CAN::CAN(){
    static bool initialized = false;
    if(!initialized){
        period = configXML->canAttribute("period");
        if(period < 1){
            period = 2000000L;
        }
        CANHAL = 0;
        initialized = true;
    }
    canhal = 0;
}

int CAN::ifaceIsUp(){
    if(canhal == 1){
        return 1;
    }
    if(device[0] == '/'){
        if(access(device, F_OK) == 0){
            return 1;
        }
        return 0;
    }
    int ret;
    struct ifreq ifr;
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
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
    if(ifaceIsUp() == 1){
        printf("iface %s is up\n", device);
        return 0;
    }
    if(device[0] == '/'){
        printf("iface %s is down\n", device);
        return -1;
    }
    printf("starting iface %s...\n", device);
    char cmd[256];
    int length = 0;
    length += snprintf(cmd + length, sizeof(cmd) - length, "sudo ip link set %s up type can bitrate %d", device, baudrate);
    if(canfd == 1){
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
    if(canhal == 1 || device[0] == '/'){
        return 0;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "sudo ifconfig %s down", device);
    system(cmd);
    sleep(1);
    return 0;
}

int CAN::open(int const masterID){
    int sock;
    if(device[0] == '/'){
        sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if(sock == -1){
            printf("creating socket failed\n");
            return -1;
        }else if(sock >= 128){
            printf("too many sockets\n");
            return -1;
        }
        struct sockaddr_un addr;
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, device, sizeof(addr.sun_path) - 1);
        if(connect(sock, (struct sockaddr const*)&addr, sizeof(addr)) == -1){
            printf("connecting to %s failed\n", device);
            close(sock);
            return -1;
        }
        return sock;
    }
    struct ifreq ifr;
    struct sockaddr_can addr;
    if(ifaceIsUp() != 1){
        printf("iface %s is down\n", device);
        return -1;
    }
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
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
        printf("binding to %s failed\n", device);
        close(sock);
        return -1;
    }
    if(masterID > 0){
        struct can_filter rfilter[1];
        rfilter[0].can_id = masterID;
        rfilter[0].can_mask = CAN_SFF_MASK;
        if(setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) != 0){
            printf("setsockopt CAN_RAW_FILTER failed\n");
            close(sock);
            return -1;
        }
    }
    if(canfd == 1){
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
        static unsigned int cnt = 0xffffffff;
        cnt++;
        if(cnt % 100 == 0){
            printf("send: cans[%d] write ret = %d\n", order, ret);
        }
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
    if(device[0] == '/' && canfd == 0){
        frame.flags &= ~CANFD_FDF;
    }else{
        frame.flags &= CANFD_FDF;
    }
    memcpy(frame.data, data, length);
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        static unsigned int cnt = 0xffffffff;
        cnt++;
        if(cnt % 100 == 0){
            printf("sendfd: cans[%d] write ret = %d\n", order, ret);
        }
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

CAN::~CAN(){
    if(CANHAL == 0){
        return;
    }
    if(CANHAL > 0){
        CANHAL = -CANHAL - 1;
    }
    CANHAL++;
    if(CANHAL == -1){
        canDeInit();
        CANHAL = 0;
    }
}

pthread_t CANDriver::rxPth, CANDriver::txPth, CANDriver::txPth_;
int CANDriver::rxCPU, CANDriver::txCPU, CANDriver::txCPU_;
std::map<std::string, DriverParameters*> CANDriver::type2parameters;
int* CANDriver::alias2masterID_;
unsigned short* CANDriver::alias2status;
DriverParameters** CANDriver::alias2parameters;
int CANDriver::orderSlaveID2alias[8][16];
canDriverRXFunction CANDriver::rxFuncs[2048][8];
canDriverTXFunction CANDriver::txFuncs[2048][8];

CANDriver::CANDriver(int const order, char const* device) : CAN(){
    static bool initialized = false;
    if(!initialized){
        rxPth = txPth = txPth_ = 0;
        rxCPU  = configXML->canAttribute("rx_cpu");
        txCPU  = configXML->canAttribute("tx_cpu");
        txCPU_ = configXML->canAttribute("tx_cpu_");
        adjustCPU(&rxCPU,  processorsCAN[0]);
        adjustCPU(&txCPU,  processorsCAN[1]);
        adjustCPU(&txCPU_, processorsCAN[2]);
        type2parameters.clear();
        alias2masterID_ = new int[dofAll + 1];
        alias2status = new unsigned short[dofAll + 1];
        alias2parameters = new DriverParameters*[dofAll + 1];
        int i = 0;
        while(i <= dofAll){
            alias2masterID_[i] = -1;
            alias2status[i] = 65535;
            alias2parameters[i] = nullptr;
            i++;
        }
        i = 0;
        while(i < 8){
            int j = 0;
            while(j < 16){
                orderSlaveID2alias[i][j] = -1;
                j++;
            }
            i++;
        }
        initialized = true;
    }
    rxSwap = nullptr;
    txSwap = nullptr;
    sock = -1;
    MASK = 0;
    mask = 0;
    rollingCounter = 0xff;
    this->order = order;
    alias2type = canAlias2type[order];
    alias2masterID = canAlias2masterID[order];
    alias2slaveID = canAlias2slaveID[order];
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        std::string const& type = itr->second;
        if(configXML->typeCategory("CAN", type.c_str()) != "driver"){
            printf("non-driver device %s on cans[%d] excluded\n", type.c_str(), order);
            itr = alias2type.erase(itr);
        }else{
            itr++;
        }
    }
    if(alias2type.size() == 0){
        return;
    }
    printf("cans[%d]\n", order);
    itr = alias2type.begin();
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
    canhal    = configXML->masterFeature("CAN", order, "canhal");
    baudrate  = configXML->masterAttribute("CAN", order, "baudrate");
    canfd     = configXML->masterFeature("CAN", order, "canfd");
    dbaudrate = configXML->masterAttribute("CAN", order, "dbaudrate");
    division  = configXML->masterAttribute("CAN", order, "division");
}

int CANDriver::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    if(ifaceUp() == -1){
        return -1;
    }
    if(canhal == 0){
        sock = open(0);
        if(sock == -1){
            return -1;
        }
    }
    rxSwap = new SwapList(dofAll * sizeof(DriverRxData));
    txSwap = new SwapList(dofAll * sizeof(DriverTxData));
    int i = 0;
    auto itr = alias2slaveID.begin();
    while(itr != alias2slaveID.end()){
        int alias = itr->first, slave = itr->second;
        std::string type = alias2type.find(alias)->second;
#ifndef NIIC
        if(drivers[alias - 1].init("CAN", 1, order, 0, slave, alias, type, i * sizeof(DriverRxData), i * sizeof(DriverTxData), nullptr, nullptr) != 0){
#else
        if(drivers[alias - 1].init("CAN", 1, order, 0, slave, alias, type, i * sizeof(DriverRxData), i * sizeof(DriverTxData), nullptr) != 0){
#endif
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

void CANDriver::cleanup(void* arg){
    int* epfd = (int*)arg;
    if(*epfd > -1){
        close(*epfd);
        *epfd = -1;
    }
}

void* CANDriver::rx(void* arg){
    std::vector<CANDriver>& cans = *(std::vector<CANDriver>*)arg;
    unsigned int count = 0xffffffff;
    unsigned char data[64];
    struct canframe frames[32];
    struct pack_info packInfo;
    packInfo.length = 32;
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
                if(cans[i].canhal == 0){
                    while(itr != cans[i].alias2slaveID.end()){
                        int alias = itr->first, slaveID = itr->second, length = rxFuncs[alias2masterID_[alias]][i](alias, data);
                        if(cans[i].device[0] == '/' || cans[i].canfd == 1){
                            cans[i].sendfd(slaveID, data, length);
                        }else{
                            cans[i].send(slaveID, data, length);
                        }
                        itr++;
                    }
                }else if(cans[i].canhal == 1){
                    packInfo.data_num = 0;
                    while(itr != cans[i].alias2slaveID.end()){
                        int alias = itr->first, slaveID = itr->second, nr = packInfo.data_num;
                        frames[nr].canid = BSWAP(slaveID);
                        frames[nr].count = cans[i].rollingCounter++;
                        frames[nr].can_type = cans[i].canfd;
                        frames[nr].can_channel = cans[i].device[3] - '0';
                        frames[nr].len = rxFuncs[alias2masterID_[alias]][i](alias, frames[nr].data);
                        packInfo.data_num++;
                        itr++;
                    }
                    int ret = canSendMsgFrame(cans[i].device, frames, &packInfo);
                    if(ret <= 0){
                        static unsigned int cnt = 0xffffffff;
                        cnt++;
                        if(cnt % 100 == 0){
                            printf("canSendMsgFrame: cans[%d] write ret = %d\n", i, ret);
                        }
                    }
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

void* CANDriver::tx(void* arg){
    std::vector<CANDriver>& cans = *(std::vector<CANDriver>*)arg;
    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if(epfd == -1){
        printf("epoll_create1() error\n");
        exit(-1);
    }
    pthread_cleanup_push(cleanup, &epfd);
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
                pthread_exit(nullptr);
            }
        }
        i++;
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
            int order = sock2order[events[i].data.fd], masterID = 0, length = 0;
            CANDriver& can = cans[order];
            if(can.device[0] == '/' || can.canfd == 1){
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

void CANDriver::cleanup_(void* arg){
    std::vector<pthread_t> pths = *(std::vector<pthread_t>*)arg;
    int i = 0;
    while(i < pths.size()){
        if(pths[i] > 0){
            pthread_cancel(pths[i]);
            pths[i] = 0;
        }
        i++;
    }
}

void* CANDriver::tx__(void* arg){
    CANDriver& can = *(CANDriver*)arg;
    struct canframe frames[32];
    struct pack_info packInfo;
    packInfo.length = 32;
    while(true){
        int ret = canRecvMsgFrame(can.device, frames, &packInfo);
        if(ret < 0){
            static unsigned int cnt = 0xffffffff;
            cnt++;
            if(cnt % 100 == 0){
                printf("canRecvMsgFrame: cans[%d] read ret = %d\n", can.order, ret);
            }
            continue;
        }
        int i = 0;
        while(i < packInfo.data_num){
            int masterID = frames[i].canid, length = frames[i].len;
            if(length == 0){
                i++;
                continue;
            }
            txFuncs[masterID][can.order](can.order, masterID, frames[i].data, length, &can);
            i++;
        }
    }
    return nullptr;
}

void* CANDriver::tx_(void* arg){
    std::vector<CANDriver>& cans = *(std::vector<CANDriver>*)arg;
    std::vector<pthread_t> pths;
    pthread_cleanup_push(cleanup_, &pths);
    int i = 0;
    while(i < cans.size()){
        if(cans[i].canhal != 1){
            i++;
            continue;
        }
        pthread_t pth;
        if(pthread_create(&pth, nullptr, &tx__, (void*)&cans[i]) != 0){
            printf("creating canhal tx__ thread failed\n");
            pthread_exit(nullptr);
        }
        pths.push_back(pth);
        i++;
    }
    i = 0;
    while(i < pths.size()){
        pthread_join(pths[i], nullptr);
        i++;
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

int CANDriver::run(std::vector<CANDriver>& cans){
    int i = 0, j = 0;
    while(i < cans.size()){
        if(cans[i].alias2type.size() > 0){
            if(cans[i].canhal == 1){
                CANHAL++;
            }
            j++;
        }
        i++;
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
    i = 1;
    while(i <= dofAll){
        printf("alias %2d, master_id %2d, status %d\n", i, alias2masterID_[i], alias2status[i]);
        i++;
    }
    i = 0;
    while(i < 8){
        printf("cans[%d]:\t", i);
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
    return 0;
}

CANDriver::~CANDriver(){
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
    if(alias2masterID_ != nullptr){
        delete[] alias2masterID_;
        alias2masterID_ = nullptr;
    }
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