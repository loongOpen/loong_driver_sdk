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
#define CANFD_BRS 0x01
#define CANFD_ESI 0x02
#define CANFD_FDF 0x04
#define BSWAP(X) (((unsigned int)(X) & 0xff000000) >> 24 | ((unsigned int)(X) & 0x00ff0000) >> 8 | ((unsigned int)(X) & 0x0000ff00) << 8 | ((unsigned int)(X) & 0x000000ff) << 24)

extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> canAlias2type;
extern std::vector<std::map<int, std::vector<int>>> canAlias2masterIDs;
extern std::vector<std::map<int, int>> canAlias2slaveID;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern std::vector<unsigned short> processorsCAN;

unsigned short float2para(float const f, float const min, float const max, int const bit){
    return (f - min) * ((1 << bit) - 1.0) / (max - min);
}

float para2float(unsigned short const us, float const min, float const max, int const bit){
    return us * (max - min) / ((1 << bit) - 1.0) + min;
}

int nullRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    return 0;
}

void nullTX(int const order, int const masterID, unsigned char* const data, int const length, CAN* const can){
    printf("unexpected master_id %d on cans[%d]\n", masterID, order);
}

unsigned char const EncosEnable [3] = {0x71, 0x03, 0xe8};
unsigned char const EncosDisable[3] = {0x6d, 0x00, 0x00};
unsigned char const EncosDamp   [3] = {0x69, 0x00, 0x00};

int encosRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 2:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            memcpy(data, EncosDamp, 3);
            return 3;
            break;
        case 0x0231:
            break;
        }
        break;
    case 1:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            break;
        case 0x0231:
            memcpy(data, EncosEnable, 3);
            CAN::alias2status[alias] = 0x0237;
            return 3;
            break;
        }
        break;
    case 0:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            memcpy(data, EncosDisable, 3);
            CAN::alias2status[alias] = 0x0231;
            return 3;
            break;
        case 0x0231:
            break;
        }
        break;
    }
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    unsigned short p, v, t, kp, kd;
    if(CAN::alias2status[alias] != 0x0237){
         p = float2para(0.0, parameters->minP,  parameters->maxP,  16);
         v = float2para(0.0, parameters->minV,  parameters->maxV,  12);
        kp = float2para(0.0, parameters->minKp, parameters->maxKp, 12);
        kd = float2para(0.0, parameters->minKd, parameters->maxKd,  9);
         t = float2para(0.0, parameters->minT,  parameters->maxT,  12);
    }else{
         p = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetPosition, parameters->minP,  parameters->maxP,  16);
         v = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetVelocity, parameters->minV,  parameters->maxV,  12);
        kp = float2para(half2single(drivers[alias - 1].rx.previous()->ControlWord),   parameters->minKp, parameters->maxKp, 12);
        kd = float2para(half2single(drivers[alias - 1].rx.previous()->TargetTorque),  parameters->minKd, parameters->maxKd,  9);
         t = float2para(half2single(drivers[alias - 1].rx.previous()->TorqueOffset),  parameters->minT,  parameters->maxT,  12);
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

void encosTX(int const order, int const masterID, unsigned char* const data, int const length, CAN* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] & 0x1f;
    data[0] = data[2];
    unsigned short p = *(unsigned short*)(data + 0);
    data[2] = data[4];
    unsigned short v = *(unsigned short*)(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *(unsigned short*)(data + 3);
    int const slaveID = CAN::orderMasterID2slaveID[order][masterID], alias = CAN::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    signed char temperature = (data[6] - 50) / 2;
    bool error = err > 0 && (err != 1 && err != 2 && err != 4 || err == 1 && temperature > 99);
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition =             para2float(p, parameters->minP, parameters->maxP, 16);
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity =             para2float(v, parameters->minV, parameters->maxV, 12);
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
              drivers[alias - 1].tx.next()->Undefined      = temperature;
              drivers[alias - 1].tx.next()->StatusWord     = error ? 0x0218 : CAN::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = error ? err : 0;
    can->mask |= 1 << slaveID;
    if(can->mask == can->MASK){
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        if(CAN::alias2status[alias] == 0x0000){
            int i = 0;
            while(i < 16){
                int a = CAN::orderSlaveID2alias[order][i];
                if(a > 0){
                    CAN::alias2status[a] = 0x0231;
                }
                i++;
            }
        }
    }
}

unsigned char const DamiaoEnable [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
unsigned char const DamiaoDisable[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
unsigned char const DamiaoClrErr [8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

int damiaoRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 1:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            break;
        case 0x0231:
            memcpy(data, DamiaoEnable, 8);
            CAN::alias2status[alias] = 0x0237;
            return 8;
            break;
        }
        break;
    case 0:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            memcpy(data, DamiaoDisable, 8);
            CAN::alias2status[alias] = 0x0231;
            return 8;
            break;
        case 0x0231:
            break;
        }
        break;
    case -1:
        switch(CAN::alias2status[alias]){
        case 0x0237:
            break;
        case 0x0231:
            memcpy(data, DamiaoClrErr, 8);
            return 8;
            break;
        }
        break;
    }
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    unsigned short  p = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetPosition, parameters->minP,  parameters->maxP,  16);
    unsigned short  v = float2para(  *(float*)&drivers[alias - 1].rx.previous()->TargetVelocity, parameters->minV,  parameters->maxV,  12);
    unsigned short kp = float2para(half2single(drivers[alias - 1].rx.previous()->ControlWord),   parameters->minKp, parameters->maxKp, 12);
    unsigned short kd = float2para(half2single(drivers[alias - 1].rx.previous()->TargetTorque),  parameters->minKd, parameters->maxKd, 12);
    unsigned short  t = float2para(half2single(drivers[alias - 1].rx.previous()->TorqueOffset),  parameters->minT,  parameters->maxT,  12);
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

void damiaoTX(int const order, int const masterID, unsigned char* const data, int const length, CAN* const can){
    if(length != 8){
        return;
    }
    unsigned char err = data[0] >> 4;
    // int const slaveID = data[0] & 0x0f;
    data[0] = data[2];
    unsigned short p = *(unsigned short*)(data + 0);
    data[2] = data[4];
    unsigned short v = *(unsigned short*)(data + 2);
    v >>= 4;
    data[3] = data[5];
    data[4] = data[4] & 0x0f;
    unsigned short t = *(unsigned short*)(data + 3);
    int const slaveID = CAN::orderMasterID2slaveID[order][masterID], alias = CAN::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition =             para2float(p, parameters->minP, parameters->maxP, 16);
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity =             para2float(v, parameters->minV, parameters->maxV, 12);
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(para2float(t, parameters->minT, parameters->maxT, 12));
              drivers[alias - 1].tx.next()->Undefined      = data[7];
              drivers[alias - 1].tx.next()->StatusWord     = err > 1 ? 0x0218 : CAN::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = err > 1 ? err : 0x0000;
    can->mask |= 1 << slaveID;
    if(can->mask == can->MASK){
        can->txSwap->advanceNodePtr();
        can->mask = 0;
        if(CAN::alias2status[alias] == 0x0000){
            int i = 0;
            while(i < 16){
                int a = CAN::orderSlaveID2alias[order][i];
                if(a > 0){
                    CAN::alias2status[a] = 0x0231;
                }
                i++;
            }
        }
    }
}

unsigned char const RealManIAP    [8] = {0x02, 0x49, 0x00};
unsigned char const RealManEnable [8] = {0x02, 0x0a, 0x01};
unsigned char const RealManDisable[8] = {0x02, 0x0a, 0x00};
unsigned char const RealManClrErr [8] = {0x02, 0x0f, 0x01};

int realManRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    switch(drivers[alias - 1].rx.previous()->Undefined){
    case 1:
        switch(CAN::alias2status[alias]){
        case 0x0250:
            CAN::alias2status[alias] = 0x0237;
        case 0x0237:
            break;
        case 0x0231:
            memcpy(data, RealManEnable, 3);
            return 3;
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    case 0:
        switch(CAN::alias2status[alias]){
        case 0x0250:
        case 0x0237:
            memcpy(data, RealManDisable, 3);
            return 3;
            break;
        case 0x0231:
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    case -1:
        switch(CAN::alias2status[alias]){
        case 0x0250:
            memcpy(data, RealManDisable, 3);
            return 3;
            break;
        case 0x0237:
            break;
        case 0x0231:
            memcpy(data, RealManClrErr, 3);
            return 3;
            break;
        case 0x0050:
            *slaveID += 0x0600;
            return 0;
            break;
        case 0x0000:
            memcpy(data, RealManIAP, 3);
            return 3;
            break;
        }
        break;
    }
    unsigned char* const data_ = data + 64;
    int length = std::numeric_limits<int>::min(), index = (*slaveID - 1) * 8;
    static bool enabled[8];
    if(*slaveID == 1){
        enabled[order] = true;
    }
    if(CAN::alias2status[alias] != 0x0237){
        enabled[order] = false;
    }
    if(*slaveID == 1){
        memset(data_, 0x00, 56);
        memset(data_ + 56, 0xff, 7);
        data_[63] = 0xaf;
    }else if(*slaveID == 7){
        if(enabled[order]){
            *slaveID = 0x02f;
            length = -64;
        }else{
            *slaveID = 0x07f;
            memset(data_, 0x00, 23);
            data_[23] = 0x0f;
            return -24;
        }
    }
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    *(  int*)(data_ + index + 0) =   *(float*)&drivers[alias - 1].rx.previous()->TargetPosition * 180.0 / Pi / parameters->pUnit;
    *(short*)(data_ + index + 4) =   *(float*)&drivers[alias - 1].rx.previous()->VelocityOffset * 30.0 / Pi / parameters->vOffsetUnit;
    *(short*)(data_ + index + 6) = half2single(drivers[alias - 1].rx.previous()->TorqueOffset) / parameters->tConstant / parameters->cOffsetUnit;
    return length;
}

void realManTX(int const order, int const masterID, unsigned char* const data, int const length, CAN* const can){
    int const slaveID = CAN::orderMasterID2slaveID[order][masterID], alias = CAN::orderSlaveID2alias[order][slaveID];
    DriverParameters const* parameters = CAN::alias2parameters[alias];
    if(masterID > 0x700){
        if(length != 16){
            return;
        }
        switch(CAN::alias2status[alias]){
        case 0x0050:
            unsigned short err = *(unsigned short*)(data + 0);
            *(float*)&drivers[alias - 1].tx->ActualPosition = *(int*)(data + 8) * parameters->pUnit * Pi / 180.0;
            *(float*)&drivers[alias - 1].tx->ActualVelocity = 0.0;
                      drivers[alias - 1].tx->ActualTorque   = single2half(*(int*)(data + 12) * parameters->actualCUnit * parameters->tConstant);
                      drivers[alias - 1].tx->Undefined      = *(short*)(data + 4) * 0.1;
                      drivers[alias - 1].tx->StatusWord     = err > 0 ? 0x0218 : 0x0250;
                      drivers[alias - 1].tx->ErrorCode      = err;
            CAN::alias2status[alias] = 0x0250;
            break;
        }
        return;
    }else if(masterID > 0x100 && masterID != 0x5fe){
        if(length != 3){
            return;
        }
        switch(CAN::alias2status[alias]){
        case 0x0250:
        case 0x0237:
            if(data[1] == 0x0a){
                CAN::alias2status[alias] = 0x0231;
            }
            break;
        case 0x0231:
            if(data[1] == 0x0a){
                CAN::alias2status[alias] = 0x0237;
            }
            break;
        case 0x0000:
            if(data[1] == 0x49 && data[2] == 0x01){
                CAN::alias2status[alias] = 0x0050;
            }
            break;
        }
        return;
    }else if(masterID > 0x081){
        if(length != 24){
            return;
        }
    }
    unsigned short err = *(unsigned short*)(data + 12);
    *(float*)&drivers[alias - 1].tx.next()->ActualPosition = *(int*)(data + 8) * parameters->pUnit * Pi / 180.0;
    *(float*)&drivers[alias - 1].tx.next()->ActualVelocity = *(int*)(data + 4) * parameters->actualVUnit * Pi / 30.0;
              drivers[alias - 1].tx.next()->ActualTorque   = single2half(*(int*)(data + 0) * parameters->actualCUnit * parameters->tConstant);
              drivers[alias - 1].tx.next()->Undefined      = *(short*)(data + 16) * 0.1;
              drivers[alias - 1].tx.next()->StatusWord     = err > 0 ? 0x0218 : CAN::alias2status[alias];
              drivers[alias - 1].tx.next()->ErrorCode      = err;
    can->mask |= 1 << slaveID;
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
    pUnit       = configXML->readDeviceParameter("CAN", type.c_str(),       "PUnit");
    targetVUnit = configXML->readDeviceParameter("CAN", type.c_str(), "TargetVUnit");
    actualVUnit = configXML->readDeviceParameter("CAN", type.c_str(), "ActualVUnit");
    vOffsetUnit = configXML->readDeviceParameter("CAN", type.c_str(), "VOffsetUnit");
    targetCUnit = configXML->readDeviceParameter("CAN", type.c_str(), "TargetCUnit");
    actualCUnit = configXML->readDeviceParameter("CAN", type.c_str(), "ActualCUnit");
    cOffsetUnit = configXML->readDeviceParameter("CAN", type.c_str(), "COffsetUnit");
    tConstant   = configXML->readDeviceParameter("CAN", type.c_str(),   "TConstant");
    if( minP  == std::numeric_limits<float>::min() ||
        maxP  == std::numeric_limits<float>::min() ||
        minV  == std::numeric_limits<float>::min() ||
        maxV  == std::numeric_limits<float>::min() ||
        minKp == std::numeric_limits<float>::min() ||
        maxKp == std::numeric_limits<float>::min() ||
        minKd == std::numeric_limits<float>::min() ||
        maxKd == std::numeric_limits<float>::min() ||
        minT  == std::numeric_limits<float>::min() ||
        maxT  == std::numeric_limits<float>::min() ||
        pUnit       == std::numeric_limits<float>::min() ||
        targetVUnit == std::numeric_limits<float>::min() ||
        actualVUnit == std::numeric_limits<float>::min() ||
        vOffsetUnit == std::numeric_limits<float>::min() ||
        targetCUnit == std::numeric_limits<float>::min() ||
        actualCUnit == std::numeric_limits<float>::min() ||
        cOffsetUnit == std::numeric_limits<float>::min() ||
        tConstant   == std::numeric_limits<float>::min()){
        printf("no parameter of driver with type %s is set in xml\n", type.c_str());
        return -1;
    }
    return 0;
}

DriverParameters::~DriverParameters(){
}

long CANBase::period;
int CANBase::CANHAL;

CANBase::CANBase(int const order, char const* device){
    static bool initialized = false;
    if(!initialized){
        period = configXML->canAttribute("period");
        if(period < 1000000L){
            period = 1000000L;
        }
        CANHAL = 0;
        initialized = true;
    }
    this->order = order;
    canhal    = configXML->masterFeature("CAN", order, "canhal");
    baudrate  = configXML->masterAttribute("CAN", order, "baudrate");
    canfd     = configXML->masterFeature("CAN", order, "canfd");
    dbaudrate = configXML->masterAttribute("CAN", order, "dbaudrate");
    division  = configXML->masterAttribute("CAN", order, "division");
    slaveCount = 0;
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
}

int CANBase::ifaceIsUp(){
    if(canhal == 1){
        return 1;
    }
    if(device[0] == '/'){
        if(access(device, F_OK) == 0){
            return 1;
        }
        return 0;
    }
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW), ret;
    if(sock == -1){
        printf("creating socket failed\n");
        return -1;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, device);
    if(ioctl(sock, SIOCGIFFLAGS, &ifr) == -1){
        printf("ioctl() failed\n");
        close(sock);
        return -1;
    }
    if(ifr.ifr_ifru.ifru_flags & IFF_RUNNING){
        ret = 1;
    }else{
        ret = 0;
    }
    close(sock);
    return ret;
}

int CANBase::ifaceUp(){
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
    int length = snprintf(cmd, sizeof(cmd), "ip link set %s txqueuelen 100 up type can restart-ms 10 bitrate %d", device, baudrate);
    if(canfd == 1){
        length += snprintf(cmd + length, sizeof(cmd) - length, " fd on dbitrate %d", dbaudrate);
    }
    if(system(cmd) == -1){
        printf("system() failed\n");
        return -1;
    }
    usleep(500000);
    printf("iface %s is started\n", device);
    return 0;
}

int CANBase::ifaceUp_(){
    if(ifaceIsUp() == 1){
        printf("iface %s is up\n", device);
        return 0;
    }
    if(device[0] == '/'){
        printf("iface %s is down\n", device);
        return -1;
    }
    printf("starting iface %s...\n", device);
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
        printf("creating socket failed\n");
        return -1;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, device);
    if(ioctl(sock, SIOCGIFFLAGS, &ifr) == -1){
        printf("ioctl() failed\n");
        close(sock);
        return -1;
    }
    strcpy(ifr.ifr_name, device);
    ifr.ifr_flags |= IFF_UP;
    if(ioctl(sock, SIOCSIFFLAGS, &ifr) == -1){
        printf("ioctl() failed\n");
        close(sock);
        return -1;
    }
    close(sock);
    printf("iface %s is started\n", device);
    return 0;
}

int CANBase::ifaceDown(){
    if(canhal == 1 || device[0] == '/'){
        return 0;
    }
    printf("stopping iface %s...\n", device);
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
        printf("creating socket failed\n");
        return -1;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, device);
    if(ioctl(sock, SIOCGIFFLAGS, &ifr) == -1){
        printf("ioctl() failed\n");
        close(sock);
        return -1;
    }
    strcpy(ifr.ifr_name, device);
    ifr.ifr_flags &= ~IFF_UP;
    if(ioctl(sock, SIOCSIFFLAGS, &ifr) == -1){
        printf("ioctl() failed\n");
        close(sock);
        return -1;
    }
    close(sock);
    printf("iface %s is stopped\n", device);
    return 0;
}

int CANBase::open(int const masterID){
    int sock;
    if(device[0] == '/'){   // iMotion Unix Domain Sockets CAN
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
    if(ifaceIsUp() != 1){
        printf("iface %s is not ready\n", device);
        return -2;
    }
    printf("opening iface %s\n", device);
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(sock == -1){
        printf("creating socket failed\n");
        return -1;
    }else if(sock >= 128){
        printf("too many sockets\n");
        return -1;
    }
    struct ifreq ifr;
    strcpy(ifr.ifr_name, device);
    ioctl(sock, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
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
    printf("iface %s is opened\n", device);
    return sock;
}

int CANBase::send(int const slaveID, unsigned char const* data, int length){
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
        if(cnt % slaveCount == 0){
            printf("send: cans[%d] write ret = %d\n", order, ret);
            ifaceDown();
            ifaceUp_();
        }
        return -1;
    }
    return frame.can_id;
}

int CANBase::recv(unsigned char* const data, int const length, int* const masterID){
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

int CANBase::sendfd(int const slaveID, unsigned char const* data, int const length){
    int ret;
    struct canfd_frame frame;
    if(length > sizeof(frame.data)){
        printf("sendfd: cans[%d] send data length cannot be greater than %ld\n", order, sizeof(frame.data));
        return -1;
    }
    frame.can_id = slaveID;
    frame.len = length;
    if(canfd == 0){
        frame.flags &= ~CANFD_FDF;
    }else{
        frame.flags |= CANFD_FDF;
        if(baudrate == dbaudrate){
            frame.flags &= ~CANFD_BRS;
        }else{
            frame.flags |= CANFD_BRS;
        }
    }
    memcpy(frame.data, data, length);
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        static unsigned int cnt = 0xffffffff;
        cnt++;
        if(cnt % slaveCount == 0){
            printf("sendfd: cans[%d] write ret = %d\n", order, ret);
            ifaceDown();
            ifaceUp_();
        }
        return -1;
    }
    return frame.can_id;
}

int CANBase::recvfd(unsigned char* const data, int const length, int* const masterID){
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

CANBase::~CANBase(){
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

pthread_t CAN::rxPth, CAN::txPth, CAN::txPth_;
int CAN::rxCPU, CAN::txCPU, CAN::txCPU_;
std::map<std::string, DriverParameters*> CAN::type2parameters;
unsigned short* CAN::alias2status;
DriverParameters** CAN::alias2parameters;
int CAN::orderSlaveID2alias[8][16];
int CAN::orderMasterID2slaveID[8][2048];
canRXFunction CAN::rxFuncs[8][16];
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
        type2parameters.clear();
        alias2status = new unsigned short[dofAll + 1];
        alias2parameters = new DriverParameters*[dofAll + 1];
        int i = 0, j;
        while(i <= dofAll){
            alias2status[i] = 65535;
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
            while(j < 16){
                orderSlaveID2alias[i][j] = 0;
                j++;
            }
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 16){
                rxFuncs[i][j] = nullRX;
                j++;
            }
            i++;
        }
        i = 0;
        while(i < 8){
            j = 0;
            while(j < 2048){
                txFuncs[i][j] = nullTX;
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
    alias2type = canAlias2type[order];
    alias2masterIDs = canAlias2masterIDs[order];
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
        int alias = itr->first, slaveID = alias2slaveID.find(alias)->second;
        std::vector<int> masterIDs = alias2masterIDs.find(alias)->second;
        std::string const& type = itr->second;
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
        printf("\talias %d, type %s, master_ids", alias, type.c_str());
        int i = 0;
        while(i < masterIDs.size()){
            printf(" %d", masterIDs[i]);
            orderMasterID2slaveID[order][masterIDs[i]] = slaveID;
            if(txFuncs[order][masterIDs[i]] != nullTX){
                printf("\ninvalid can bus configuration\n");
                exit(-1);
            }
            if(type.starts_with("Encos")){
                txFuncs[order][masterIDs[i]] = encosTX;
            }else if(type.starts_with("Damiao")){
                txFuncs[order][masterIDs[i]] = damiaoTX;
            }else if(type.starts_with("RealMan")){
                txFuncs[order][masterIDs[i]] = realManTX;
            }
            i++;
        }
        printf(", slave_id %d\n", slaveID);
        orderSlaveID2alias[order][slaveID] = alias;
        if(rxFuncs[order][slaveID] != nullRX){
            printf("invalid can bus configuration\n");
            exit(-1);
        }
        if(type.starts_with("Encos")){
            rxFuncs[order][slaveID] = encosRX;
        }else if(type.starts_with("Damiao")){
            rxFuncs[order][slaveID] = damiaoRX;
        }else if(type.starts_with("RealMan")){
            rxFuncs[order][slaveID] = realManRX;
        }
        itr++;
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
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        count++;
        int i = 0;
        while(i < cans.size()){
            if(cans[i].rxSwap != nullptr && count % cans[i].division == 0){
                if(cans[i].canhal == 1){
                    packInfo.data_num = 0;
                }
                auto itr = cans[i].alias2slaveID.begin();
                while(itr != cans[i].alias2slaveID.end()){
                    int alias = itr->first, slaveID = itr->second, length = rxFuncs[i][slaveID](i, alias, &slaveID, data);
                    if(length >= 0){
                        data_ = data;
                    }else if(length == std::numeric_limits<int>::min()){
                        itr++;
                        continue;
                    }else{
                        data_ = data + 64;
                        length = -length;
                    }
                    if(cans[i].canhal == 0){
                        if(cans[i].canfd == 1 || cans[i].device[0] == '/'){
                            cans[i].sendfd(slaveID, data_, length);
                        }else{
                            cans[i].send(slaveID, data_, length);
                        }
                    }else{
                        int nr = packInfo.data_num;
                        frames[nr].canid = BSWAP(slaveID);
                        frames[nr].count = cans[i].rollingCounter++;
                        frames[nr].can_type = cans[i].canfd == 1 ? 2 : 0;
                        frames[nr].can_channel = cans[i].device[3] - '0';
                        frames[nr].len = length;
                        memcpy(frames[nr].data, data_, length);
                        packInfo.data_num++;
                    }
                    itr++;
                }
                if(cans[i].canhal == 1){
                    int ret = canSendMsgFrame(cans[i].device, frames, &packInfo);
                    if(ret <= 0){
                        printf("canSendMsgFrame: cans[%d] write ret = %d\n", i, ret);
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
            int order = sock2order[events[i].data.fd], masterID, length;
            CAN& can = cans[order];
            if(can.canfd == 1 || can.device[0] == '/'){
                length = can.recvfd(data, 64, &masterID);
            }else{
                length = can.recv(data, 64, &masterID);
            }
            if(length <= 0){
                i++;
                continue;
            }
            txFuncs[order][masterID](order, masterID, data, length, &cans[order]);
            i++;
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
        i++;
    }
}

void* CAN::tx__(void* arg){
    CAN& can = *(CAN*)arg;
    struct canframe frames[32];
    struct pack_info packInfo;
    packInfo.length = 32;
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
                i++;
                continue;
            }
            txFuncs[can.order][masterID](can.order, masterID, frames[i].data, length, &can);
            i++;
        }
    }
    return nullptr;
}

void* CAN::tx_(void* arg){
    std::vector<CAN>& cans = *(std::vector<CAN>*)arg;
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

int CAN::run(std::vector<CAN>& cans){
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
    i = 0;
    while(i < cans.size()){
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

CAN::~CAN(){
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