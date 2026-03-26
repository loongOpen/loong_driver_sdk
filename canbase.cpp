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
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <sys/un.h>
#include <sstream>

namespace DriverSDK{
#define CANFD_BRS 0x01
#define CANFD_ESI 0x02
#define CANFD_FDF 0x04

extern ConfigXML* configXML;

unsigned short float2para(float const f, float const min, float const max, int const bit){
    return (f - min) * ((1 << bit) - 1.0) / (max - min);
}

float para2float(unsigned short const us, float const min, float const max, int const bit){
    return us * (max - min) / ((1 << bit) - 1.0) + min;
}

int nullRX(int const order, int const alias, int* const slaveID, unsigned char* const data){
    return 0;
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
    canhal      = configXML->masterFeature("CAN", order, "canhal");
    autoRestart = configXML->masterFeature("CAN", order, "auto_restart");
    baudrate    = configXML->masterAttribute("CAN", order, "baudrate");
    canfd       = configXML->masterFeature("CAN", order, "canfd");
    dbaudrate   = configXML->masterAttribute("CAN", order, "dbaudrate");
    division    = configXML->masterAttribute("CAN", order, "division");
    sock = -1;
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
    static unsigned int count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        count[order]++;
        if(count[order] % (2 * slaveCount) == 0){
            if(autoRestart){
                printf("send: cans[%d] write ret = %d\n", order, ret);
                ifaceDown();
                ifaceUp_();
            }
        }
        return -1;
    }
    count[order] = 0;
    return frame.can_id;
}

int CANBase::recv(unsigned char* const data, int const length, int* const masterID){
    int ret;
    struct can_frame frame;
    ret = read(sock, &frame, sizeof(frame));
    if(ret < 1){
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
    static unsigned int count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    ret = write(sock, &frame, sizeof(frame));
    if(ret != sizeof(frame)){
        count[order]++;
        if(count[order] % (2 * slaveCount) == 0){
            if(autoRestart){
                printf("sendfd: cans[%d] write ret = %d\n", order, ret);
                ifaceDown();
                ifaceUp_();
            }
        }
        return -1;
    }
    count[order] = 0;
    return frame.can_id;
}

int CANBase::recvfd(unsigned char* const data, int const length, int* const masterID){
    int ret;
    struct canfd_frame frame;
    ret = read(sock, &frame, sizeof(frame));
    if(ret < 1){
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
    if(device != nullptr){
        ifaceDown();
        free(device);
        device = nullptr;
    }
}
}