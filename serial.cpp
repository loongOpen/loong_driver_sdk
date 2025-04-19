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

#include "serial.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

namespace DriverSDK{
Serial::Serial(char const* device, int const baudrate, int const frameLength, unsigned char const header0, unsigned char const header1){
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    this->baudrate = baudrate;
    this->frameLength = frameLength;
    this->header0 = header0;
    this->header1 = header1;
    txSwap = new SwapList(frameLength);
    int i = 0;
    while(i < 3){
        txSwap->nodePtr.load()->memPtr[0] = header0;
        txSwap->nodePtr.load()->memPtr[1] = header1;
        txSwap->advanceNodePtr();
        i++;
    }
    pth = 0;
}

void Serial::cleanup(void* arg){
    ChainNode* current = (ChainNode*)arg;
    while(current != nullptr){
        ChainNode* node = current;
        current = current->next;
        node->previous->next = nullptr;
        delete node;
    }
}

void* Serial::serialRead(void* arg){
    Serial* obj = (Serial*)arg;
    int    speedArray[] = {B921600, B576000, B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int baudrateArray[] = { 921600,  576000,  460800,  230400,  115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
    int i = 0, j;
    while(i < 13){
        if(baudrateArray[i] == obj->baudrate){
            break;
        }
        i++;
    }
    if(i == 13){
        printf("invalid baudrate %d\n", obj->baudrate);
        exit(-1);
    }
    int fd = open(obj->device, O_RDONLY | O_NOCTTY);
    if(fd < 0){
        printf("opening device %s failed\n", obj->device);
        exit(-1);
    }
    struct termios opt;
    tcgetattr(fd, &opt);
    cfsetispeed(&opt, speedArray[i]);
    cfsetospeed(&opt, speedArray[i]);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CRTSCTS;
    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_iflag &= ~INPCK;
    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(OCRNL | ONLCR);
    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 1;
    tcsetattr(fd, TCSANOW, &opt);
    tcflush(fd, TCIOFLUSH);
    printf("opened device %s baudrate %d\n", obj->device, obj->baudrate);
    char buff[2 * obj->frameLength], * buffA = buff, * buffB = buff + obj->frameLength;
    ChainNode* node0 = new ChainNode(), * node = node0;
    i = 0;
    while(i < 2 * obj->frameLength - 1){
        node->nr = i;
        node->next = new ChainNode();
        node->next->previous = node;
        node = node->next;
        i++;
    }
    node->nr = 2 * obj->frameLength - 1;
    node->next = node0;
    node0->previous = node;
    node = node0;
    pthread_cleanup_push(Serial::cleanup, node0);
    j = 0;
    do{
        j += read(fd, buffA + j, obj->frameLength - j);
    }while(j < obj->frameLength);
    struct timespec step{0, 1000};
    i = 2;
    while(true){
        if(node->nr == obj->frameLength - 1){
            j = 0;
            do{
                j += read(fd, buffB + j, obj->frameLength - j);
            }while(j < obj->frameLength);
        }else if(node->nr == 2 * obj->frameLength - 1){
            j = 0;
            do{
                j += read(fd, buffA + j, obj->frameLength - j);
            }while(j < obj->frameLength);
        }
        node = node->next;
        if(i < obj->frameLength){
            obj->txSwap->nodePtr.load()->next->memPtr[i] = buff[node->nr];
            i++;
        }
        if(buff[node->previous->nr] == obj->header0 && buff[node->nr] == obj->header1){
            if(obj->valid(obj->txSwap->nodePtr.load()->next->memPtr)){
                obj->txSwap->advanceNodePtr();
            }
            memset(obj->txSwap->nodePtr.load()->next->memPtr + 2, 0, obj->frameLength - 2);
            i = 2;
            nanosleep(&step, nullptr);
        }
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

int Serial::run(){
    if(strlen(device) == 0){
        return 1;
    }
    if(pthread_create(&pth, nullptr, &serialRead, this) != 0){
        printf("creating serialRead thread failed\n");
        return -1;
    }
    return 0;
}

Serial::~Serial(){
    if(pth > 0){
        pthread_cancel(pth);
    }
    if(txSwap != nullptr){
        delete txSwap;
    }
    free(device);
}

IMU::IMU(char const* device, int const baudrate, int const frameLength, unsigned char const header0, unsigned char const header1) : Serial(device, baudrate, frameLength, header0, header1){
}

float IMU::quadchar2float(unsigned char const* qc){
    float f = 0;
    unsigned char* c = (unsigned char*)&f;
    *(c + 0) = *(qc + 3);
    *(c + 1) = *(qc + 2);
    *(c + 2) = *(qc + 1);
    *(c + 3) = *(qc + 0);
    return f;
}

bool IMU::valid(unsigned char const* buff){
    if( buff[ 4] != 0x20 || buff[ 5] != 0x30 || buff[ 6] != 0x0c ||
        buff[19] != 0x40 || buff[20] != 0x20 || buff[21] != 0x0c ||
        buff[34] != 0x80 || buff[35] != 0x20 || buff[36] != 0x0c){
        return false;
    }
    int i = 1, sum = 0;
    while(i < frameLength - 1){
        sum += buff[i];
        i++;
    }
    if(sum > 0xff){
       sum = ~sum;
       sum += 1;
    }
    sum &= 0xff;
    return sum == buff[frameLength - 1];
}

IMU::~IMU(){
}
}