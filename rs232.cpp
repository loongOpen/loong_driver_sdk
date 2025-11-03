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

#include "rs232.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

namespace DriverSDK{
bool validXsens(unsigned char const* buff){
    if(!((buff[ 4] == 0x20 && buff[ 5] == 0x10 && buff[ 6] == 0x10) ||
         (buff[ 4] == 0x20 && buff[ 5] == 0x30 && buff[ 6] == 0x0c))||
        !(buff[19] == 0x40 && buff[20] == 0x20 && buff[21] == 0x0c) ||
        !(buff[34] == 0x80 && buff[35] == 0x20 && buff[36] == 0x0c)){
        return false;
    }
    int i = 1, sum = 0;
    while(i < 50 - 1){
        sum += buff[i];
        i++;
    }
    if(sum > 0xff){
       sum = ~sum;
       sum += 1;
    }
    sum &= 0xff;
    return sum == buff[50 - 1];
}

void crcUpdate(unsigned short* currentCRC, unsigned char const* src, int const length){
    unsigned int crc = *currentCRC;
    int i = 0, j;
    while(i < length){
        unsigned int byte = src[i];
        crc ^= byte << 8;
        j = 0;
        while(j < 8){
            unsigned int temp = crc << 1;
            if(crc & 0x8000){
                temp ^= 0x1021;
            }
            crc = temp;
            j++;
        }
        i++;
    }
    *currentCRC = crc;
}

bool validHiPNUC(unsigned char const* buff){
    unsigned short crc = 0;
    crcUpdate(&crc, buff, 4);
    crcUpdate(&crc, buff + 6, 76);
    return crc == (buff[5] << 8 | buff[4]);
}

float quadchar2float(unsigned char const* qc){
    float f = 0;
    unsigned char* c = (unsigned char*)&f;
    *(c + 0) = *(qc + 3);
    *(c + 1) = *(qc + 2);
    *(c + 2) = *(qc + 1);
    *(c + 3) = *(qc + 0);
    return f;
}

float quadchar2float_(unsigned char const* qc){
    return *(float*)qc;
}

float rpy0xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr +  7) * Pi / 180.0;
}

float rpy1xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 11) * Pi / 180.0;
}

float rpy2xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 15) * Pi / 180.0;
}

float gyr0xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 37);
}

float gyr1xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 41);
}

float gyr2xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 45);
}

float acc0xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 22);
}

float acc1xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 26);
}

float acc2xsens(SwapList const* txSwap){
    return quadchar2float(txSwap->nodePtr.load()->memPtr + 30);
}

float rpy0hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 58) * Pi / 180.0;
}

float rpy1hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 54) * Pi / 180.0;
}

float rpy2hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 62) * Pi / 180.0;
}

float gyr0hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 30) * Pi / 180.0;
}

float gyr1hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 34) * Pi / 180.0;
}

float gyr2hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 38) * Pi / 180.0;
}

float acc0hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 18) * 9.81;
}

float acc1hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 22) * 9.81;
}

float acc2hipnuc(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 26) * 9.81;
}

RS232::RS232(char const* device, int const baudrate, char const* type){
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    this->baudrate = baudrate;
    if(strcmp(type, "Xsens") == 0){
        frameLength = 50;
        header0 = 0xfa;
        header1 = 0xff;
        valid = validXsens;
        rpy0 = rpy0xsens;
        rpy1 = rpy1xsens;
        rpy2 = rpy2xsens;
        gyr0 = gyr0xsens;
        gyr1 = gyr1xsens;
        gyr2 = gyr2xsens;
        acc0 = acc0xsens;
        acc1 = acc1xsens;
        acc2 = acc2xsens;
    }else if(strcmp(type, "HiPNUC") == 0){
        frameLength = 82;
        header0 = 0x5a;
        header1 = 0xa5;
        valid = validHiPNUC;
        rpy0 = rpy0hipnuc;
        rpy1 = rpy1hipnuc;
        rpy2 = rpy2hipnuc;
        gyr0 = gyr0hipnuc;
        gyr1 = gyr1hipnuc;
        gyr2 = gyr2hipnuc;
        acc0 = acc0hipnuc;
        acc1 = acc1hipnuc;
        acc2 = acc2hipnuc;
    }else{
        printf("invalid type %s\n", type);
        exit(-1);
    }
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

void RS232::cleanup(void* arg){
    ChainNode* current = (ChainNode*)arg;
    while(current != nullptr){
        ChainNode* node = current;
        current = current->next;
        node->previous->next = nullptr;
        delete node;
    }
}

void* RS232::recv(void* arg){
    RS232* imu = (RS232*)arg;
    int    speedArray[] = {B921600, B576000, B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int baudrateArray[] = { 921600,  576000,  460800,  230400,  115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
    int i = 0, j;
    while(i < 13){
        if(baudrateArray[i] == imu->baudrate){
            break;
        }
        i++;
    }
    if(i == 13){
        printf("invalid baudrate %d\n", imu->baudrate);
        exit(-1);
    }
    int fd = open(imu->device, O_RDONLY | O_NOCTTY);
    if(fd < 0){
        printf("opening device %s failed\n", imu->device);
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
    printf("opened device %s baudrate %d\n", imu->device, imu->baudrate);
    unsigned char buff[2 * imu->frameLength], * buffA = buff, * buffB = buff + imu->frameLength;
    ChainNode* node0 = new ChainNode(), * node = node0;
    i = 0;
    while(i < 2 * imu->frameLength - 1){
        node->nr = i;
        node->next = new ChainNode();
        node->next->previous = node;
        node = node->next;
        i++;
    }
    node->nr = 2 * imu->frameLength - 1;
    node->next = node0;
    node0->previous = node;
    node = node0;
    pthread_cleanup_push(cleanup, node0);
    j = 0;
    do{
        j += read(fd, buffA + j, imu->frameLength - j);
    }while(j < imu->frameLength);
    struct timespec step{0, 1000};
    i = 2;
    while(true){
        if(node->nr == imu->frameLength - 1){
            j = 0;
            do{
                j += read(fd, buffB + j, imu->frameLength - j);
            }while(j < imu->frameLength);
        }else if(node->nr == 2 * imu->frameLength - 1){
            j = 0;
            do{
                j += read(fd, buffA + j, imu->frameLength - j);
            }while(j < imu->frameLength);
        }
        node = node->next;
        if(i < imu->frameLength){
            imu->txSwap->nodePtr.load()->next->memPtr[i] = buff[node->nr];
            i++;
        }
        if(buff[node->previous->nr] == imu->header0 && buff[node->nr] == imu->header1){
            if(imu->valid(imu->txSwap->nodePtr.load()->next->memPtr)){
                imu->txSwap->advanceNodePtr();
            }
            memset(imu->txSwap->nodePtr.load()->next->memPtr + 2, 0, imu->frameLength - 2);
            i = 2;
            nanosleep(&step, nullptr);
        }
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

int RS232::run(){
    if(strlen(device) == 0){
        return 1;
    }
    if(pthread_create(&pth, nullptr, &recv, this) != 0){
        printf("creating recv thread failed\n");
        return -1;
    }
    return 0;
}

RS232::~RS232(){
    if(pth > 0){
        pthread_cancel(pth);
        pth = 0;
    }
    if(txSwap != nullptr){
        delete txSwap;
        txSwap = nullptr;
    }
    if(device != nullptr){
        free(device);
        device = nullptr;
    }
}
}