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
#ifdef NIIC
#include <qiuniu/init.h>
#endif

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

unsigned int const table[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

unsigned int crc32(unsigned int crc, unsigned char const* buff, unsigned int const size){
    int i = 0;
    while(i < size){
        crc = table[(crc ^ buff[i]) & 0xff] ^ crc >> 8;
        i++;
    }
    return crc;
}

bool validForsense(unsigned char const* buff){
    return crc32(1, buff, 50) == *(unsigned int*)(buff + 50);
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

float rpy0forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 14) * Pi / 180.0;
}

float rpy1forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 10) * Pi / 180.0;
}

float rpy2forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 18) * Pi / 180.0;
}

float gyr0forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 34) * Pi / 180.0;
}

float gyr1forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 38) * Pi / 180.0;
}

float gyr2forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 42) * Pi / 180.0;
}

float acc0forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 22) * 9.81;
}

float acc1forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 26) * 9.81;
}

float acc2forsense(SwapList const* txSwap){
    return quadchar2float_(txSwap->nodePtr.load()->memPtr + 30) * 9.81;
}

RS232::RS232(char const* device, int const baudrate, char const* type){
    fd = -1;
    pth = 0;
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
    }else if(strcmp(type, "Forsense") == 0){
        frameLength = 54;
        header0 = 0xaa;
        header1 = 0x55;
        valid = validForsense;
        rpy0 = rpy0forsense;
        rpy1 = rpy1forsense;
        rpy2 = rpy2forsense;
        gyr0 = gyr0forsense;
        gyr1 = gyr1forsense;
        gyr2 = gyr2forsense;
        acc0 = acc0forsense;
        acc1 = acc1forsense;
        acc2 = acc2forsense;
    }else{
        printf("invalid imu type %s\n", type);
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
#ifndef NIIC
    imu->fd = open(imu->device, O_RDONLY | O_NOCTTY);
#else
    imu->fd = __RT(open(imu->device, O_RDONLY | O_NOCTTY));
#endif
    if(imu->fd < 0){
        printf("opening device %s failed\n", imu->device);
        exit(-1);
    }
    struct termios opt;
    tcgetattr(imu->fd, &opt);
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
    tcsetattr(imu->fd, TCSANOW, &opt);
    tcflush(imu->fd, TCIOFLUSH);
    printf("opened imu %s baudrate %d\n", imu->device, imu->baudrate);
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
#ifndef NIIC
        j += read(imu->fd, buffA + j, imu->frameLength - j);
#else
        j += __RT(read(imu->fd, buffA + j, imu->frameLength - j));
#endif
    }while(j < imu->frameLength);
    struct timespec step{0, 1000};
    i = 2;
    while(true){
        if(node->nr == imu->frameLength - 1){
            j = 0;
            do{
#ifndef NIIC
                j += read(imu->fd, buffB + j, imu->frameLength - j);
#else
                j += __RT(read(imu->fd, buffB + j, imu->frameLength - j));
#endif
            }while(j < imu->frameLength);
        }else if(node->nr == 2 * imu->frameLength - 1){
            j = 0;
            do{
#ifndef NIIC
                j += read(imu->fd, buffA + j, imu->frameLength - j);
#else
                j += __RT(read(imu->fd, buffA + j, imu->frameLength - j));
#endif
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
#ifndef NIIC
            nanosleep(&step, nullptr);
#else
            __RT(nanosleep(&step, nullptr));
#endif
        }
    }
    pthread_cleanup_pop(1);
    return nullptr;
}

int RS232::run(){
#ifdef NIIC
    static bool initialized = false;
    if(!initialized){
        qiuniu_init();
        initialized = true;
    }
#endif
    if(strlen(device) == 0){
        return 1;
    }
#ifndef NIIC
    if(pthread_create(&pth, nullptr, &recv, this) != 0){
#else
    if(__RT(pthread_create(&pth, nullptr, &recv, this)) != 0){
#endif
        printf("creating imu recv thread failed\n");
        return -1;
    }
    if(pthread_detach(pth) != 0){
        printf("detaching imu recv thread failed\n");
        return -1;
    }
    printf("imu recv\n");
    return 0;
}

RS232::~RS232(){
    if(pth > 0){
        pthread_cancel(pth);
        pth = 0;
    }
    if(fd > -1){
#ifndef NIIC
        close(fd);
#else
        __RT(close(fd));
#endif
        fd = -1;
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