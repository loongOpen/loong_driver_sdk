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
#include "rs232.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#ifdef NIIC
#include <qiuniu/init.h>
#endif

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> rs232alias2type;
extern int imuCount;
extern WrapperPair<IMURXData, IMUTXData, IMUParameters>* imus;

bool validXsens(unsigned char const* buff){
    if(!((buff[ 4] == 0x20 && buff[ 5] == 0x10 && buff[ 6] == 0x10) ||
         (buff[ 4] == 0x20 && buff[ 5] == 0x30 && buff[ 6] == 0x0c))||
        !(buff[19] == 0x40 && buff[20] == 0x20 && buff[21] == 0x0c) ||
        !(buff[34] == 0x80 && buff[35] == 0x20 && buff[36] == 0x0c)){
        return false;
    }
    int i = 1, sum = 0;
    while(i < 49){
        sum += buff[i];
        ++i;
    }
    if(sum > 0xff){
       sum = ~sum;
       sum += 1;
    }
    sum &= 0xff;
    return sum == buff[49];
}

void crcUpdate(unsigned short* currentCRC, unsigned char const* buff, int const length){
    unsigned int crc = *currentCRC;
    int i = 0, j;
    while(i < length){
        crc ^= (unsigned int)buff[i] << 8;
        j = 0;
        while(j < 8){
            unsigned int temp = crc << 1;
            if((crc & 0x8000) != 0){
                temp ^= 0x1021;
            }
            crc = temp;
            ++j;
        }
        ++i;
    }
    *currentCRC = crc;
}

bool validHipnuc(unsigned char const* buff){
    unsigned short crc = 0;
    crcUpdate(&crc, buff, 4);
    crcUpdate(&crc, buff + 6, 76);
    return crc == (buff[5] << 8 | buff[4]);
}

bool validForsense(unsigned char const* buff){
    return crc32(1, buff, 50) == *(unsigned int*)(buff + 50);
}

unsigned short checksum(unsigned char const* buff, int length){
    unsigned char a = 0, b = 0;
	int i = 0;
	while(i < length){
		a += buff[i];
		b += a;
		++i;
	}
	return (unsigned short)b << 8 | a;
}

bool validYesense(unsigned char const* buff){
    return checksum(buff + 2, 63) == *(unsigned short*)(buff + 65);
}

bool validYesense_(unsigned char const* buff){
    return checksum(buff + 2, 45) == *(unsigned short*)(buff + 47);
}

bool validLinstech(unsigned char const* buff){
    int i = 2, sum = 0;
    while(i < 40){
        sum += buff[i];
        ++i;
    }
    sum = ~sum;
    sum &= 0xff;
    return sum == buff[40];
}

void parseXsens(SwapList const* txSwap_, int const index){
    imus[index].tx.next()->rpy[0] = quadchar2float(txSwap_->nodePtr.load()->memPtr +  7) * Pi / 180.0;
    imus[index].tx.next()->rpy[1] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 11) * Pi / 180.0;
    imus[index].tx.next()->rpy[2] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 15) * Pi / 180.0;
    imus[index].tx.next()->gyr[0] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 37);
    imus[index].tx.next()->gyr[1] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 41);
    imus[index].tx.next()->gyr[2] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 45);
    imus[index].tx.next()->acc[0] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 22);
    imus[index].tx.next()->acc[1] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 26);
    imus[index].tx.next()->acc[2] = quadchar2float(txSwap_->nodePtr.load()->memPtr + 30);
}

void parseHipnuc(SwapList const* txSwap_, int const index){
    imus[index].tx.next()->rpy[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 58) * Pi / 180.0;
    imus[index].tx.next()->rpy[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 54) * Pi / 180.0;
    imus[index].tx.next()->rpy[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 62) * Pi / 180.0;
    imus[index].tx.next()->gyr[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 30) * Pi / 180.0;
    imus[index].tx.next()->gyr[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 34) * Pi / 180.0;
    imus[index].tx.next()->gyr[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 38) * Pi / 180.0;
    imus[index].tx.next()->acc[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 18) * 9.81;
    imus[index].tx.next()->acc[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 22) * 9.81;
    imus[index].tx.next()->acc[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 26) * 9.81;
}

void parseForsense(SwapList const* txSwap_, int const index){
    imus[index].tx.next()->rpy[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 14) * Pi / 180.0;
    imus[index].tx.next()->rpy[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 10) * Pi / 180.0;
    imus[index].tx.next()->rpy[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 18) * Pi / 180.0;
    imus[index].tx.next()->gyr[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 34) * Pi / 180.0;
    imus[index].tx.next()->gyr[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 38) * Pi / 180.0;
    imus[index].tx.next()->gyr[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 42) * Pi / 180.0;
    imus[index].tx.next()->acc[0] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 22) * 9.81;
    imus[index].tx.next()->acc[1] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 26) * 9.81;
    imus[index].tx.next()->acc[2] = quadchar2float_(txSwap_->nodePtr.load()->memPtr + 30) * 9.81;
}

void parseYesense(SwapList const* txSwap_, int const index){
    imus[index].tx.next()->rpy[0] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 39) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->rpy[1] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 35) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->rpy[2] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 43) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[0] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 21) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[1] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 25) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[2] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 29) / 1000000.0 * Pi / 180.0;
    imus[index].tx.next()->acc[0] = quadchar2int_(txSwap_->nodePtr.load()->memPtr +  7) / 1000000.0;
    imus[index].tx.next()->acc[1] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 11) / 1000000.0;
    imus[index].tx.next()->acc[2] = quadchar2int_(txSwap_->nodePtr.load()->memPtr + 15) / 1000000.0;
}

void parseLinstech(SwapList const* txSwap_, int const index){
    imus[index].tx.next()->rpy[0] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 26) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->rpy[1] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 30) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->rpy[2] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 34) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[0] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 14) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[1] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 18) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->gyr[2] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 22) / 10000.0 * Pi / 180.0;
    imus[index].tx.next()->acc[0] = quadchar2int(txSwap_->nodePtr.load()->memPtr +  2) / 10000.0 * 9.81;
    imus[index].tx.next()->acc[1] = quadchar2int(txSwap_->nodePtr.load()->memPtr +  6) / 10000.0 * 9.81;
    imus[index].tx.next()->acc[2] = quadchar2int(txSwap_->nodePtr.load()->memPtr + 10) / 10000.0 * 9.81;
}

RS232::RS232(int const order, char const* device){
    rxSwap = txSwap = txSwap_ = nullptr;
    fd = -1;
    pth = 0;
    this->order = order;
    alias2type = rs232alias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("rs232s[%d]\n", order);
    auto itr = alias2type.begin();
    std::string const& type = itr->second;
    printf("\talias %d, type %s\n", itr->first, type.c_str());
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    baudrate = configXML->masterAttribute("RS232", order, "baudrate");
    this->type = (char*)malloc(strlen(type.c_str()) + 1);
    strcpy(this->type, type.c_str());
}

RS232::RS232(char const* device, int const baudrate, char const* type){
    rxSwap = txSwap = txSwap_ = nullptr;
    fd = -1;
    pth = 0;
    order = 0;
    alias2type.insert(std::make_pair(240, std::string(type)));
    printf("rs232s[0]\n\talias 240, type %s\n", type);
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    this->baudrate = baudrate;
    this->type = (char*)malloc(strlen(type) + 1);
    strcpy(this->type, type);
}

int RS232::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    if(strlen(device) == 0){
        return 1;
    }
    if(strcmp(type, "Xsens") == 0){
        frameLength = 50;
        header0 = 0xfa;
        header1 = 0xff;
        valid = validXsens;
        parse = parseXsens;
    }else if(strcmp(type, "HiPNUC") == 0){
        frameLength = 82;
        header0 = 0x5a;
        header1 = 0xa5;
        valid = validHipnuc;
        parse = parseHipnuc;
    }else if(strcmp(type, "Forsense") == 0){
        frameLength = 54;
        header0 = 0xaa;
        header1 = 0x55;
        valid = validForsense;
        parse = parseForsense;
    }else if(strcmp(type, "YESENSE") == 0){
        frameLength = 67;
        header0 = 0x59;
        header1 = 0x53;
        valid = validYesense;
        parse = parseYesense;
    }else if(strcmp(type, "YESENSE_") == 0){
        frameLength = 49;
        header0 = 0x59;
        header1 = 0x53;
        valid = validYesense_;
        parse = parseYesense;
    }else if(strcmp(type, "Lins-Tech") == 0){
        frameLength = 41;
        header0 = 0x7f;
        header1 = 0x94;
        valid = validLinstech;
        parse = parseLinstech;
    }else{
        printf("\tinvalid imu type %s\n", type);
        return -1;
    }
    rxSwap = new SwapList(imuCount * sizeof(IMURXData));
    txSwap = new SwapList(imuCount * sizeof(IMUTXData));
    auto itr = alias2type.begin();
    int index = itr->first - 240;
    if(index < 0 || index > 15){
        printf("\tinvalid imu alias %d\n", itr->first);
        return -1;
    }
#ifndef NIIC
    if(imus[index].init("RS232", 5, order, 0, 0, itr->first, itr->second, index * sizeof(IMURXData), index * sizeof(IMUTXData), nullptr, nullptr) != 0){
#else
    if(imus[index].init("RS232", 5, order, 0, 0, itr->first, itr->second, index * sizeof(IMURXData), index * sizeof(IMUTXData)) != 0){
#endif
        printf("\timus[%d] init failed\n", index);
        return -1;
    }
    if(imus[index].config("RS232", order, 0, rxSwap, txSwap) != 0){
        printf("\timus[%d] config failed\n", index);
        return -1;
    }
    txSwap_ = new SwapList(frameLength);
    int i = 0;
    while(i < 3){
        txSwap_->nodePtr.load()->memPtr[0] = header0;
        txSwap_->nodePtr.load()->memPtr[1] = header1;
        txSwap_->advanceNodePtr();
        ++i;
    }
    return 0;
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
    RS232* rs232 = (RS232*)arg;
    int    speedArray[] = {B921600, B576000, B460800, B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int baudrateArray[] = { 921600,  576000,  460800,  230400,  115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
    int i = 0, j;
    while(i < 13){
        if(baudrateArray[i] == rs232->baudrate){
            break;
        }
        ++i;
    }
    if(i == 13){
        printf("invalid rs232s[%d] baudrate %d\n", rs232->order, rs232->baudrate);
        exit(-1);
    }
#ifndef NIIC
    rs232->fd = open(rs232->device, O_RDONLY | O_NOCTTY);
#else
    rs232->fd = __RT(open(rs232->device, O_RDONLY | O_NOCTTY));
#endif
    if(rs232->fd < 0){
        printf("opening rs232s[%d] device %s failed\n", rs232->order, rs232->device);
        exit(-1);
    }
    struct termios opt;
    tcgetattr(rs232->fd, &opt);
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
    tcsetattr(rs232->fd, TCSANOW, &opt);
    tcflush(rs232->fd, TCIOFLUSH);
    printf("opened rs232s[%d] %s baudrate %d\n", rs232->order, rs232->device, rs232->baudrate);
    unsigned char buff[2 * rs232->frameLength], * buffA = buff, * buffB = buff + rs232->frameLength;
    ChainNode* node0 = new ChainNode(), * node = node0;
    i = 0;
    while(i < 2 * rs232->frameLength - 1){
        node->nr = i;
        node->next = new ChainNode();
        node->next->previous = node;
        node = node->next;
        ++i;
    }
    node->nr = 2 * rs232->frameLength - 1;
    node->next = node0;
    node0->previous = node;
    node = node0;
    pthread_cleanup_push(cleanup, node0);
    j = 0;
    do{
#ifndef NIIC
        j += read(rs232->fd, buffA + j, rs232->frameLength - j);
#else
        j += __RT(read(rs232->fd, buffA + j, rs232->frameLength - j));
#endif
    }while(j < rs232->frameLength);
    struct timespec step{0, 1000};
    i = 2;
    while(true){
        if(node->nr == rs232->frameLength - 1){
            j = 0;
            do{
#ifndef NIIC
                j += read(rs232->fd, buffB + j, rs232->frameLength - j);
#else
                j += __RT(read(rs232->fd, buffB + j, rs232->frameLength - j));
#endif
            }while(j < rs232->frameLength);
        }else if(node->nr == 2 * rs232->frameLength - 1){
            j = 0;
            do{
#ifndef NIIC
                j += read(rs232->fd, buffA + j, rs232->frameLength - j);
#else
                j += __RT(read(rs232->fd, buffA + j, rs232->frameLength - j));
#endif
            }while(j < rs232->frameLength);
        }
        node = node->next;
        if(i < rs232->frameLength){
            rs232->txSwap_->nodePtr.load()->next->memPtr[i] = buff[node->nr];
            ++i;
        }
        if(buff[node->previous->nr] == rs232->header0 && buff[node->nr] == rs232->header1){
            if(rs232->valid(rs232->txSwap_->nodePtr.load()->next->memPtr)){
                rs232->txSwap_->advanceNodePtr();
            }
            memset(rs232->txSwap_->nodePtr.load()->next->memPtr + 2, 0, rs232->frameLength - 2);
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
    if(alias2type.size() == 0){
        return 0;
    }
    if(strlen(device) == 0){
        return 1;
    }
#ifndef NIIC
    if(pthread_create(&pth, nullptr, &recv, this) != 0){
#else
    if(__RT(pthread_create(&pth, nullptr, &recv, this)) != 0){
#endif
        printf("creating rs232s[%d] recv thread failed\n", order);
        return -1;
    }
    if(pthread_detach(pth) != 0){
        printf("detaching rs232s[%d] recv thread failed\n", order);
        return -1;
    }
    printf("rs232s[%d] recv\n", order);
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
    if(rxSwap != nullptr){
        delete rxSwap;
        rxSwap = nullptr;
    }
    if(txSwap != nullptr){
        delete txSwap;
        txSwap = nullptr;
    }
    if(txSwap_ != nullptr){
        delete txSwap_;
        txSwap_ = nullptr;
    }
    if(device != nullptr){
        free(device);
        device = nullptr;
    }
    if(type != nullptr){
        free(type);
        type = nullptr;
    }
}
}