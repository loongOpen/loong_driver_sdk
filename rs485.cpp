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
#include "rs485.h"
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits>

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> rs485alias2type;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
extern WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];

void changingTekRX(modbus_t* const ctx, int const alias){
    unsigned short data[4] = {0, 0, 100, 100};
    int position = 0;
    if(alias == 200){
        static unsigned int count = 0xffffffff;
        count++;
        if(count % 8 != 0){
            return;
        }
        position = digits[0].rx->TargetPosition;
    }else if(alias == 201){
        static unsigned int count = 0xffffffff;
        count++;
        if(count % 8 != 0){
            return;
        }
        position = digits[dofLeftEffector].rx->TargetPosition;
    }
    position *= 100;
    data[0] = position >> 16 & 0xffff;
    data[1] = position & 0xffff;
    modbus_set_slave(ctx, alias);
    if(modbus_write_register(ctx, 0x0100, 1) != 1){
        return;
    }
    usleep(4000);
    if(modbus_write_registers(ctx, 0x0102, 4, data) != 4){
        return;
    }
    usleep(4000);
    if(modbus_write_register(ctx, 0x0108, 1) != 1){
        return;
    }
}

void changingTekTX(modbus_t* const ctx, int const alias){
    unsigned short data[2] = {0, 0};
    modbus_set_slave(ctx, alias);
    if(modbus_read_registers(ctx, 0x0609, 2, data) != 2){
        return;
    }
    int position = 0;
    *((unsigned short*)&position + 1) = data[0];
    *(unsigned short*)&position = data[1];
    position = (position - 100) * 90 / (1150 - 100);
    if(alias == 200){
        digits[0].tx->ActualPosition = position;
    }else if(alias == 201){
        digits[dofLeftEffector].tx->ActualPosition = position;
    }
}

unsigned int const TargetPosRegs[] = {0x05d8, 0x05d6, 0x05d4, 0x05d2, 0x05d0, 0x05ce};

void InspireRX(modbus_t* const ctx, int const alias){
    unsigned short targetPositions[] = {0, 0, 0, 0, 0, 0};
    int i = 0, j = 0;
    if(alias == 200){
        i = 0;
    }else if(alias == 201){
        i = dofLeftEffector;
    }
    while(j < 6){
        targetPositions[j] = 1000 - digits[i + j].rx->TargetPosition * 1000 / 90;
        j++;
    }
    modbus_set_slave(ctx, alias);
    j = 0;
    while(j < 6){
        modbus_write_register(ctx, TargetPosRegs[j], targetPositions[j]);
        usleep(2000);
        j++;
    }
}

unsigned int const ActualPosRegs[] = {0x0614, 0x0612, 0x0610, 0x060e, 0x060c, 0x060a};

void InspireTX(modbus_t* const ctx, int const alias){
    unsigned short actualPositions[] = {0, 0, 0, 0, 0, 0};
    int readResults[] = {0, 0, 0, 0, 0, 0};
    modbus_set_slave(ctx, alias);
    int i = 0, j = 0;
    while(j < 6){
        readResults[j] = modbus_read_registers(ctx, ActualPosRegs[j], 1, actualPositions + j);
        usleep(2000);
        j++;
    }
    if(alias == 200){
        i = 0;
    }else if(alias == 201){
        i = dofLeftEffector;
    }
    j = 0;
    while(j < 6){
        if(readResults[j] == 1){
            digits[i + j].tx->ActualPosition = 90 - actualPositions[j] * 90 / 1000;
        }
        j++;
    }
}

RS485::RS485(int const order, char const* deviceR, char const* deviceS, long const period){
    this->order = order;
    alias2type = rs485alias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("rs485s[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        printf("\talias %d, type %s\n", itr->first, itr->second.c_str());
        itr++;
    }
    device = nullptr;
    this->deviceR = (char*)malloc(strlen(deviceR) + 1);
    this->deviceS = (char*)malloc(strlen(deviceS) + 1);
    strcpy(this->deviceR, deviceR);
    strcpy(this->deviceS, deviceS);
    baudrate = std::numeric_limits<int>::max();
    this->period = period;
    rxSwap = nullptr;
    txSwap = nullptr;
    ctx = nullptr;
    fdR = -1;
    fdS = -1;
    pth = 0;
    leftRX = rightRX = leftTX = rightTX = nullptr;
}

RS485::RS485(int const order, char const* device){
    this->order = order;
    alias2type = rs485alias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("rs485s[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        printf("\talias %d, type %s\n", itr->first, itr->second.c_str());
        itr++;
    }
    this->device = (char*)malloc(strlen(device) + 1);
    deviceR = nullptr;
    deviceS = nullptr;
    strcpy(this->device, device);
    baudrate = configXML->baudrate("RS485", order);
    period = configXML->period("RS485", order);
    rxSwap = nullptr;
    txSwap = nullptr;
    ctx = nullptr;
    fdR = -1;
    fdS = -1;
    pth = 0;
    leftRX = rightRX = leftTX = rightTX = nullptr;
}

int RS485::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    if(device == nullptr){
        if(access(deviceR, F_OK) == 0 && remove(deviceR) != 0){
            printf("removing %s failed\n", deviceR);
            return -1;
        }
        if(access(deviceS, F_OK) == 0 && remove(deviceS) != 0){
            printf("removing %s failed\n", deviceS);
            return -1;
        }
        if(mkfifo(deviceR, 0666) != 0){
            printf("creating %s failed\n", deviceR);
            return -1;
        }
        if(mkfifo(deviceS, 0666) != 0){
            printf("creating %s failed\n", deviceS);
            return -1;
        }
        ctx = modbus_new_ipc(deviceR, deviceS);
    }else{
        ctx = modbus_new_rtu(device, baudrate, 'N', 8, 1);
    }
    if(ctx == nullptr){
        return -1;
    }
    modbus_set_response_timeout(ctx, 0, 24000);
    if(device == nullptr){
        fdS = open(deviceS, O_RDONLY | O_NDELAY | O_CLOEXEC);
        if(fdS < 0){
            printf("opening %s failed\n", deviceS);
            return -1;
        }
    }
    if(modbus_connect(ctx) != 0){
        modbus_free(ctx);
        ctx = nullptr;
        return -1;
    }
    if(device == nullptr){
        fdR = open(deviceR, O_WRONLY | O_CLOEXEC);
        if(fdR < 0){
            printf("opening %s failed\n", deviceR);
            return -1;
        }
    }
    rxSwap = new SwapList(dofEffector * sizeof(DigitRxData));
    txSwap = new SwapList(dofEffector * sizeof(DigitTxData));
    int slave = 0;
    auto itr = alias2type.find(200);
    if(itr != alias2type.end()){
        int i = 0;
        while(i < dofLeftEffector){
            if(digits[i].init("RS485", order, 0, slave, 200, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr) != 0){
                printf("\tdigits[%d] init failed\n", i);
                return -1;
            }
            if(digits[i].config("RS485", order, 0, rxSwap, txSwap) != 0){
                printf("digits[%d] config failed\n", i);
                return -1;
            }
            i++;
        }
        if(itr->second == "ChangingTek"){
            leftRX = changingTekRX;
            leftTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            leftRX = InspireRX;
            leftTX = InspireTX;
        }else{
            ;
        }
        slave++;
    }
    itr = alias2type.find(201);
    if(itr != alias2type.end()){
        int i = dofLeftEffector;
        while(i < dofEffector){
            if(digits[i].init("RS485", order, 0, slave, 201, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr) != 0){
                printf("\tdigits[%d] init failed\n", i);
                return -1;
            }
            if(digits[i].config("RS485", order, 0, rxSwap, txSwap) != 0){
                printf("digits[%d] config failed\n", i);
                return -1;
            }
            i++;
        }
        if(itr->second == "ChangingTek"){
            rightRX = changingTekRX;
            rightTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            rightRX = InspireRX;
            rightTX = InspireTX;
        }else{
            ;
        }
        slave++;
    }
    return 0;
}

void* RS485::rxtx(void* arg){
    RS485* rs485 = (RS485*)arg;
    if(rs485->baudrate == std::numeric_limits<int>::max()){
        printf("rs485s[%d], deviceR %s, deviceS %s, period %ld\n", rs485->order, rs485->deviceR, rs485->deviceS, rs485->period);
    }else{
        printf("rs485s[%d], device %s, baudrate %d, period %ld\n", rs485->order, rs485->device, rs485->baudrate, rs485->period);
    }
    struct timespec currentTime, wakeupTime, step{0, 6 * rs485->period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        if(rs485->leftRX != nullptr){
            rs485->leftRX(rs485->ctx, 200);
            nanosleep(&step, nullptr);
        }
        if(rs485->rightRX != nullptr){
            rs485->rightRX(rs485->ctx, 201);
            nanosleep(&step, nullptr);
        }
        wakeupTime.tv_nsec += rs485->period;
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
            if(sleep && (TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime) < 6 * rs485->period / 100)){
                sleep = false;
            }
        }while(TIMESPEC2NS(currentTime) < TIMESPEC2NS(wakeupTime));
        if(rs485->leftTX != nullptr){
            rs485->leftTX(rs485->ctx, 200);
            nanosleep(&step, nullptr);
        }
        if(rs485->rightTX != nullptr){
            rs485->rightTX(rs485->ctx, 201);
            nanosleep(&step, nullptr);
        }
        rs485->txSwap->advanceNodePtr();
    }
    return nullptr;
}

int RS485::run(){
    if(alias2type.size() == 0){
        return 0;
    }
    if(pthread_create(&pth, nullptr, &rxtx, this) != 0){
        printf("creating rs485s[%d] rxtx thread failed\n", order);
        return -1;
    }
    printf("rs485s[%d] rxtx\n", order);
    return 0;
}

RS485::~RS485(){
    if(pth > 0){
        pthread_cancel(pth);
    }
    if(fdR > -1){
        close(fdR);
    }
    if(fdS > -1){
        close(fdS);
    }
    if(ctx != nullptr){
        modbus_close(ctx);
        modbus_free(ctx);
    }
    if(rxSwap != nullptr){
        delete rxSwap;
    }
    if(txSwap != nullptr){
        delete txSwap;
    }
    if(device != nullptr){
        free(device);
    }
    if(deviceR != nullptr){
        free(deviceR);
    }
    if(deviceS != nullptr){
        free(deviceS);
    }
}
}