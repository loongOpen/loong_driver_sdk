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
#include <chrono>
#include <pthread.h>

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> rs485alias2type;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;

void nullRX(modbus_t* const ctx, int const alias){
}

void nullTX(modbus_t* const ctx, int const alias){
}

void changingTekRX(modbus_t* const ctx, int const alias){
    unsigned short data[4] = {0, 0, 100, 100};
    int position = 0;
    if(alias == 200){
        static long lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        static unsigned short lastPosition = 0;
        long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        unsigned short targetPosition = digits[0].rx.previous()->TargetPosition;
        if(std::abs(targetPosition - lastPosition) < 5 || currentTime - lastTime < 800){
            return;
        }
        lastTime = currentTime;
        position = lastPosition = targetPosition;
    }else{
        static long lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        static unsigned short lastPosition = 0;
        long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        unsigned short targetPosition = digits[dofLeftEffector].rx.previous()->TargetPosition;
        if(std::abs(targetPosition - lastPosition) < 5 || currentTime - lastTime < 800){
            return;
        }
        lastTime = currentTime;
        position = lastPosition = targetPosition;
    }
    position *= 100;
    data[0] = position >> 16 & 0xffff;
    data[1] = position & 0xffff;
    modbus_set_slave(ctx, alias);
    if(modbus_write_registers(ctx, 0x0102, 4, data) != 4){
        return;
    }
    usleep(4000);
    modbus_write_register(ctx, 0x0108, 1);
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
    if(position < 0){
        position = 0;
    }else if(position > 1000){
        position = 1000;
    }
    position = position * 90 / 1000;
    if(alias == 200){
        digits[0].tx.next()->ActualPosition = position;
    }else{
        digits[dofLeftEffector].tx.next()->ActualPosition = position;
    }
}

unsigned int const TargetPosRegs[] = {0x05d8, 0x05d6, 0x05d4, 0x05d2, 0x05d0, 0x05ce};

void inspireRX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short targetPositions[] = {0, 0, 0, 0, 0, 0};
    int j = 0;
    while(j < 6){
        targetPositions[j] = 1000 - digits[i + j].rx.previous()->TargetPosition * 1000 / 90;
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

void inspireTX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short actualPositions[] = {0, 0, 0, 0, 0, 0};
    int readResults[] = {0, 0, 0, 0, 0, 0};
    modbus_set_slave(ctx, alias);
    int j = 0;
    while(j < 6){
        readResults[j] = modbus_read_registers(ctx, ActualPosRegs[j], 1, actualPositions + j);
        usleep(2000);
        j++;
    }
    j = 0;
    while(j < 6){
        if(readResults[j] == 1){
            digits[i + j].tx.next()->ActualPosition = 90 - actualPositions[j] * 90 / 1000;
        }
        j++;
    }
}

unsigned int const TargetPosRegs_[] = {0x0410, 0x0411, 0x0412, 0x0413, 0x0414, 0x0415};

void inspireRX_(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short targetPositions[] = {0, 0, 0, 0, 0, 0};
    int j = 0;
    while(j < 4){
        targetPositions[j] = 1750 - digits[i + j].rx.previous()->TargetPosition * (1750 - 900) / 90;
        j++;
    }
    targetPositions[4] = 1350 - digits[i + 4].rx.previous()->TargetPosition * (1350 - 1200) / 90;
    targetPositions[5] = 1800 - digits[i + 5].rx.previous()->TargetPosition * (1800 - 600) / 90;
    modbus_set_slave(ctx, alias);
    j = 0;
    while(j < 6){
        modbus_write_register(ctx, TargetPosRegs_[j], targetPositions[j]);
        usleep(2000);
        j++;
    }
}

unsigned int const ActualPosRegs_[] = {0x0428, 0x0429, 0x042a, 0x042b, 0x042c, 0x042d};

void inspireTX_(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short actualPositions[] = {0, 0, 0, 0, 0, 0};
    int readResults[] = {0, 0, 0, 0, 0, 0};
    modbus_set_slave(ctx, alias);
    int j = 0;
    while(j < 6){
        readResults[j] = modbus_read_registers(ctx, ActualPosRegs_[j], 1, actualPositions + j);
        usleep(2000);
        j++;
    }
    j = 0;
    while(j < 4){
        if(readResults[j] == 1){
            digits[i + j].tx.next()->ActualPosition = (1750 - actualPositions[j]) * 90 / (1750 - 900);
        }
        j++;
    }
    if(readResults[4] == 1){
        digits[i + 4].tx.next()->ActualPosition = (1350 - actualPositions[4]) * 90 / (1350 - 1200);
    }
    if(readResults[5] == 1){
        digits[i + 5].tx.next()->ActualPosition = (1800 - actualPositions[5]) * 90 / (1800 - 600);
    }
}

void brainCoRX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short targetPositions[] = {0, 0, 0, 0, 0, 0};
    int j = 0;
    while(j < 6){
        targetPositions[j] = digits[i + j].rx.previous()->TargetPosition * 100 / 90;
        j++;
    }
    modbus_set_slave(ctx, alias);
    modbus_write_registers(ctx, 1010, 6, targetPositions);
}

void brainCoTX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short actualPositions[] = {0, 0, 0, 0, 0, 0};
    modbus_set_slave(ctx, alias);
    if(modbus_read_registers(ctx, 1010, 6, actualPositions) != 6){
        return;
    }
    int j = 0;
    while(j < 6){
        digits[i + j].tx.next()->ActualPosition = actualPositions[j] * 90 / 100;
        j++;
    }
}

float const targetVelocities[] = {55.0, 42.76, 31.46, 31.46, 31.46, 31.46};

void humanoidShanghaiRX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    float targetPositions[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    targetPositions[0] = 138.22 - digits[i + 1].rx.previous()->TargetPosition * (138.22 - 92.22) / 90.0;
    targetPositions[1] = 7.26 - digits[i + 0].rx.previous()->TargetPosition * (7.26 - 96.15) / 90.0;
    int j = 2;
    while(j < 6){
        targetPositions[j] = 173.1 - digits[i + j].rx.previous()->TargetPosition * (173.1 - 84.33) / 90.0;
        j++;
    }
    unsigned short data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    j = 0;
    while(j < 6){
        data[j] = single2half(targetPositions[j]);
        j++;
    }
    j = 0;
    while(j < 6){
        data[j + 6] = single2half(targetVelocities[j]);
        j++;
    }
    modbus_set_slave(ctx, alias);
    modbus_write_registers(ctx, 0x0001, 12, data);
}

void humanoidShanghaiTX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
}

RS485::RS485(int const order, char const* deviceR, char const* deviceS){
    device = nullptr;
    rxSwap = nullptr;
    txSwap = nullptr;
    ctx = nullptr;
    fdR = -1;
    fdS = -1;
    pth = 0;
    leftRX = rightRX = leftTX = rightTX = nullptr;
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
    this->deviceR = (char*)malloc(strlen(deviceR) + 1);
    this->deviceS = (char*)malloc(strlen(deviceS) + 1);
    strcpy(this->deviceR, deviceR);
    strcpy(this->deviceS, deviceS);
    baudrate = std::numeric_limits<int>::max();
    period = configXML->masterAttribute("RS485Emu", order, "period");
}

RS485::RS485(int const order, char const* device){
    deviceR = nullptr;
    deviceS = nullptr;
    rxSwap = nullptr;
    txSwap = nullptr;
    ctx = nullptr;
    fdR = -1;
    fdS = -1;
    pth = 0;
    leftRX = rightRX = leftTX = rightTX = nullptr;
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
    strcpy(this->device, device);
    baudrate = configXML->masterAttribute("RS485", order, "baudrate");
    period = configXML->masterAttribute("RS485", order, "period");
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
        fdR = open(deviceR, O_WRONLY | O_CLOEXEC);
        if(fdR < 0){
            printf("opening %s failed\n", deviceR);
            return -1;
        }
    }
    if(modbus_connect(ctx) != 0){
        modbus_free(ctx);
        ctx = nullptr;
        return 1;
    }
    // modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    rxSwap = new SwapList(dofEffector * sizeof(DigitRxData));
    txSwap = new SwapList(dofEffector * sizeof(DigitTxData));
    int slave = 0;
    auto itr = alias2type.find(200);
    if(itr != alias2type.end()){
        int i = 0;
        while(i < dofLeftEffector){
#ifndef NIIC
            if(digits[i].init("RS485", 2, order, 0, slave, 200, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr, nullptr) != 0){
#else
            if(digits[i].init("RS485", 2, order, 0, slave, 200, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr) != 0){
#endif
                printf("\tdigits[%d] init failed\n", i);
                return -1;
            }
            if(digits[i].config("RS485", order, 0, rxSwap, txSwap) != 0){
                printf("\tdigits[%d] config failed\n", i);
                return -1;
            }
            i++;
        }
        if(itr->second == "ChangingTek"){
            leftRX = changingTekRX;
            leftTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            leftRX = inspireRX;
            leftTX = inspireTX;
        }else if(itr->second == "Inspire_"){
            leftRX = inspireRX_;
            leftTX = inspireTX_;
        }else if(itr->second == "BrainCo"){
            leftRX = brainCoRX;
            leftTX = brainCoTX;
        }else if(itr->second == "HumanoidShanghai"){
            leftRX = humanoidShanghaiRX;
            leftTX = humanoidShanghaiTX;
        }else{
            leftRX = nullRX;
            leftTX = nullTX;
        }
        slave++;
    }
    itr = alias2type.find(201);
    if(itr != alias2type.end()){
        int i = dofLeftEffector;
        while(i < dofEffector){
#ifndef NIIC
            if(digits[i].init("RS485", 2, order, 0, slave, 201, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr, nullptr) != 0){
#else
            if(digits[i].init("RS485", 2, order, 0, slave, 201, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr) != 0){
#endif
                printf("\tdigits[%d] init failed\n", i);
                return -1;
            }
            if(digits[i].config("RS485", order, 0, rxSwap, txSwap) != 0){
                printf("\tdigits[%d] config failed\n", i);
                return -1;
            }
            i++;
        }
        if(itr->second == "ChangingTek"){
            rightRX = changingTekRX;
            rightTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            rightRX = inspireRX;
            rightTX = inspireTX;
        }else if(itr->second == "Inspire_"){
            rightRX = inspireRX_;
            rightTX = inspireTX_;
        }else if(itr->second == "BrainCo"){
            rightRX = brainCoRX;
            rightTX = brainCoTX;
        }else if(itr->second == "HumanoidShanghai"){
            rightRX = humanoidShanghaiRX;
            rightTX = humanoidShanghaiTX;
        }else{
            rightRX = nullRX;
            rightTX = nullTX;
        }
        slave++;
    }
    if(slave == 0){
        printf("\tinvalid effector alias\n");
        return -1;
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
        long diff = 0;
        do{
            if(sleep){
                nanosleep(&step, nullptr);
            }
            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            diff = TIMESPEC2NS(wakeupTime) - TIMESPEC2NS(currentTime);
            if(sleep){
                if(diff < - 3 * rs485->period / 4){
                    wakeupTime = currentTime;
                }else if(diff < 9 * rs485->period / 100){
                    sleep = false;
                }
            }
        }while(diff > 0);
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
    if(pthread_detach(pth) != 0){
        printf("detaching rs485s[%d] rxtx thread failed\n", order);
        return -1;
    }
    printf("rs485s[%d] rxtx\n", order);
    return 0;
}

RS485::~RS485(){
    if(pth > 0){
        pthread_cancel(pth);
        pth = 0;
    }
    if(fdR > -1){
        close(fdR);
        fdR = -1;
    }
    if(fdS > -1){
        close(fdS);
        fdS = -1;
    }
    if(ctx != nullptr){
        modbus_close(ctx);
        modbus_free(ctx);
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
        free(device);
        device = nullptr;
    }
    if(deviceR != nullptr){
        free(deviceR);
        deviceR = nullptr;
    }
    if(deviceS != nullptr){
        free(deviceS);
        deviceS = nullptr;
    }
}
}