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
extern int dofAll, dofLeftEffector, dofEffector;
extern WrapperPair<DriverRXData, DriverTXData, MotorParameters>* drivers;
extern WrapperPair<DigitRXData, DigitTXData, EffectorParameters>* digits;

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

void changingTekRX_(modbus_t* const ctx, int const alias){
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
    position *= 58;
    data[0] = position >> 16 & 0xffff;
    data[1] = position & 0xffff;
    modbus_set_slave(ctx, alias);
    if(modbus_write_registers(ctx, 0x0102, 4, data) != 4){
        return;
    }
    usleep(4000);
    modbus_write_register(ctx, 0x0108, 1);
}

void changingTekTX_(modbus_t* const ctx, int const alias){
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
    }else if(position > 420){
        position = 420;
    }
    position = position * 90 / 420;
    if(alias == 200){
        digits[0].tx.next()->ActualPosition = position;
    }else{
        digits[dofLeftEffector].tx.next()->ActualPosition = position;
    }
}

void dhRX(modbus_t* const ctx, int const alias){
    unsigned short position = 0;
    if(alias == 200){
        static long lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        static unsigned short lastPosition = 0;
        long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        unsigned short targetPosition = digits[0].rx.previous()->TargetPosition;
        if(std::abs(targetPosition - lastPosition) < 5 || currentTime - lastTime < 600){
            return;
        }
        lastTime = currentTime;
        position = lastPosition = targetPosition;
    }else{
        static long lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        static unsigned short lastPosition = 0;
        long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        unsigned short targetPosition = digits[dofLeftEffector].rx.previous()->TargetPosition;
        if(std::abs(targetPosition - lastPosition) < 5 || currentTime - lastTime < 600){
            return;
        }
        lastTime = currentTime;
        position = lastPosition = targetPosition;
    }
    position = (90 - position) * 1000 / 90;
    modbus_set_slave(ctx, alias);
    modbus_write_register(ctx, 0x0103, position);
}

void dhTX(modbus_t* const ctx, int const alias){
    short position = 0;
    modbus_set_slave(ctx, alias);
    if(modbus_read_registers(ctx, 0x0202, 1, (unsigned short*)&position) != 1){
        return;
    }
    if(position < 0){
        position = 0;
    }else if(position > 1000){
        position = 1000;
    }
    position = (1000 - position) * 90 / 1000;
    if(alias == 200){
        digits[0].tx.next()->ActualPosition = position;
    }else{
        digits[dofLeftEffector].tx.next()->ActualPosition = position;
    }
}

void agibotRX(modbus_t* const ctx, int const alias){
    unsigned short data[6] = {0x0000, 0x007f, 0x007f, 0x007f, 0x007f, 0x0001};
    if(alias == 200){
        data[0] = digits[0].rx.previous()->TargetPosition;
    }else{
        data[0] = digits[dofLeftEffector].rx.previous()->TargetPosition;
    }
    data[0] = (90 - data[0]) * 255 / 90;
    modbus_set_slave(ctx, alias);
    if(modbus_write_registers(ctx, 10, 6, data) != 6){
        return;
    }
}

void agibotTX(modbus_t* const ctx, int const alias){
    unsigned char data[2] = {0, 0};
    modbus_set_slave(ctx, alias);
    if(modbus_read_registers(ctx, 22, 1, (unsigned short*)data) != 1){
        return;
    }
    data[0] = (255 - data[0]) * 90 / 255;
    if(alias == 200){
        digits[0].tx.next()->ActualPosition = data[0];
    }else{
        digits[dofLeftEffector].tx.next()->ActualPosition = data[0];
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
        ++j;
    }
    modbus_set_slave(ctx, alias);
    j = 0;
    while(j < 6){
        modbus_write_register(ctx, TargetPosRegs[j], targetPositions[j]);
        usleep(2000);
        ++j;
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
        ++j;
    }
    j = 0;
    while(j < 6){
        if(readResults[j] == 1){
            digits[i + j].tx.next()->ActualPosition = 90 - actualPositions[j] * 90 / 1000;
        }
        ++j;
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
        ++j;
    }
    targetPositions[4] = 1350 - digits[i + 4].rx.previous()->TargetPosition * (1350 - 1200) / 90;
    targetPositions[5] = 1800 - digits[i + 5].rx.previous()->TargetPosition * (1800 - 600) / 90;
    modbus_set_slave(ctx, alias);
    j = 0;
    while(j < 6){
        modbus_write_register(ctx, TargetPosRegs_[j], targetPositions[j]);
        usleep(2000);
        ++j;
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
        ++j;
    }
    j = 0;
    while(j < 4){
        if(readResults[j] == 1){
            digits[i + j].tx.next()->ActualPosition = (1750 - actualPositions[j]) * 90 / (1750 - 900);
        }
        ++j;
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
    unsigned short data[] = {0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000, 0, 1000};
    int j = 0;
    while(j < 6){
        data[2 * j] = digits[i + j].rx.previous()->TargetPosition * 1000 / 90;
        ++j;
    }
    modbus_set_slave(ctx, alias);
    modbus_write_registers(ctx, 1010, 12, data);
}

void brainCoTX(modbus_t* const ctx, int const alias){
    int i = 0;
    if(alias == 201){
        i = dofLeftEffector;
    }
    unsigned short actualPositions[] = {0, 0, 0, 0, 0, 0};
    modbus_set_slave(ctx, alias);
    if(modbus_read_input_registers(ctx, 2000, 6, actualPositions) != 6){
        return;
    }
    int j = 0;
    while(j < 6){
        digits[i + j].tx.next()->ActualPosition = actualPositions[j] * 90 / 1000;
        ++j;
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
        ++j;
    }
    unsigned short data[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    j = 0;
    while(j < 6){
        data[j] = single2half(targetPositions[j]);
        ++j;
    }
    j = 0;
    while(j < 6){
        data[j + 6] = single2half(targetVelocities[j]);
        ++j;
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

void dihoolRX(modbus_t* const ctx, int const alias){
    usleep(180000);
    if(drivers[alias - 1].rx.previous()->Undefined != 1){
        return;
    }
    static long lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    static unsigned short lastPosition = 0;
    long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned short targetPosition = drivers[alias - 1].rx.previous()->TargetPosition;
    if(std::abs(targetPosition - lastPosition) < 2 || currentTime - lastTime < 1200){
        return;
    }
    lastTime = currentTime;
    lastPosition = targetPosition;
    modbus_set_slave(ctx, 1);
    modbus_write_register(ctx, 0x0002, targetPosition);
    usleep(180000);
}

void dihoolTX(modbus_t* const ctx, int const alias){
    usleep(180000);
    unsigned short actualPosition;
    modbus_set_slave(ctx, 1);
    if(modbus_read_registers(ctx, 0x0002, 1, &actualPosition) != 1){
        return;
    }
    drivers[alias - 1].tx.next()->ActualPosition = actualPosition;
    if(drivers[alias - 1].rx.previous()->Undefined == 1){
        drivers[alias - 1].tx.next()->StatusWord = 0x0007;
    }else{
        drivers[alias - 1].tx.next()->StatusWord = 0x0001;
    }
    usleep(180000);
}

RS485::RS485(int const order, char const* device){
    rxSwap = txSwap = rxSwap_ = txSwap_ = nullptr;
    ctx = nullptr;
    pth = 0;
    this->order = order;
    alias2type = rs485alias2type[order];
    if(alias2type.size() == 0){
        return;
    }
    printf("rs485s[%d]\n", order);
    auto itr = alias2type.begin();
    while(itr != alias2type.end()){
        printf("\talias %d, type %s\n", itr->first, itr->second.c_str());
        ++itr;
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
    ctx = modbus_new_rtu(device, baudrate, 'N', 8, 1);
    if(ctx == nullptr){
        return -1;
    }
    modbus_set_response_timeout(ctx, 0, 24000);
    if(modbus_connect(ctx) != 0){
        modbus_free(ctx);
        ctx = nullptr;
        return -2;
    }
    // modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    rxSwap  = new SwapList(dofAll      * sizeof(DriverRXData));
    txSwap  = new SwapList(dofAll      * sizeof(DriverTXData));
    rxSwap_ = new SwapList(dofEffector * sizeof( DigitRXData));
    txSwap_ = new SwapList(dofEffector * sizeof( DigitTXData));
    int slave = 0, k = 0;
    auto itr = alias2type.begin();
    if(itr != alias2type.end()){
        int alias = itr->first;
        std::string const& type = itr->second;
        std::string const category = configXML->typeCategory("RS485", type.c_str());
        if(category == "driver"){
#ifndef NIIC
            if(drivers[alias - 1].init("RS485", 4, order, 0, slave, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData), nullptr, nullptr) != 0){
#else
            if(drivers[alias - 1].init("RS485", 4, order, 0, slave, alias, type, k * sizeof(DriverRXData), k * sizeof(DriverTXData)) != 0){
#endif
                printf("\tdrivers[%d] init failed\n", alias - 1);
                return -1;
            }
            if(drivers[alias - 1].config("RS485", order, 0, rxSwap, txSwap) != 0){
                printf("\tdrivers[%d] config failed\n", alias - 1);
                return -1;
            }
            if(type == "DIHOOL"){
                alias2rxFunction.emplace(alias, dihoolRX);
                alias2txFunction.emplace(alias, dihoolTX);
            }
            ++k;
        }else if(category == "effector"){
            int i, j;
            if(alias == 200){
                i = 0;
                j = dofLeftEffector;
            }else if(alias == 201){
                i = dofLeftEffector;
                j = dofEffector;
            }else{
                printf("\tinvalid effector alias %d\n", alias);
                return -1;
            }
            while(i < j){
#ifndef NIIC
                if(digits[i].init("RS485", 4, order, 0, slave, alias, type, i * sizeof(DigitRXData), i * sizeof(DigitTXData), nullptr, nullptr) != 0){
#else
                if(digits[i].init("RS485", 4, order, 0, slave, alias, type, i * sizeof(DigitRXData), i * sizeof(DigitTXData)) != 0){
#endif
                    printf("\tdigits[%d] init failed\n", i);
                    return -1;
                }
                if(digits[i].config("RS485", order, 0, rxSwap_, txSwap_) != 0){
                    printf("\tdigits[%d] config failed\n", i);
                    return -1;
                }
                ++i;
            }
            if(type == "ChangingTek"){
                alias2rxFunction.emplace(alias, changingTekRX);
                alias2txFunction.emplace(alias, changingTekTX);
            }else if(type == "ChangingTek_"){
                alias2rxFunction.emplace(alias, changingTekRX_);
                alias2txFunction.emplace(alias, changingTekTX_);
            }else if(type == "DH-Robotics"){
                alias2rxFunction.emplace(alias, dhRX);
                alias2txFunction.emplace(alias, dhTX);
                modbus_set_slave(ctx, alias);
                modbus_write_register(ctx, 0x0100, 0x0001);
            }else if(type == "AGIBOT"){
                alias2rxFunction.emplace(alias, agibotRX);
                alias2txFunction.emplace(alias, agibotTX);
            }else if(type == "Inspire"){
                alias2rxFunction.emplace(alias, inspireRX);
                alias2txFunction.emplace(alias, inspireTX);
            }else if(type == "Inspire_"){
                alias2rxFunction.emplace(alias, inspireRX_);
                alias2txFunction.emplace(alias, inspireTX_);
            }else if(type == "BrainCo"){
                alias2rxFunction.emplace(alias, brainCoRX);
                alias2txFunction.emplace(alias, brainCoTX);
                modbus_set_slave(ctx, alias);
                modbus_write_register(ctx, 937, 0x0000);
            }else if(type == "HumanoidShanghai"){
                alias2rxFunction.emplace(alias, humanoidShanghaiRX);
                alias2txFunction.emplace(alias, humanoidShanghaiTX);
            }
        }
        ++slave;
        ++itr;
    }
    return 0;
}

void* RS485::rxtx(void* arg){
    RS485* rs485 = (RS485*)arg;
    printf("rs485s[%d], device %s, baudrate %d, period %ld\n", rs485->order, rs485->device, rs485->baudrate, rs485->period);
    struct timespec currentTime, wakeupTime, step{0, 6 * rs485->period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        ++step.tv_sec;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    while(true){
        auto itr = rs485->alias2rxFunction.begin();
        while(itr != rs485->alias2rxFunction.end()){
            itr->second(rs485->ctx, itr->first);
            nanosleep(&step, nullptr);
            ++itr;
        }
        wakeupTime.tv_nsec += rs485->period;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            ++wakeupTime.tv_sec;
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
        itr = rs485->alias2txFunction.begin();
        while(itr != rs485->alias2txFunction.end()){
            itr->second(rs485->ctx, itr->first);
            nanosleep(&step, nullptr);
            ++itr;
        }
        rs485->txSwap->advanceNodePtr();
        rs485->txSwap_->advanceNodePtr();
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
    if(rxSwap_ != nullptr){
        delete rxSwap_;
        rxSwap_ = nullptr;
    }
    if(txSwap_ != nullptr){
        delete txSwap_;
        txSwap_ = nullptr;
    }
    if(device != nullptr){
        free(device);
        device = nullptr;
    }
}
}