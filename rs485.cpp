﻿/* Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司
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

namespace DriverSDK{
extern ConfigXML* configXML;
extern std::vector<std::map<int, std::string>> rs485Alias2type;
extern int dofLeg, dofArm, dofWaist, dofNeck, dofAll, dofLeftEffector, dofRightEffector, dofEffector;
extern WrapperPair<DriverRxData, DriverTxData, MotorParameters>* drivers;
extern WrapperPair<DigitRxData, DigitTxData, EffectorParameters>* digits;
extern WrapperPair<SensorRxData, SensorTxData, SensorParameters> sensors[2];

void changingTekRX(modbus_t* const ctx, int const alias){
    int position = 0;
    if(alias == 200){
        position = digits[0].rx->TargetPosition;
    }else if(alias == 201){
        position = digits[dofLeftEffector].rx->TargetPosition;
    }
    modbus_set_slave(ctx, alias);
    modbus_write_register(ctx, 0x0100, 1);
    modbus_write_register(ctx, 0x0102, position >> 16 & 0xffff);
    modbus_write_register(ctx, 0x0103, position & 0xffff);
    modbus_write_register(ctx, 0x0104, 100);
    modbus_write_register(ctx, 0x0105, 100);
    modbus_write_register(ctx, 0x0108, 1);
}

void changingTekTX(modbus_t* const ctx, int const alias){
    int position = 0;
    modbus_set_slave(ctx, alias);
    if(modbus_read_registers(ctx, 0x0609, 1, (unsigned short*)&position + 1) != 1){
        return;
    }
    if(modbus_read_registers(ctx, 0x060a, 1, (unsigned short*)&position) != 1){
        return;
    }
    position = (position - 100) * 9000 / (1150 - 100);
    if(alias == 200){
        digits[0].tx->ActualPosition = position;
    }else if(alias == 201){
        digits[dofLeftEffector].tx->ActualPosition = position;
    }
}

RS485::RS485(int const order, char const* device){
    this->order = order;
    this->device = (char*)malloc(strlen(device) + 1);
    strcpy(this->device, device);
    baudrate = configXML->baudrate("RS485", order);
    alias2type = rs485Alias2type[order];
    period = configXML->period("RS485", order);
    ctx = nullptr;
    pth = 0;
    leftRX = rightRX = leftTX = rightTX = nullptr;
}

int RS485::config(){
    if(alias2type.size() == 0){
        return 0;
    }
    ctx = modbus_new_rtu(device, baudrate, 'N', 8, 1);
    if(ctx == nullptr){
        return -1;
    }
    modbus_set_response_timeout(ctx, 1, 500000);
    if(modbus_connect(ctx) != 0){
        modbus_free(ctx);
        ctx = nullptr;
        return -1;
    }
    rxSwap = new SwapList(dofEffector * sizeof(DigitRxData));
    txSwap = new SwapList(dofEffector * sizeof(DigitTxData));
    auto itr = alias2type.find(200);
    if(itr != alias2type.end()){
        int i = 0;
        while(i < dofLeftEffector){
            digits[i].init("RS485", order, i, 200, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr);
            digits[i].config("RS485", order, rxSwap, txSwap);
            i++;
        }
        if(itr->second == "ChangingTek"){
            leftRX = changingTekRX;
            leftTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            ;
        }
    }
    itr = alias2type.find(201);
    if(itr != alias2type.end()){
        int i = dofLeftEffector;
        while(i < dofEffector){
            digits[i].init("RS485", order, i, 201, itr->second, i * sizeof(DigitRxData), i * sizeof(DigitTxData), nullptr);
            digits[i].config("RS485", order, rxSwap, txSwap);
            i++;
        }
        if(itr->second == "ChangingTek"){
            rightRX = changingTekRX;
            rightTX = changingTekTX;
        }else if(itr->second == "Inspire"){
            ;
        }
    }
    return 0;
}

void* RS485::rxtx(void* arg){
    RS485* rs485 = (RS485*)arg;
    printf("rs485s[%d] device %s baudrate %d, period %ld\n", rs485->order, rs485->device, rs485->baudrate, rs485->period);
    struct timespec currentTime, wakeupTime, step{0, 6 * rs485->period / 100};
    while(step.tv_nsec >= NSEC_PER_SEC){
        step.tv_nsec -= NSEC_PER_SEC;
        step.tv_sec++;
    }
    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);
    bool sleep = true;
    int count = 0;
    while(true){
        if(count % 8 == 0){
            if(rs485->leftRX != nullptr){
                rs485->leftRX(rs485->ctx, 200);
                nanosleep(&step, nullptr);
            }
            if(rs485->rightRX != nullptr){
                rs485->rightRX(rs485->ctx, 201);
                nanosleep(&step, nullptr);
            }
        }
        count++;
        wakeupTime.tv_nsec += rs485->period;
        while(wakeupTime.tv_nsec >= NSEC_PER_SEC){
            wakeupTime.tv_nsec -= NSEC_PER_SEC;
            wakeupTime.tv_sec++;
        }
        sleep = true;
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
    return 0;
}

RS485::~RS485(){
    if(pth > 0){
        pthread_cancel(pth);
    }
    if(rxSwap != nullptr){
        delete rxSwap;
    }
    if(txSwap != nullptr){
        delete txSwap;
    }
    if(ctx != nullptr){
        modbus_close(ctx);
        modbus_free(ctx);
    }
    free(device);
}
}