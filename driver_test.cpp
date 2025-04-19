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

#include "loong_driver_sdk.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char** argv){
    DriverSDK::DriverSDK& driverSDK = DriverSDK::DriverSDK::instance();
    printf("loong_driver_sdk version: %s\n", driverSDK.version().c_str());
    driverSDK.setCPU(2);
    std::vector<char> mode = {
        8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8, 8,
        8, 8, 8,
        8, 8
    };
    driverSDK.setMode(mode);
    std::vector<unsigned short> maxCurr = {
        1000, 1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000, 600, 600,
        1000, 1000, 1000, 1000, 1000, 600, 600,
        1000, 1000, 1000,
        600, 600
    };
    driverSDK.setMaxCurr(maxCurr);
    driverSDK.init("configuration.xml");
    std::vector<int> activeMotors = driverSDK.getActiveMotors();
    int i = 0, j = 0, motorNr = driverSDK.getTotalMotorNr(), digitNr = driverSDK.getLeftDigitNr() + driverSDK.getRightDigitNr();;
    printf("%d | %d: ", digitNr, motorNr);
    while(i < activeMotors.size()){
        printf("%d ", activeMotors[i]);
        i++;
    }
    printf("\n");
    std::vector<DriverSDK::motorSDOClass> sdoData;
    i = 0;
    while(i < motorNr){
        sdoData.emplace_back(i);
        i++;
    }
    i = 0;
    while(i < motorNr){
        driverSDK.fillSDO(sdoData[i], "ActualPosition");
        i++;
    }
    DriverSDK::imuStruct imuData;
    std::vector<DriverSDK::digitTargetStruct> digitTargetData;
    std::vector<DriverSDK::digitActualStruct> digitActualData;
    i = 0;
    while(i < digitNr){
        digitTargetData.push_back(DriverSDK::digitTargetStruct{});
        digitActualData.push_back(DriverSDK::digitActualStruct{});
        i++;
    }
    std::vector<DriverSDK::motorTargetStruct> motorTargetData;
    std::vector<DriverSDK::motorActualStruct> motorActualData;
    i = 0;
    while(i < motorNr){
        motorTargetData.push_back(DriverSDK::motorTargetStruct{});
        motorActualData.push_back(DriverSDK::motorActualStruct{});
        i++;
    }
    std::vector<DriverSDK::sensorStruct> sensorData;
    i = 0;
    while(i < 2){
        sensorData.push_back(DriverSDK::sensorStruct{});
        i++;
    }
    i = 0;
    while(i < 1600){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2]);
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            j++;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            printf("motor%02d:\t%8.3f %8d %8d %8d\n", j + 1, motorActualData[j].pos, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
            j++;
        }
        usleep(16000);
        i++;
    }
    float motorPositions[motorNr];
    i = 0;
    while(i < motorNr){
        motorPositions[i] = motorActualData[i].pos;
        // motorTargetData[i].enabled = 1;
        i++;
    }
    i = 0;
    while(i < 3200){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2]);
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            j++;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            motorTargetData[j].pos = motorPositions[j] * std::cos(DriverSDK::Pi / 2.0 * (float)i / 3200.0);
            printf("motor%02d:\t%8.3f %8d %8d %8d\n", j + 1, motorActualData[j].pos, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
            j++;
        }
        driverSDK.setMotorTarget(motorTargetData);
        usleep(4000);
        i++;
    }
    i = 0;
    while(i < motorNr){
        motorTargetData[i].enabled = 0;
        i++;
    }
    i = 0;
    while(i < 800){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2]);
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            j++;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            motorTargetData[j].pos = motorActualData[j].pos;
            printf("motor%02d:\t%8.3f %8d %8d %8d\n", j + 1, motorActualData[j].pos, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
            j++;
        }
        driverSDK.setMotorTarget(motorTargetData);
        usleep(8000);
        i++;
    }
    i = 0;
    while(i < 3200){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2]);
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            j++;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            driverSDK.sendMotorSDORequest(sdoData[j]);
            driverSDK.recvMotorSDOResponse(sdoData[j]);
            motorTargetData[j].pos = motorActualData[j].pos;
            printf("motor%02d:\t%8ld %8d %8d %8d\n", j + 1, sdoData[j].value, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
            j++;
        }
        driverSDK.setMotorTarget(motorTargetData);
        usleep(4000);
        i++;
    }
    i = 0;
    while(i < motorNr){
        printf("motor%02d CountBias %d\n", i + 1, driverSDK.calibrate(i));
        usleep(8000);
        i++;
    }
    i = 0;
    while(i < digitNr){
        digitTargetData[i].pos = 0;
        i++;
    }
    i = 0;
    while(i < 128){
        driverSDK.getDigitActual(digitActualData);
        printf("digits:\t");
        j = 0;
        while(j < digitNr){
            if(i % 8 == 0){
                digitTargetData[j].pos = std::abs(digitTargetData[j].pos - 90);
            }
            printf("%8d\t", digitActualData[j].pos);
            j++;
        }
        printf("\n");
        usleep(400000);
        if(i % 8 == 0){
            driverSDK.setDigitTarget(digitTargetData);
            driverSDK.setMotorTarget(motorTargetData);
        }
        usleep(400000);
        i++;
    }
    return 0;
}