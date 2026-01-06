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

#include "loong_driver_sdk.h"
#include <stdio.h>
#include <unistd.h>
#include <cmath>

float const Pi = std::acos(-1);

int main(int argc, char** argv){
    DriverSDK::DriverSDK& driverSDK = DriverSDK::DriverSDK::instance();
    printf("loong_driver_sdk version: %s\n", driverSDK.version().c_str());
    std::vector<unsigned short> cpusECAT = {2, 3, 3, 3, 3, 3};
    driverSDK.setCPUs(cpusECAT, "ECAT");
    std::vector<unsigned short> cpusCAN = {4, 4, 4};
    driverSDK.setCPUs(cpusCAN, "CAN");
    std::vector<unsigned short> maxCurr = {
        1000, 1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000, 1000,
        1000, 1000, 1000, 1000, 1000, 600, 600,
        1000, 1000, 1000, 1000, 1000, 600, 600,
        1000, 1000, 1000,
        600, 600
    };
    driverSDK.setMaxCurr(maxCurr);
    std::vector<char> mode = {
        8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8, 8,
        8, 8, 8, 8, 8, 8, 8,
        8, 8, 8,
        8, 8
    };
    driverSDK.setMode(mode);
    driverSDK.init("configuration.xml");
    std::vector<int> activeMotors = driverSDK.getActiveMotors();
    int i = 0, j = 0, motorNr = driverSDK.getTotalMotorNr(), digitNr = driverSDK.getLeftDigitNr() + driverSDK.getRightDigitNr();
    printf("digitNr %d\nmotorNr %d:\t", digitNr, motorNr);
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
        driverSDK.advance();
        usleep(4000);
        driverSDK.advance();
        usleep(4000);
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2], imuData.gyr[0], imuData.gyr[1], imuData.gyr[2], imuData.acc[0], imuData.acc[1], imuData.acc[2]);
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
        driverSDK.advance();
        usleep(4000);
        driverSDK.advance();
        usleep(4000);
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
    while(true){
        while(i < 3200){
            driverSDK.getIMU(imuData);
            printf("imu:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2], imuData.gyr[0], imuData.gyr[1], imuData.gyr[2], imuData.acc[0], imuData.acc[1], imuData.acc[2]);
            driverSDK.getSensor(sensorData);
            j = 0;
            while(j < 2){
                printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
                j++;
            }
            driverSDK.getMotorActual(motorActualData);
            j = 0;
            while(j < motorNr){
                motorTargetData[j].pos = motorPositions[j] * std::cos(Pi / 2.0 * (float)i / 3200.0);
                motorTargetData[j].kp = 50.0;
                motorTargetData[j].kd = 0.85;
                printf("motor%02d:\t%8.3f %8d %8d %8d\n", j + 1, motorActualData[j].pos, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
                j++;
            }
            driverSDK.setMotorTarget(motorTargetData);
            usleep(4000);
            i++;
        }
        while(i > 0){
            driverSDK.getIMU(imuData);
            printf("imu:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2], imuData.gyr[0], imuData.gyr[1], imuData.gyr[2], imuData.acc[0], imuData.acc[1], imuData.acc[2]);
            driverSDK.getSensor(sensorData);
            j = 0;
            while(j < 2){
                printf("sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
                j++;
            }
            driverSDK.getMotorActual(motorActualData);
            j = 0;
            while(j < motorNr){
                motorTargetData[j].pos = motorPositions[j] * std::cos(Pi / 2.0 * (float)i / 3200.0);
                motorTargetData[j].kp = 50.0;
                motorTargetData[j].kd = 0.85;
                printf("motor%02d:\t%8.3f %8d %8d %8d\n", j + 1, motorActualData[j].pos, motorActualData[j].temp, motorActualData[j].statusWord, motorActualData[j].errorCode);
                j++;
            }
            driverSDK.setMotorTarget(motorTargetData);
            usleep(4000);
            i--;
        }
    }
    i = 0;
    while(i < motorNr){
        motorTargetData[i].enabled = 0;
        i++;
    }
    i = 0;
    while(i < 1600){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2], imuData.gyr[0], imuData.gyr[1], imuData.gyr[2], imuData.acc[0], imuData.acc[1], imuData.acc[2]);
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
        usleep(4000);
        i++;
    }
    i = 0;
    while(i < 3200){
        driverSDK.getIMU(imuData);
        printf("imu:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n", imuData.rpy[0], imuData.rpy[1], imuData.rpy[2], imuData.gyr[0], imuData.gyr[1], imuData.gyr[2], imuData.acc[0], imuData.acc[1], imuData.acc[2]);
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
        driverSDK.advance();
        usleep(4000);
        // printf("motor%02d CountBias %d\n", i + 1, driverSDK.calibrate(i));
        driverSDK.advance();
        usleep(4000);
        i++;
    }
    i = 0;
    while(i < digitNr){
        digitTargetData[i].pos = 0;
        i++;
    }
    i = 0;
    while(i < 6400){
        driverSDK.getDigitActual(digitActualData);
        printf("digits:\t");
        j = 0;
        while(j < digitNr){
            digitTargetData[j].pos = std::abs(90.0 * std::sin(2.0 * Pi * (float)i / 6400.0));
            printf("%8d\t", digitActualData[j].pos);
            j++;
        }
        printf("\n");
        driverSDK.setDigitTarget(digitTargetData);
        driverSDK.advance();
        usleep(4000);
        i++;
    }
    return 0;
}