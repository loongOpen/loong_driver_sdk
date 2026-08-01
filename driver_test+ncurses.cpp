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
#include <curses.h>
#include <cmath>

double const Pi = std::acos(-1);

int main(int argc, char** argv){
    int enabled = 0, count = 2, count_ = 2;
    DriverSDK::DriverSDK& driverSDK = DriverSDK::DriverSDK::instance();
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
    sleep(1);
    std::vector<int> activeMotors = driverSDK.getActiveMotors();
    int i = 0, j = 0, motorNr = driverSDK.getTotalMotorNr(), digitNr = driverSDK.getLeftDigitNr() + driverSDK.getRightDigitNr(), imuNr = driverSDK.getIMUNr();
    initscr();
    idlok(stdscr, true);
    scrollok(stdscr, true);
    setscrreg(4, 4 + imuNr + 2 + motorNr);
    printw("loong_driver_sdk %s\nimuNr %d\ndigitNr %d: %d + %d\nmotorNr %ld/%d: ", driverSDK.version().c_str(), imuNr, digitNr, driverSDK.getLeftDigitNr(), driverSDK.getRightDigitNr(), activeMotors.size(), motorNr);
    while(i < activeMotors.size()){
        printw("%d ", activeMotors[i] + 1);
        ++i;
    }
    printw("\n");
    refresh();
    std::vector<DriverSDK::motorSDOClass> sdoData;
    i = 0;
    while(i < motorNr){
        sdoData.emplace_back(i);
        ++i;
    }
    i = 0;
    while(i < motorNr){
        driverSDK.fillSDO(sdoData[i], "ActualPosition");
        ++i;
    }
    std::vector<DriverSDK::imuStruct> imuData;
    i = 0;
    while(i < imuNr){
        imuData.push_back(DriverSDK::imuStruct{});
        ++i;
    }
    std::vector<DriverSDK::digitTargetStruct> digitTargetData;
    std::vector<DriverSDK::digitActualStruct> digitActualData;
    i = 0;
    while(i < digitNr){
        digitTargetData.push_back(DriverSDK::digitTargetStruct{});
        digitActualData.push_back(DriverSDK::digitActualStruct{});
        ++i;
    }
    std::vector<DriverSDK::motorTargetStruct> motorTargetData;
    std::vector<DriverSDK::motorActualStruct> motorActualData;
    std::vector<int> encoderCounts;
    i = 0;
    while(i < motorNr){
        motorTargetData.push_back(DriverSDK::motorTargetStruct{});
        motorActualData.push_back(DriverSDK::motorActualStruct{});
        encoderCounts.emplace_back(0);
        ++i;
    }
    std::vector<DriverSDK::sensorStruct> sensorData;
    i = 0;
    while(i < 2){
        sensorData.push_back(DriverSDK::sensorStruct{});
        ++i;
    }
    float motorPositions[motorNr];
    if(argc == 2 && argv[1][0] == 'e'){
        goto effector;
    }
    if(argc == 3){
        enabled = atoi(argv[1]);
        count = count_ = atoi(argv[2]);
    }
    i = 0;
    while(i < 1600){
        driverSDK.advance();
        usleep(4000);
        driverSDK.advance();
        usleep(4000);
        driverSDK.getIMU(imuData);
        j = 0;
        while(j < imuNr){
            mvprintw(4 + j, 0, "imu%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, imuData[j].rpy[0], imuData[j].rpy[1], imuData[j].rpy[2], imuData[j].gyr[0], imuData[j].gyr[1], imuData[j].gyr[2], imuData[j].acc[0], imuData[j].acc[1], imuData[j].acc[2]);
            ++j;
        }
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            mvprintw(4 + imuNr + j, 0, "sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            ++j;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            mvprintw(4 + imuNr + 2 + j, 0, "motor%02d:\t%8.3f %8d 0x%04x 0x%04x", j + 1, motorActualData[j].pos, motorActualData[j].temp[0], motorActualData[j].statusWord, motorActualData[j].errorCode);
            ++j;
        }
        driverSDK.advance();
        usleep(4000);
        driverSDK.advance();
        usleep(4000);
        refresh();
        ++i;
    }
    i = 0;
    while(i < motorNr){
        motorPositions[i] = motorActualData[i].pos;
        motorTargetData[i].enabled = enabled;
        ++i;
    }
    i = 0;
    while(count > 0){
        while(i < 3200){
            driverSDK.getIMU(imuData);
            j = 0;
            while(j < imuNr){
                mvprintw(4 + j, 0, "imu%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, imuData[j].rpy[0], imuData[j].rpy[1], imuData[j].rpy[2], imuData[j].gyr[0], imuData[j].gyr[1], imuData[j].gyr[2], imuData[j].acc[0], imuData[j].acc[1], imuData[j].acc[2]);
                ++j;
            }
            driverSDK.getSensor(sensorData);
            j = 0;
            while(j < 2){
                mvprintw(4 + imuNr + j, 0, "sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
                ++j;
            }
            driverSDK.getMotorActual(motorActualData);
            j = 0;
            while(j < motorNr){
                motorTargetData[j].pos = motorPositions[j] * std::cos(Pi / 2.0 * i / 3200.0);
                motorTargetData[j].kp = 50.0;
                motorTargetData[j].kd = 0.85;
                mvprintw(4 + imuNr + 2 + j, 0, "motor%02d:\t%8.3f %8d 0x%04x 0x%04x", j + 1, motorActualData[j].pos, motorActualData[j].temp[0], motorActualData[j].statusWord, motorActualData[j].errorCode);
                ++j;
            }
            driverSDK.setMotorTarget(motorTargetData);
            usleep(4000);
            refresh();
            ++i;
        }
        usleep(4000);
        while(i > 0){
            driverSDK.getIMU(imuData);
            j = 0;
            while(j < imuNr){
                mvprintw(4 + j, 0,"imu%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, imuData[j].rpy[0], imuData[j].rpy[1], imuData[j].rpy[2], imuData[j].gyr[0], imuData[j].gyr[1], imuData[j].gyr[2], imuData[j].acc[0], imuData[j].acc[1], imuData[j].acc[2]);
                ++j;
            }
            driverSDK.getSensor(sensorData);
            j = 0;
            while(j < 2){
                mvprintw(4 + imuNr + j, 0, "sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
                ++j;
            }
            driverSDK.getMotorActual(motorActualData);
            j = 0;
            while(j < motorNr){
                motorTargetData[j].pos = motorPositions[j] * std::cos(Pi / 2.0 * i / 3200.0);
                motorTargetData[j].kp = 50.0;
                motorTargetData[j].kd = 0.85;
                mvprintw(4 + imuNr + 2 + j, 0, "motor%02d:\t%8.3f %8d 0x%04x 0x%04x", j + 1, motorActualData[j].pos, motorActualData[j].temp[0], motorActualData[j].statusWord, motorActualData[j].errorCode);
                ++j;
            }
            driverSDK.setMotorTarget(motorTargetData);
            usleep(4000);
            refresh();
            --i;
        }
        --count;
    }
    i = 0;
    while(i < motorNr){
        motorTargetData[i].enabled = 0;
        ++i;
    }
    i = 0;
    while(i < 1600){
        driverSDK.getIMU(imuData);
        j = 0;
        while(j < imuNr){
            mvprintw(4 + j, 0, "imu%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, imuData[j].rpy[0], imuData[j].rpy[1], imuData[j].rpy[2], imuData[j].gyr[0], imuData[j].gyr[1], imuData[j].gyr[2], imuData[j].acc[0], imuData[j].acc[1], imuData[j].acc[2]);
            ++j;
        }
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            mvprintw(4 + imuNr + j, 0, "sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            ++j;
        }
        driverSDK.getMotorActual(motorActualData);
        j = 0;
        while(j < motorNr){
            motorTargetData[j].pos = motorActualData[j].pos;
            mvprintw(4 + imuNr + 2 + j, 0, "motor%02d:\t%8.3f %8d 0x%04x 0x%04x", j + 1, motorActualData[j].pos, motorActualData[j].temp[0], motorActualData[j].statusWord, motorActualData[j].errorCode);
            ++j;
        }
        driverSDK.setMotorTarget(motorTargetData);
        usleep(4000);
        refresh();
        ++i;
    }
    i = 0;
    while(i < 3200){
        driverSDK.getIMU(imuData);
        j = 0;
        while(j < imuNr){
            mvprintw(4 + j, 0, "imu%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, imuData[j].rpy[0], imuData[j].rpy[1], imuData[j].rpy[2], imuData[j].gyr[0], imuData[j].gyr[1], imuData[j].gyr[2], imuData[j].acc[0], imuData[j].acc[1], imuData[j].acc[2]);
            ++j;
        }
        driverSDK.getSensor(sensorData);
        j = 0;
        while(j < 2){
            mvprintw(4 + imuNr + j, 0, "sensor%02d:\t%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f", j + 1, sensorData[j].F[0], sensorData[j].F[1], sensorData[j].F[2], sensorData[j].M[0], sensorData[j].M[1], sensorData[j].M[2]);
            ++j;
        }
        driverSDK.getMotorActual(motorActualData);
        driverSDK.getEncoderCount(encoderCounts);
        j = 0;
        while(j < motorNr){
            driverSDK.sendMotorSDORequest(sdoData[j]);
            driverSDK.recvMotorSDOResponse(sdoData[j]);
            motorTargetData[j].pos = motorActualData[j].pos;
            mvprintw(4 + imuNr + 2 + j, 0, "motor%02d:\t%8ld|%11d %8d 0x%04x 0x%04x", j + 1, sdoData[j].value, encoderCounts[j], motorActualData[j].temp[0], motorActualData[j].statusWord, motorActualData[j].errorCode);
            ++j;
        }
        driverSDK.setMotorTarget(motorTargetData);
        usleep(4000);
        refresh();
        ++i;
    }
    move(4, 0);
    i = 0;
    while(i < motorNr){
        driverSDK.advance();
        usleep(4000);
        if(enabled == 0 && count_ == 0){
            printw("motor%02d CountBias %d\n", i + 1, driverSDK.calibrate(i));
        }
        driverSDK.advance();
        usleep(4000);
        refresh();
        ++i;
    }
    endwin();
    return 0;
effector:
    i = 0;
    while(i < digitNr){
        digitTargetData[i].pos = 0;
        ++i;
    }
    move(4, 0);
    i = 0;
    while(i < 6400){
        driverSDK.getDigitActual(digitActualData);
        printw("digits:\t");
        j = 0;
        while(j < digitNr){
            digitTargetData[j].pos = std::abs(90.0 * std::sin(2.0 * Pi * i / 6400.0));
            printw("%8d\t", digitActualData[j].pos);
            ++j;
        }
        printw("\n");
        driverSDK.setDigitTarget(digitTargetData);
        driverSDK.advance();
        usleep(4000);
        refresh();
        ++i;
    }
    endwin();
    return 0;
}