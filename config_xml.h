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

#pragma once

#include <tinyxml2.h>
#include <string>
#include <vector>
#include <map>
#include <tuple>

namespace DriverSDK{
class ConfigXML{
public:
    char* file;
    tinyxml2::XMLDocument xmlDoc;
    ConfigXML(char const* file);
    int writeMotorParameter(int const alias, char const* parameter, float const value);
    float readMotorParameter(int const alias, char const* parameter);
    float readDeviceParameter(char const* bus, char const* type, char const* parameter);
    std::vector<std::vector<int>> motorAlias();
    std::vector<std::vector<int>> domainDivisions(char const* bus);
    std::string typeAttribute(char const* bus, char const* type, char const* name);
    std::string imuAttribute(char const* name);
    int imuBaudrate();
    int canAttribute(char const* name);
    std::vector<std::tuple<int, std::vector<int>, std::string>> canBus();
    std::string masterDevice(char const* bus, int const order, char const* name);
    int masterAttribute(char const* bus, int const order, char const* name);
    bool masterFeature(char const* bus, int const order, char const* name);
    tinyxml2::XMLElement* device(char const* bus, char const* VendorID, char const* ProductCode);
    tinyxml2::XMLElement* device(char const* bus, char const* type);
    std::string deviceType(tinyxml2::XMLElement const* deviceElement);
    std::string typeCategory(char const* bus, char const* type);
    unsigned int vendorID(tinyxml2::XMLElement const* deviceElement);
    unsigned int productCode(tinyxml2::XMLElement const* deviceElement);
    std::vector<std::vector<std::string>> pdos(tinyxml2::XMLElement* const deviceElement, char const* rxtx);
    std::vector<std::string> entry(tinyxml2::XMLElement* const deviceElement, char const* object);
    std::vector<std::map<int, std::string>> alias2type(char const* bus);
    std::vector<std::map<int, int>> alias2attribute(char const* bus, char const* name);
    tinyxml2::XMLError save();
    ~ConfigXML();
};
}