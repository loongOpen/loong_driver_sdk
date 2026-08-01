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
#include <sstream>
#include <limits>

namespace DriverSDK{
ConfigXML::ConfigXML(char const* name){
    this->name = (char*)malloc(strlen(name) + 1);
    strcpy(this->name, name);
    xmlDoc.LoadFile(name);
}

int ConfigXML::writeMotorParameter(int const alias, char const* parameter, float const value){
    tinyxml2::XMLElement* motorElement = xmlDoc.FirstChildElement("Config")->FirstChildElement("Motors")->FirstChildElement("Motor");
    while(motorElement != nullptr){
        if(motorElement->IntAttribute("alias") == alias){
            tinyxml2::XMLElement* parameterElement = motorElement->FirstChildElement(parameter);
            if(parameterElement == nullptr){
                return -1;
            }else{
                parameterElement->SetText(value);
                return 0;
            }
        }
        motorElement = motorElement->NextSiblingElement("Motor");
    }
    return -1;
}

float ConfigXML::readMotorParameter(int const alias, char const* parameter){
    tinyxml2::XMLElement* motorElement = xmlDoc.FirstChildElement("Config")->FirstChildElement("Motors")->FirstChildElement("Motor");
    while(motorElement != nullptr){
        if(motorElement->IntAttribute("alias") == alias){
            tinyxml2::XMLElement* parameterElement = motorElement->FirstChildElement(parameter);
            if(parameterElement == nullptr){
                return std::numeric_limits<float>::min();
            }else{
                return parameterElement->FloatText();
            }
        }
        motorElement = motorElement->NextSiblingElement("Motor");
    }
    return std::numeric_limits<float>::min();
}

float ConfigXML::readDeviceParameter(char const* bus, char const* type, char const* parameter){
    tinyxml2::XMLElement* deviceElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Devices")->FirstChildElement("Device");
    while(deviceElement != nullptr){
        if(strcmp(deviceElement->Attribute("type"), type) == 0){
            tinyxml2::XMLElement* parameterElement = deviceElement->FirstChildElement(parameter);
            if(parameterElement == nullptr){
                return 0.0;
            }else{
                return parameterElement->FloatText();
            }
        }
        deviceElement = deviceElement->NextSiblingElement("Device");
    }
    return std::numeric_limits<float>::min();
}

std::vector<std::vector<int>> ConfigXML::motorAlias(){
    std::vector<std::vector<int>> ret;
    tinyxml2::XMLElement* motorElement = xmlDoc.FirstChildElement("Config")->FirstChildElement("Motors")->FirstChildElement("Motor");
    while(motorElement != nullptr){
        int limb = motorElement->IntAttribute("limb"), motor = motorElement->IntAttribute("motor");
        while(ret.size() <= limb){
            ret.push_back(std::vector<int>());
        }
        while(ret[limb].size() <= motor){ 
            ret[limb].push_back(0);
        }
        ret[limb][motor] = motorElement->IntAttribute("alias");
        motorElement = motorElement->NextSiblingElement("Motor");
    }
    while(ret.size() < 6){
        ret.push_back(std::vector<int>());
    }
    return ret;
}

std::string ConfigXML::imuAttribute(char const* name){
    return xmlDoc.FirstChildElement("Config")->FirstChildElement("IMU")->Attribute(name);
}

int ConfigXML::imuBaudrate(){
    return xmlDoc.FirstChildElement("Config")->FirstChildElement("IMU")->IntAttribute("baudrate");
}

int ConfigXML::canAttribute(char const* name){
    tinyxml2::XMLElement* mastersElement = xmlDoc.FirstChildElement("Config")->FirstChildElement("CAN")->FirstChildElement("Masters");
    if(mastersElement != nullptr){
        tinyxml2::XMLAttribute const* attribute = mastersElement->FindAttribute(name);
        if(attribute == nullptr){
            return 0;
        }else{
            return attribute->IntValue();
        }
    }
    return 0;
}

std::string ConfigXML::masterDevice(char const* bus, int const order, char const* name){
    tinyxml2::XMLElement* masterElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Masters")->FirstChildElement("Master");
    while(masterElement != nullptr){
        if(masterElement->IntAttribute("order") == order){
            tinyxml2::XMLAttribute const* attribute = masterElement->FindAttribute(name);
            if(attribute == nullptr){
                return "";
            }else{
                return attribute->Value();
            }
        }
        masterElement = masterElement->NextSiblingElement("Master");
    }
    return "";
}

int ConfigXML::masterAttribute(char const* bus, int const order, char const* name){
    tinyxml2::XMLElement* masterElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Masters")->FirstChildElement("Master");
    while(masterElement != nullptr){
        if(masterElement->IntAttribute("order") == order){
            tinyxml2::XMLAttribute const* attribute = masterElement->FindAttribute(name);
            if(attribute == nullptr){
                return 0;
            }else{
                return attribute->IntValue();
            }
        }
        masterElement = masterElement->NextSiblingElement("Master");
    }
    return 0;
}

bool ConfigXML::masterFeature(char const* bus, int const order, char const* name){
    tinyxml2::XMLElement* masterElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Masters")->FirstChildElement("Master");
    while(masterElement != nullptr){
        if(masterElement->IntAttribute("order") == order){
            tinyxml2::XMLAttribute const* attribute = masterElement->FindAttribute(name);
            if(attribute == nullptr){
                return false;
            }else{
                return attribute->BoolValue();
            }
        }
        masterElement = masterElement->NextSiblingElement("Master");
    }
    return false;
}

std::vector<std::vector<int>> ConfigXML::domainDivisions(char const* bus){
    std::vector<std::vector<int>> ret;
    if(xmlDoc.FirstChildElement("Config")->FirstChildElement(bus) == nullptr){
        return ret;
    }
    tinyxml2::XMLElement* domainElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Domains")->FirstChildElement("Domain");
    while(domainElement != nullptr){
        int master = domainElement->IntAttribute("master"), order = domainElement->IntAttribute("order");
        while(ret.size() <= master){
            ret.push_back(std::vector<int>());
        }
        while(ret[master].size() <= order){
            ret[master].push_back(1);
        }
        ret[master][order] = domainElement->IntAttribute("division");
        domainElement = domainElement->NextSiblingElement("Domain");
    }
    return ret;
}

tinyxml2::XMLElement* ConfigXML::device(char const* bus, char const* VendorID, char const* ProductCode){
    tinyxml2::XMLElement* deviceElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Devices")->FirstChildElement("Device");
    while(deviceElement != nullptr){
        if(strcmp(deviceElement->FirstChildElement("VendorID")->GetText(), VendorID) == 0 && strcmp(deviceElement->FirstChildElement("ProductCode")->GetText(), ProductCode) == 0){
            return deviceElement;
        }
        deviceElement = deviceElement->NextSiblingElement("Device");
    }
    return nullptr;
}

tinyxml2::XMLElement* ConfigXML::device(char const* bus, char const* type){
    tinyxml2::XMLElement* deviceElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Devices")->FirstChildElement("Device");
    while(deviceElement != nullptr){
        if(strcmp(deviceElement->Attribute("type"), type) == 0){
            return deviceElement;
        }
        deviceElement = deviceElement->NextSiblingElement("Device");
    }
    return nullptr;
}

std::string ConfigXML::deviceType(tinyxml2::XMLElement const* deviceElement){
    return deviceElement->Attribute("type");
}

std::string ConfigXML::typeCategory(char const* bus, char const* type){
    tinyxml2::XMLElement* categoryElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Categories")->FirstChildElement("Category");
    while(categoryElement != nullptr){
        tinyxml2::XMLElement* typeElement = categoryElement->FirstChildElement("Type");
        while(typeElement != nullptr){
            if(strcmp(typeElement->GetText(), type) == 0){
                return categoryElement->Attribute("name");
            }
            typeElement = typeElement->NextSiblingElement("Type");
        }
        categoryElement = categoryElement->NextSiblingElement("Category");
    }
    printf("the category is not specified of device type %s\n", type);
    return "";
}

std::string ConfigXML::typeAttribute(char const* bus, char const* type, char const* name){
    tinyxml2::XMLElement* categoryElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Categories")->FirstChildElement("Category");
    while(categoryElement != nullptr){
        tinyxml2::XMLElement* typeElement = categoryElement->FirstChildElement("Type");
        while(typeElement != nullptr){
            if(strcmp(typeElement->GetText(), type) == 0){
                return typeElement->Attribute(name);
            }
            typeElement = typeElement->NextSiblingElement("Type");
        }
        categoryElement = categoryElement->NextSiblingElement("Category");
    }
    printf("the attribute %s is not found of device type %s\n", name, type);
    return "";
}

unsigned int ConfigXML::vendorID(tinyxml2::XMLElement const* deviceElement){
    return strtoul(deviceElement->FirstChildElement("VendorID")->GetText(), nullptr, 16);
}

unsigned int ConfigXML::productCode(tinyxml2::XMLElement const* deviceElement){
    return strtoul(deviceElement->FirstChildElement("ProductCode")->GetText(), nullptr, 16);
}

std::vector<std::vector<std::string>> ConfigXML::pdos(tinyxml2::XMLElement* const deviceElement, char const* rxtx){
    std::vector<std::vector<std::string>> ret;
    tinyxml2::XMLElement* pdosElement = deviceElement->FirstChildElement(rxtx);
    while(pdosElement != nullptr){
        std::vector<std::string> tmp;
        tmp.push_back(pdosElement->Attribute("index"));
        tinyxml2::XMLElement* objectElement = pdosElement->FirstChildElement("Object");
        while(objectElement != nullptr){
            tmp.push_back(objectElement->GetText());
            objectElement = objectElement->NextSiblingElement("Object");
        }
        ret.push_back(tmp);
        pdosElement = pdosElement->NextSiblingElement(rxtx);
    }
    return ret;
}

int ConfigXML::txDivision(tinyxml2::XMLElement* const deviceElement, char const* index){
    tinyxml2::XMLElement* txPDOsElement = deviceElement->FirstChildElement("TxPDOs");
    while(txPDOsElement != nullptr){
        if(strcmp(txPDOsElement->Attribute("index"), index) == 0){
            tinyxml2::XMLAttribute const* attribute = txPDOsElement->FindAttribute("division");
            if(attribute == nullptr){
                return 1;
            }else{
                return attribute->IntValue();
            }
        }
        txPDOsElement = txPDOsElement->NextSiblingElement("TxPDOs");
    }
    return 1;
}

std::vector<std::string> ConfigXML::entry(tinyxml2::XMLElement* const deviceElement, char const* object){
    std::vector<std::string> ret;
    tinyxml2::XMLElement* dictionaryElement = deviceElement->FirstChildElement("Dictionary");
    ret.push_back(object);
    tinyxml2::XMLElement* objectElement = dictionaryElement->FirstChildElement("Object");
    while(objectElement != nullptr){
        if(strcmp(objectElement->GetText(), object) == 0){
            ret.push_back(objectElement->Attribute("index"));
            ret.push_back(objectElement->Attribute("subindex"));
            ret.push_back(objectElement->Attribute("signed"));
            ret.push_back(objectElement->Attribute("bit_length"));
            ret.push_back(objectElement->Attribute("operation"));
            return ret;
        }
        objectElement = objectElement->NextSiblingElement("Object");
    }
    return ret;
}

std::vector<std::map<int, std::string>> ConfigXML::alias2type(char const* bus, std::vector<std::vector<std::tuple<std::vector<int>, int, std::string>>>* const aliases2domainType, bool emu){
    std::vector<std::map<int, std::string>> ret;
    if(xmlDoc.FirstChildElement("Config")->FirstChildElement(bus) == nullptr){
        return ret;
    }
    tinyxml2::XMLElement* slaveElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Slaves")->FirstChildElement("Slave");
    while(slaveElement != nullptr){
        if(slaveElement->IntText() != 1){
            slaveElement = slaveElement->NextSiblingElement("Slave");
            continue;
        }
        int master = 0;
        if(slaveElement->FindAttribute("master") != nullptr){
            master = slaveElement->IntAttribute("master");
        }else if(aliases2domainType != nullptr && slaveElement->FindAttribute("alias") != nullptr){
            int i = 0, count = 0, alias = slaveElement->IntAttribute("alias");
            while(i < aliases2domainType->size()){
                int j = 0;
                while(j < (*aliases2domainType)[i].size()){
                    std::vector<int> aliases;
                    std::tie(aliases, std::ignore, std::ignore) = (*aliases2domainType)[i][j];
                    int k = 0;
                    while(k < aliases.size()){
                        if(alias == aliases[k]){
                            master = count;
                            j = (*aliases2domainType)[i].size() - 1;
                            i = aliases2domainType->size() - 1;
                            break;
                        }
                        ++k;
                    }
                    ++count;
                    ++j;
                }
                ++i;
            }
        }
        while(ret.size() <= master){
            ret.push_back(std::map<int, std::string>());
        }
        while(!emu && aliases2domainType != nullptr && aliases2domainType->size() <= master){
            aliases2domainType->push_back(std::vector<std::tuple<std::vector<int>, int, std::string>>());
        }
        if(slaveElement->FindAttribute("alias") == nullptr){
            if(emu || aliases2domainType == nullptr || slaveElement->FindAttribute("aliases") == nullptr){
                printf("invalid alias of device on bus %s\n", bus);
                exit(-1);
            }else{
                std::vector<int> items;
                std::stringstream ss(slaveElement->Attribute("aliases"));
                std::string token;
                while(std::getline(ss, token, ' ')){
                    items.push_back(atoi(token.c_str()));
                }
                if(items.size() == 0){
                    printf("invalid aliases of device on bus %s\n", bus);
                    exit(-1);
                }
                (*aliases2domainType)[master].push_back(std::make_tuple(items, slaveElement->IntAttribute("domain"), slaveElement->Attribute("type")));
            }
        }else{
            int alias = slaveElement->IntAttribute("alias");
            if(ret[master].find(alias) != ret[master].end()){
                printf("duplicate alias on bus %s master %d\n", bus, master);
                exit(-1);
            }
            ret[master].emplace(alias, slaveElement->Attribute("type"));
        }
        slaveElement = slaveElement->NextSiblingElement("Slave");
    }
    return ret;
}

std::vector<std::map<int, int>> ConfigXML::alias2attribute(char const* bus, char const* name, std::vector<std::vector<std::tuple<std::vector<int>, int, std::string>>> const* aliases2domainType){
    std::vector<std::map<int, int>> ret;
    if(xmlDoc.FirstChildElement("Config")->FirstChildElement(bus) == nullptr){
        return ret;
    }
    tinyxml2::XMLElement* slaveElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Slaves")->FirstChildElement("Slave");
    while(slaveElement != nullptr){
        if(slaveElement->IntText() != 1){
            slaveElement = slaveElement->NextSiblingElement("Slave");
            continue;
        }
        int master = 0;
        if(slaveElement->FindAttribute("master") != nullptr){
            master = slaveElement->IntAttribute("master");
        }else if(aliases2domainType != nullptr && slaveElement->FindAttribute("alias") != nullptr){
            int i = 0, count = 0, alias = slaveElement->IntAttribute("alias");
            while(i < aliases2domainType->size()){
                int j = 0;
                while(j < (*aliases2domainType)[i].size()){
                    std::vector<int> aliases;
                    std::tie(aliases, std::ignore, std::ignore) = (*aliases2domainType)[i][j];
                    int k = 0;
                    while(k < aliases.size()){
                        if(alias == aliases[k]){
                            master = count;
                            j = (*aliases2domainType)[i].size() - 1;
                            i = aliases2domainType->size() - 1;
                            break;
                        }
                        ++k;
                    }
                    ++count;
                    ++j;
                }
                ++i;
            }
        }
        while(ret.size() <= master){
            ret.push_back(std::map<int, int>());
        }
        if(slaveElement->FindAttribute("alias") != nullptr){
            int alias = slaveElement->IntAttribute("alias");
            if(ret[master].find(alias) != ret[master].end()){
                printf("duplicate alias on bus %s master %d\n", bus, master);
                exit(-1);
            }
            if(slaveElement->FindAttribute(name) == nullptr){
                printf("missing %s of device with alias %d\n", name, alias);
                exit(-1);
            }
            ret[master].emplace(alias, slaveElement->IntAttribute(name));
        }
        slaveElement = slaveElement->NextSiblingElement("Slave");
    }
    return ret;
}

std::vector<std::map<int, std::vector<int>>> ConfigXML::alias2attribute_(char const* bus, char const* name, std::vector<std::vector<std::tuple<std::vector<int>, int, std::string>>> const* aliases2domainType){
    std::vector<std::map<int, std::vector<int>>> ret;
    if(xmlDoc.FirstChildElement("Config")->FirstChildElement(bus) == nullptr){
        return ret;
    }
    tinyxml2::XMLElement* slaveElement = xmlDoc.FirstChildElement("Config")->FirstChildElement(bus)->FirstChildElement("Slaves")->FirstChildElement("Slave");
    while(slaveElement != nullptr){
        if(slaveElement->IntText() != 1){
            slaveElement = slaveElement->NextSiblingElement("Slave");
            continue;
        }
        int master = 0;
        if(slaveElement->FindAttribute("master") != nullptr){
            master = slaveElement->IntAttribute("master");
        }else if(aliases2domainType != nullptr && slaveElement->FindAttribute("alias") != nullptr){
            int i = 0, count = 0, alias = slaveElement->IntAttribute("alias");
            while(i < aliases2domainType->size()){
                int j = 0;
                while(j < (*aliases2domainType)[i].size()){
                    std::vector<int> aliases;
                    std::tie(aliases, std::ignore, std::ignore) = (*aliases2domainType)[i][j];
                    int k = 0;
                    while(k < aliases.size()){
                        if(alias == aliases[k]){
                            master = count;
                            j = (*aliases2domainType)[i].size() - 1;
                            i = aliases2domainType->size() - 1;
                            break;
                        }
                        ++k;
                    }
                    ++count;
                    ++j;
                }
                ++i;
            }
        }
        while(ret.size() <= master){
            ret.push_back(std::map<int, std::vector<int>>());
        }
        if(slaveElement->FindAttribute("alias") != nullptr){
            int alias = slaveElement->IntAttribute("alias");
            if(ret[master].find(alias) != ret[master].end()){
                printf("duplicate alias on bus %s master %d\n", bus, master);
                exit(-1);
            }
            std::vector<int> items;
            if(slaveElement->FindAttribute(name) == nullptr){
                printf("missing %s of device with alias %d\n", name, alias);
                exit(-1);
            }
            std::stringstream ss(slaveElement->Attribute(name));
            std::string token;
            while(std::getline(ss, token, ' ')){
                items.push_back(atoi(token.c_str()));
            }
            if(items.size() == 0){
                printf("invalid %s of device with alias %d\n", name, alias);
                exit(-1);
            }
            ret[master].emplace(alias, items);
        }
        slaveElement = slaveElement->NextSiblingElement("Slave");
    }
    return ret;
}

tinyxml2::XMLError ConfigXML::save(){
    return xmlDoc.SaveFile(name);
}

ConfigXML::~ConfigXML(){
    free(name);
}
}