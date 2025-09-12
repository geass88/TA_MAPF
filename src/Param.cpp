/**
 * @file Param.cpp
 * @author Tommaso Adamo (tommaso.adamo@unisalento.it)
 * @brief
 *
 * @copyright Copyright (c) 2023 - University of Salento
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "Param.hpp"
#include <fstream>
#include <filesystem>

namespace unisalento
{

    Param::Param(std::string_view file)
    {
        if (std::filesystem::exists(file))
        {
            std::ifstream fin(file.data());
            while (!fin.eof())
            {
                std::string key, value;
                fin >> key >> value;
                // DBG << key << " " << value << std::endl;
                set(key, value);
            }
            fin.close();
        }
    }

    int Param::getInt(std::string_view key) const
    {
        return std::stoi(entries.at(key.data()));
    }

    bool Param::getBool(std::string_view key) const
    {
        return entries.at(key.data()) == "true";
    }

    double Param::getDouble(std::string_view key) const
    {
        return std::stod(entries.at(key.data()));
    }

    std::string Param::getValue(std::string_view key) const
    {
        return entries.at(key.data());
    }

    void Param::set(const std::string &key, std::string value)
    {
        entries[key] = std::move(value);
    }

    std::ostream &operator<<(std::ostream &os, const Param &param)
    {
        for (const auto &[key, val] : param.entries)
        {
            os << key << " " << val << std::endl;
        }
        return os;
    }

}
