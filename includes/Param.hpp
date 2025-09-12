/**
 * @file Param.hpp
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

#pragma once

#include <unordered_map>
#include <string>
#include <string_view>
#include <ostream>

namespace unisalento
{
    /**
     * @brief Classe di utilità per la lettura dei parametri dell'algoritmo da file di testo.
     *
     */
    class Param
    {
    private:
        std::unordered_map<std::string, std::string> entries;

    public:
        explicit Param(std::string_view file);

        int getInt(std::string_view key) const;

        bool getBool(std::string_view key) const;

        double getDouble(std::string_view key) const;

        std::string getValue(std::string_view key) const;

        void set(const std::string &key, std::string value);

        friend std::ostream &operator<<(std::ostream &os, const Param &param);
    };
}
