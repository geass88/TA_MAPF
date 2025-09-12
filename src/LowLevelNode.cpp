/**
 * @file LowLevelNode.cpp
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

#include "LowLevelNode.hpp"
// #include <boost/heap/d_ary_heap.hpp>

namespace unisalento {
    LowLevelNode::LowLevelNode() : fScore(-1), gScore(-1) {};

    LowLevelNode::LowLevelNode(const State &s, int f, int g, int c) : state(s), fScore(f), gScore(g), conflicts(c) {}

    bool LowLevelNode::operator<(const LowLevelNode &rhs) const {
        if (conflicts != rhs.conflicts) {
            return conflicts > rhs.conflicts;
        } else {
            if (fScore != rhs.fScore) {
                return fScore > rhs.fScore;
            }
            return gScore < rhs.gScore;
        }
    }

} // namespace unisalento
