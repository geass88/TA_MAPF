/**
 * @file TimeExpandedGraph.hpp
 * @author Tommaso Adamo (tommaso.adamo@unisalento.it)
 * @brief
 *
 * @copyright Copyright (c) 2024 - University of Salento
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

#include "Environment.hpp"
#include "AbstractTimeExpandedGraph.hpp"

namespace unisalento
{
    /**
     * @brief Un'implementazione di AbstractTimeExpandedGraph in cui vengono memorizzati solo i nodi tempo-espansi.
     * I metodi per i neighbors esaminano il layout fisico riducendo il memory footprint a discapito di un leggero aumento dell'onere computazionale.
     *
     */
    class TimeExpandedGraph : public AbstractTimeExpandedGraph
    {
    public:
        TimeExpandedGraph(const std::shared_ptr<MyDirectedGraph> g);

        virtual std::vector<int> outNeighbors(int u) const override;

        virtual std::vector<int> inNeighbors(int u) const override;

        virtual std::pair<mymap<int, int>::iterator, bool> insert(int i, int t) override;

        virtual void timeExpand() override;

        // virtual void timeExpand2() override;
    };
} // namespace unisalento
