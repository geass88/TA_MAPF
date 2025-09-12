/**
 * @file MCNF.hpp
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

#include "MAPFPolicy.hpp"
#include "AbstractTimeExpandedGraph.hpp"
#include <ilopl/iloopl.h>
#include <functional>

namespace unisalento {
    typedef std::pair<IloNumVarArray, IloNumArray> MIPStart;

    /**
     * @brief Implementa la strategia di risoluzione del problema di MAPF basata su multi-commodity network flow su rete parzialmente tempo-espansa.
     *
     */
    class MCNF : public MAPFPolicy {
    protected:
        const int numAgents;
        const int numVertices;
        const int numThreads;
        const int onlyFirstSolution;
        int fastForwardIteration;
        std::vector<int> delta;                // numAgents x numVertices matrix
        std::vector<mymap<int, myset<int>>> S; // S_h^{\delta}
        mymap<int, int> goal2agent;            // map each goal vertex to the corresponding agent
        // std::vector<std::set<int>> omega(numAgents);                // union of S_h^{\delta} for any \delta from 0 to current iteration
        std::vector<int> qpMakespan; // makespan of the single-agent quickest path
        long flowTime{0l};           // our objective function
        int horizon{0};
        IloEnv iloEnv;
        IloCplex::Aborter iloAborter{iloEnv};
        std::vector<mymap<int, mymap<int, IloBoolVar>>> xMap; // variables map
        //   std::map<std::tuple<int, int, int>, IloBoolVar> xMap;
        std::unique_ptr<AbstractTimeExpandedGraph> dag;
        const std::shared_ptr<PreprocessingData> quickestPaths;

        const IloBoolVar &x(int h, int id1, int id2);

        void destroyVariables();

        MIPStart createMIPStart(const IloCplex &cplex, int iteration);

        Solution buildSolution(const IloCplex &cplex);

        void prepare();

    public:
        explicit MCNF(Environment *const _env = nullptr);

        virtual ~MCNF() override;

        bool agentReachNode(int iteration, int h, int i, int t) const;

        void forEachVertex(int iteration, int h, const std::function<void(int)> &functor);

        void forEachArc(int iteration, int h, const std::function<void(int, int)> &functor);

        void timeExpansion() const;

        void spaceExpansion(int iteration);

        virtual Solution search() override;

        virtual void stop() override;

    };

}
