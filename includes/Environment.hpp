/**
 * @file Environment.hpp
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

#include "Agent.hpp"
#include "Constraints.hpp"
#include "AStar.hpp"
#include "State.hpp"
#include "Conflict.hpp"
#include "HighLevelNode.hpp"
#include "BoostDirectedGraph.hpp"
#include "PreprocessingData.hpp"
#include "Param.hpp"

namespace unisalento {
    class AStar;

    /**
     * @brief Contiene una serie di informazioni utili relativamente all'istanza di MAPF.
     *
     */
    class Environment {
        friend class AStar;

    private:
        std::vector<Agent> agents;
        std::shared_ptr<MyDirectedGraph> graph{nullptr};
        std::shared_ptr<PreprocessingData> preprocessingData{nullptr};
        std::vector<Constraints>::const_iterator constraints;
        std::unique_ptr<AStar> aStar{nullptr};
        std::vector<int> assignment;
        int freeGoalInstant{0};
        std::shared_ptr<Param> params{nullptr};

    public:
        Environment(const std::vector<Agent> &agents, std::shared_ptr<MyDirectedGraph> graph,
                    std::shared_ptr<PreprocessingData> prepData, std::shared_ptr<Param> params);

        int size() const;

        int getStart(int agentId) const;

        int getGoal(int agentId) const;

        int getAssignedGoal(int agentId) const;

        std::vector<State> getNeighbors(const State &state) const;

        int countConflicts(const Solution &solution) const;

        int countVertexConflicts(int agentId, const State &state, const Solution &solution) const;

        int countArcConflicts(int agentId, const State &state1, const State &state2, const Solution &solution) const;

        Conflict getFirstConflict(const Solution &solution) const;

        Conflict isValid(const Solution &solution) const;

        std::vector<Constraints> createConstraintsFromConflict(const Conflict &conflict) const;

        static State getState(int agentId, const Solution &solution, int t);

        bool stateValid(const State &state) const;

        bool transitionValid(const State &state1, const State &state2) const;

        int admissibleHeuristic(const State &state, int agentId) const;

        bool isAtGoal(const State &state, int agentId) const;

        Solution computeSolution(const std::vector<Constraints> &constraintDict);

        int computeSolutionCost(const Solution &solution) const;

        int computeMakespan(const Solution &solution) const;

        static int computeMaxPathLen(const Solution &solution);

        static std::vector<int> pathLens(const Solution &solution);

        void changeAssignment(const std::vector<int> &ass);

        std::shared_ptr<PreprocessingData> getPreprocessingData() const;

        int transitionCost(const State &state1, const State &state2) const;

        std::shared_ptr<MyDirectedGraph> getLayoutGraph() const;

        std::shared_ptr<Param> getParams() const;

        void dot(std::ostream &os) const;
    };
}
