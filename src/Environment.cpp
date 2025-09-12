/**
 * @file Environment.cpp
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

#include "Environment.hpp"
// #include "GridGraph.hpp"
namespace unisalento
{
    Environment::Environment(const std::vector<Agent> &a, std::shared_ptr<MyDirectedGraph> g,
                             std::shared_ptr<PreprocessingData> prepData, std::shared_ptr<Param> p) : agents(a),
                                                                                                      graph(g),
                                                                                                      preprocessingData(
                                                                                                          prepData),
                                                                                                      params(p)
    {
        this->aStar = std::make_unique<AStar>(this);
        // this->constraintDict.resize(a.size());
        int n{static_cast<int>(a.size())};
        this->assignment.resize(n);
        for (int i{0}; i < n; i++)
        {
            this->assignment[i] = i;
        }
    }

    int Environment::size() const
    {
        return static_cast<int>(agents.size());
    }

    void Environment::changeAssignment(const std::vector<int> &ass)
    {
        this->assignment = ass;
    }

    std::vector<State> Environment::getNeighbors(const State &state) const
    {
        std::vector<State> neighbors;
        neighbors.reserve(5);
        State waitAction{state.time + STEP_COST, state.location};
        if (this->stateValid(waitAction))
        {
            neighbors.emplace_back(std::move(waitAction));
        }
        auto candidates{graph->getOutNeighbors(state.location)};
        // for (auto &n : candidates)
        for (auto ei{candidates.first}; ei != candidates.second; ei++)
        {
            State s{state.time + graph->getWeight(ei), graph->getTarget(ei)};
            if (this->stateValid(s) && this->transitionValid(state, s))
            {
                neighbors.emplace_back(std::move(s));
            }
        }
        return neighbors;
    }

    Conflict Environment::isValid(const Solution &solution) const
    {
        const int n{this->size()};
        for (int h{0}; h < n; h++)
        {
            const int P_h{static_cast<int>(solution.at(h).size())};
            for (int k{h + 1}; k < n; k++)
            {
                const int P_k{static_cast<int>(solution.at(k).size())};
                const int maxP{std::max(P_h, P_k)};
                for (int i{0}, t_i{STARTING_TIME}; i < maxP; i++)
                {
                    State s1a{getState(h, solution, i)};
                    State s1b{getState(h, solution, i + 1)};
                    const int tau{this->transitionCost(s1a, s1b)};
                    for (int j{0}, t_j{STARTING_TIME}; j < maxP; j++)
                    {
                        State s2a{getState(k, solution, j)};
                        // check for vertex conflict
                        if (t_i == t_j)
                        {
                            if (s1a.sameLocation(s2a))
                            {
                                return {t_i, h, k, s1a.location};
                            }
                        }
                        State s2b{getState(k, solution, j + 1)};
                        if (i < P_h - 1 && j < P_k - 1)
                        {
                            // check for arc conflict
                            if (s1a.sameLocation(s2b) && s1b.sameLocation(s2a) && !s1a.sameLocation(s1b) &&
                                (std::min(t_i, t_j) + tau - std::max(t_i, t_j) >= 1))
                            {
                                assert(this->transitionCost(s2a, s2b) == tau);
                                return {t_i, t_j, h, k, s1a.location, s1b.location};
                            }
                        }
                        t_j += this->transitionCost(s2a, s2b);
                    }
                    t_i += tau;
                }
            }
        }

        return {}; // Conflict()
    }

#if true
    // FIXME: handle this using a macro UNIT_COST_ARCS ; for unitary arc only

    int Environment::countConflicts(const Solution &solution) const
    {
        int conflictCounter{0};
        const int maxPathLen{computeMaxPathLen(solution)};
        const int n{this->size()};
        for (int t{STARTING_TIME}; t <= maxPathLen; t++)
        {
            for (int i{0}; i < n; i++)
            {
                State s1a{getState(i, solution, t)};
                State s1b{getState(i, solution, t + 1)};
                for (int j{i + 1}; j < n; j++)
                {
                    // vertex conflict
                    State s2a{getState(j, solution, t)};
                    if (s1a.sameLocation(s2a))
                    {
                        ++conflictCounter;
                    }
                    // arc conflict
                    State s2b{getState(j, solution, t + 1)};
                    if (s1a.sameLocation(s2b) && s1b.sameLocation(s2a))
                    {
                        ++conflictCounter;
                    }
                }
            }
        }
        return conflictCounter;
    }

    int Environment::countVertexConflicts(int agentId, const State &state, const Solution &solution) const
    {
        int conflictCounter{0};
        const int n{this->size()};
        for (int i{0}; i < n; i++)
        {
            if (i != agentId && !solution[i].empty())
            {
                State state2{getState(i, solution, state.time)};
                if (state.sameLocation(state2))
                {
                    ++conflictCounter;
                }
            }
        }
        return conflictCounter;
    }

    int Environment::countArcConflicts(int agentId, const State &state1, const State &state2,
                                       const Solution &solution) const
    {
        int numConflicts = 0;
        const int n{this->size()};
        for (int i{0}; i < n; i++)
        {
            if (i != agentId && !solution[i].empty())
            {
                State s2a{getState(i, solution, state1.time)};
                State s2b{getState(i, solution, state2.time)};
                if (state1.sameLocation(s2b) && state2.sameLocation(s2a))
                {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    Conflict Environment::getFirstConflict(const Solution &solution) const
    {
        const int maxPathLen{computeMaxPathLen(solution)};
        const int n{this->size()};
        for (int t{STARTING_TIME}; t <= maxPathLen; t++)
        {

            // vertex conflict
            /*for (int i{0}; i < n; i++)
            {
                State s1{getState(i, solution, t)};
                for (int j{i + 1}; j < n; j++)
                {
                    State s2{getState(j, solution, t)};
                    if (s1.sameLocation(s2))
                    {
                        return {t, i, j, s1.location};
                    }
                }
            }
            // arc conflict
            for (int i{0}; i < n; i++)
            {
                State s1a{getState(i, solution, t)};
                State s1b{getState(i, solution, t + 1)};
                for (int j{i + 1}; j < n; j++)
                {
                    State s2a{getState(j, solution, t)};
                    State s2b{getState(j, solution, t + 1)};
                    if (s1a.sameLocation(s2b) && s1b.sameLocation(s2a))
                    {
                        return {t, i, j, s1a.location, s1b.location};
                    }
                }
            }*/

            for (int i{0}; i < n; i++)
            {
                State s1a{getState(i, solution, t)};
                State s1b{getState(i, solution, t + 1)};
                for (int j{i + 1}; j < n; j++)
                {
                    // vertex conflict
                    State s2a{getState(j, solution, t)};
                    if (s1a.sameLocation(s2a))
                    {
                        return {t, i, j, s1a.location};
                    }
                    // arc conflict
                    State s2b{getState(j, solution, t + 1)};
                    if (s1a.sameLocation(s2b) && s1b.sameLocation(s2a))
                    {
                        return {t, t, i, j, s1a.location, s1b.location};
                    }
                }
            }
        }
        return {}; // Conflict()
    }

    std::vector<Constraints> Environment::createConstraintsFromConflict(const Conflict &conflict) const
    {
        std::vector<Constraints> constraintDict(this->size());
        switch (conflict.type)
        {
        case VERTEX_CONFLICT:
        {
            // DBG << "Found vertex conflict" << std::endl;
            constraintDict[conflict.agent1].vertexConstraints.emplace(conflict.time1, conflict.location1);
            constraintDict[conflict.agent2].vertexConstraints.emplace(conflict.time1, conflict.location1);
        }
        break;
        case EDGE_CONFLICT:
        {
            // DBG << "Found edge conflict" << std::endl;
            constraintDict[conflict.agent1].edgeConstraints.emplace(conflict.time2, conflict.location1,
                                                                    conflict.location2);
            constraintDict[conflict.agent2].edgeConstraints.emplace(conflict.time1, conflict.location2,
                                                                    conflict.location1);
        }
        break;
        }
        return constraintDict;
    }

#else
    Conflict Environment::getFirstConflict(const Solution &solution) const
    {
        // return this->isValid(solution);
        const int n{this->size()};
        for (int h{0}; h < n; h++)
        {
            const int P_h{static_cast<int>(solution.at(h).size())};
            for (int k{h + 1}; k < n; k++)
            {
                // if (h == k)
                //      continue;
                const int P_k{static_cast<int>(solution.at(k).size())};
                const int maxP{std::max(P_h, P_k)};
                for (int i{0}, t_i{STARTING_TIME}, j{0}, t_j{STARTING_TIME}; i < maxP || j < maxP;)
                {
                    State s1a{getState(h, solution, i)};
                    State s2a{getState(k, solution, j)};
                    // vertex conflict
                    if (t_i == t_j)
                    {
                        if (s1a.sameLocation(s2a))
                        {
                            return {t_i, h, k, s1a.location};
                        }
                    }
                    State s1b{getState(h, solution, i + 1)};
                    const int tau_i{this->transitionCost(s1a, s1b)};
                    State s2b{getState(k, solution, j + 1)};
                    const int tau_j{this->transitionCost(s2a, s2b)};
                    if (i < P_h - 1 && j < P_k - 1)
                    {
                        // arc conflict
                        if (s1a.sameLocation(s2b) && s1b.sameLocation(s2a) && !s1a.sameLocation(s1b) && (std::min(t_i, t_j) + tau_i - std::max(t_i, t_j) >= 1))
                        {
                            // assert(this->transitionCost(s2a, s2b) == tau);
                            return {t_i, t_j, h, k, s1a.location, s1b.location};
                        }
                    }
                    int test{t_i + tau_i - t_j - tau_j};
                    if (test <= 0)
                    {
                        t_i += tau_i;
                        ++i;
                    }
                    // else
                    if (test >= 0)
                    {
                        t_j += tau_j;
                        ++j;
                    }
                    /*else
                    {
                        if (i < maxP)
                        {
                            t_i += tau_i;
                            ++i;
                        }
                        if (j < maxP)
                        {
                            t_j += tau_j;
                            ++j;
                        }
                    }*/
                }
            }
        }

        return {}; // Conflict()
    }

    std::vector<Constraints> Environment::createConstraintsFromConflict(const Conflict &conflict) const
    {
        std::vector<Constraints> constraintDict(this->size());
        switch (conflict.type)
        {
        case VERTEX_CONFLICT:
        {
            // DBG << "Found vertex conflict" << std::endl;
            constraintDict[conflict.agent1].vertexConstraints.emplace(conflict.time1, conflict.location1);
            constraintDict[conflict.agent2].vertexConstraints.emplace(conflict.time1, conflict.location1);
        }
        break;
        case EDGE_CONFLICT:
        {
            // DBG << "Found edge conflict" << std::endl;
            int tau{this->graph->getWeight(conflict.location1, conflict.location2)};
            // FIXME: symmetric graph assumption!
            assert(tau == this->graph->getWeight(conflict.location2, conflict.location1));
            // assert(tau == 1);
            for (int t{0}; t < tau; t++)
            {
                // deny arc access during the interval reserved to the other agent
                constraintDict[conflict.agent1].edgeConstraints.emplace(conflict.time2 + t, conflict.location1, conflict.location2);
                constraintDict[conflict.agent2].edgeConstraints.emplace(conflict.time1 + t, conflict.location2, conflict.location1);
            }
        }
        break;
        }
        return constraintDict;
    }
#endif

    State Environment::getState(int agentId, const Solution &solution, int t)
    {
        if (t < static_cast<int>(solution.at(agentId).size()))
        {
            return solution.at(agentId).at(t);
        }
        else
        {
            return solution.at(agentId).back();
        }
    }

    bool Environment::stateValid(const State &state) const
    {
        return constraints->vertexConstraints.find(VertexConstraint{state.time, state.location}) ==
               constraints->vertexConstraints.cend();
    }

    bool Environment::transitionValid(const State &state1, const State &state2) const
    {
        // return constraints->edgeConstraints.find(EdgeConstraint{state1.time, state1.location, state2.location}) == constraints->edgeConstraints.cend();

        bool andExpr{true};
        for (int t{state1.time}; t < state2.time; t++)
        {
            andExpr &= constraints->edgeConstraints.find(EdgeConstraint{t, state1.location, state2.location}) ==
                       constraints->edgeConstraints.cend();
        }
        return andExpr;
    }

    int Environment::admissibleHeuristic(const State &state, int agentId) const
    {
        const int goal{this->agents.at(this->assignment.at(agentId)).goal};
        // int n{this->graph->getColumns()};
        // const int n{(std::static_pointer_cast<GridGraph>(this->graph))->getColumns()};
        // return abs(state.location / n - goal / n) + abs(state.location % n - goal % n);        
        return preprocessingData->quickestCost(state.location,
                                               goal); 
        // return std::static_pointer_cast<GridGraph>(graph)->L1norm(state.location, goal);
    }

    bool Environment::isAtGoal(const State &state, int agentId) const
    {
        const int goal{this->agents.at(this->assignment.at(agentId)).goal};
        return state.location == goal && state.time > freeGoalInstant;
    }

    Solution Environment::computeSolution(const std::vector<Constraints> &constraintDict)
    {
        const int n{this->size()};
        Solution solution(n);
        for (int i{0}; i < n; i++)
        {
            this->constraints = constraintDict.cbegin() + i;
            const int goal{this->agents.at(this->assignment.at(i)).goal};
            int fgi{-1};
            for (const auto &vc : constraints->vertexConstraints)
            {
                if (vc.location == goal && vc.time > fgi)
                {
                    fgi = vc.time;
                }
            }
            this->freeGoalInstant = fgi;
            std::list<State> localSolution{aStar->search(i, solution)};
            if (localSolution.empty())
            {
                throw "Empty path detected";
            }
            solution.at(i) = std::vector(std::make_move_iterator(localSolution.cbegin()),
                                         std::make_move_iterator(localSolution.cend()));
        }
        return solution;
    }

    int Environment::computeSolutionCost(const Solution &solution) const
    {
        int total{STARTING_TIME};
        for (const auto &path : solution)
        {
            int s{static_cast<int>(path.size()) - 1};
            for (int i{0}; i < s; i++)
            {
                total += this->transitionCost(path[i], path[i + 1]); // path.size() - 1
            }
        }
        return total; 
    }

    int Environment::computeMakespan(const Solution &solution) const
    {
        int makespan{0};
        for (const auto &path : solution)
        {
            int s{static_cast<int>(path.size()) - 1};
            int total{STARTING_TIME};
            for (int i{0}; i < s; i++)
            {
                total += this->transitionCost(path[i], path[i + 1]); // path.size() - 1
            }
            if (total > makespan)
            {
                makespan = s;
            }
        }
        return makespan;
    }

    int Environment::computeMaxPathLen(const Solution &solution)
    {
        int maxPathLen{0};
        for (const auto &path : solution)
        {
            int s{static_cast<int>(path.size()) - 1};
            if (s > maxPathLen)
            {
                maxPathLen = s;
            }
        }
        return maxPathLen;
    }

    std::vector<int> Environment::pathLens(const Solution &solution)
    {
        std::vector<int> l(solution.size());
        int h{0};
        for (const auto &path : solution)
        {
            int s{static_cast<int>(path.size()) - 1};
            l[h++] = s;
        }
        return l;
    }

    int Environment::getStart(int agentId) const
    {
        return agents.at(agentId).start;
    }

    int Environment::getGoal(int agentId) const
    {
        return agents.at(agentId).goal;
    }

    int Environment::getAssignedGoal(int agentId) const
    {
        return agents.at(assignment.at(agentId)).goal;
    }

    std::shared_ptr<PreprocessingData> Environment::getPreprocessingData() const
    {
        return preprocessingData;
    }

    int Environment::transitionCost(const State &state1, const State &state2) const
    {
        return state1.sameLocation(state2) ? STEP_COST : graph->getWeight(state1.location, state2.location);
    }

    std::shared_ptr<MyDirectedGraph> Environment::getLayoutGraph() const
    {
        return graph;
    }

    std::shared_ptr<Param> Environment::getParams() const
    {
        return params;
    }

    void Environment::dot(std::ostream &os) const
    {
        os << "graph G1 {" << std::endl;
        os << "fontsize=10;" << std::endl;
        os << "margin=0;" << std::endl;
        os << "layout=fdp;" << std::endl;
        const int n{this->size()};
        mymap<int, int> start2agent(n);
        mymap<int, int> goal2agent(n);
        for (int h{0}; h < n; h++)
        {
            start2agent.emplace(getStart(h), h);
            goal2agent.emplace(getGoal(h), h);
        }
        for (auto it{boost::vertices(*graph)}; it.first != it.second; ++it.first)
        {
            auto u{*(it.first)};
            auto [x, y]{graph->getXY(u)};
            os << "v" << u << "[shape=circle, label=\"" << u << "\", pos=\"" << x << "," << y << "!\"";
            auto itS{start2agent.find(u)};
            if (itS != start2agent.cend())
            {
                os << " color=" << COLORS[itS->second] << " style=\"filled, rounded\"";
            }
            auto itG{goal2agent.find(u)};
            if (itG != goal2agent.cend())
            {
                os << " color=" << COLORS[itG->second];
            }
            os << "]" << std::endl;
        }
        for (auto it{boost::edges(*graph)}; it.first != it.second; ++it.first)
        {
            auto e{*(it.first)};
            if (e.m_source < e.m_target)
            {
                continue;
            }
            os << "v" << e.m_source << " -- v" << e.m_target << " [color=black]" << std::endl;
        }
        os << "}" << std::endl;
    }
}
