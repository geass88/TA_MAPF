/**
 * @file AStar.cpp
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

#include "AStar.hpp"
#include "LowLevelNode.hpp"
#include <boost/heap/d_ary_heap.hpp>

namespace unisalento
{
    typedef typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>,
                                             boost::heap::mutable_<true>>
        openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;

    std::list<State> AStar::reconstructPath(const mymap<State, State, Hash> &predMap, const State &current)
    {
        std::list<State> totalPath{current};
        const State *p{&current};
        while (predMap.find(*p) != predMap.cend())
        {
            p = &predMap.at(*p);
            totalPath.push_front(*p);
        }
        return totalPath;
    }

    AStar::AStar(Environment *const _env) : env(_env) {}

    std::list<State> AStar::search(int agentId, const Solution &curSol) const
    {
        const double ASTAR_SUBOPTIMAL_WEIGHT{env->getParams()->getDouble("ASTAR_SUBOPTIMAL_WEIGHT")};
        State initialState{STARTING_TIME, env->getStart(agentId)};
        myset<State, Hash> closedSet;
        mymap<State, fibHeapHandle_t, Hash> pointers;
        mymap<State, State, Hash> cameFrom;
        openSet_t openSet;
        // auto ptr{};
        pointers[initialState] = openSet.push(
            {initialState, env->admissibleHeuristic(initialState, agentId), STARTING_TIME});
        //(*ptr).handle = ptr;
        while (!openSet.empty())
        {
            LowLevelNode current{openSet.top()};
            openSet.pop();
            if (env->isAtGoal(current.state, agentId))
            {
                return reconstructPath(cameFrom, current.state);
            }
            pointers.erase(current.state);
            closedSet.insert(current.state);

            auto neighborsList{env->getNeighbors(current.state)};
            for (const auto &neighbor : neighborsList)
            {
                if (closedSet.find(neighbor) != closedSet.cend())
                {
                    continue;
                }
                int tentative_g_score{env->transitionCost(current.state, neighbor) + current.gScore};
                auto it{pointers.find(neighbor)};
                if (it == pointers.cend())
                {
// discovered new node
#ifdef ASTAR_CONFLICT_BASED
                    int conflicts{current.conflicts + env->countVertexConflicts(agentId, neighbor, curSol) + env->countArcConflicts(agentId, current.state, neighbor, curSol)};
                    int fScore = tentative_g_score + std::lround(env->admissibleHeuristic(neighbor, agentId) * (conflicts > 0 ? ASTAR_SUBOPTIMAL_WEIGHT : 1));
#else
                    int conflicts{0};
                    int fScore = tentative_g_score +
                                 std::lround(env->admissibleHeuristic(neighbor, agentId) * ASTAR_SUBOPTIMAL_WEIGHT);
#endif
                    auto ptr{openSet.push({neighbor, fScore, tentative_g_score, conflicts})};
                    pointers[neighbor] = ptr;
                }
                else
                {
                    auto ptr{it->second};
                    LowLevelNode lln{*ptr};
                    if (tentative_g_score >= lln.gScore)
                    {
                        // We found this node before with a better path
                        continue;
                    }
                    // update f and gScore
                    lln.gScore = tentative_g_score;
#ifdef ASTAR_CONFLICT_BASED
                    int conflicts{current.conflicts + env->countVertexConflicts(agentId, neighbor, curSol) + env->countArcConflicts(agentId, current.state, neighbor, curSol)};
                    lln.fScore = tentative_g_score + std::lround(env->admissibleHeuristic(neighbor, agentId) * (conflicts > 0 ? ASTAR_SUBOPTIMAL_WEIGHT : 1));
#else
                    int conflicts{0};
                    lln.fScore = tentative_g_score +
                                 std::lround(env->admissibleHeuristic(neighbor, agentId) * ASTAR_SUBOPTIMAL_WEIGHT);
#endif
                    lln.conflicts = conflicts;
                    openSet.update(ptr, std::move(lln));
                }
                cameFrom[neighbor] = current.state;
            }
        }
        return {}; // std::list<State>()
    }
}
