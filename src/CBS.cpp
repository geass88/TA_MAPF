/**
 * @file CBS.cpp
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

#include "CBS.hpp"
#include <boost/heap/d_ary_heap.hpp>

namespace unisalento
{

    Solution CBS::search()
    {
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                         boost::heap::mutable_<true>>
            openSet;
        const int CBS_STRATEGY{env->getParams()->getInt("CBS_STRATEGY")};
        // myset<HighLevelNode, Hash> closedSet;
        HighLevelNode start;
        start.constraintDict.resize(env->size());
        try
        {
            start.solution = env->computeSolution(start.constraintDict);
        }
        catch (const char *s)
        {
            ERR << "Error: " << s << std::endl;
            return {};
        }
        start.cost = env->computeSolutionCost(start.solution);
        openSet.push(start);
        while (!openSet.empty())
        {
            HighLevelNode P{openSet.top()};
            // closedSet.insert(P);
            openSet.pop();

            Conflict conflict{env->getFirstConflict(P.solution)};
            if (conflict.absent())
            {
                // DBG << "Solution found with cost " << env->computeSolutionCost(P.solution) << std::endl;
                return P.solution;
            }
            std::vector<Constraints> constraintDict{env->createConstraintsFromConflict(conflict)};
            for (int i{0}; i < env->size(); i++)
            {
                const auto &d{constraintDict[i]};
                if (d.empty())
                    continue;
                HighLevelNode newNode{P}; // deep copy
                // assert(!newNode.constraintDict[i].overlap(d));
                newNode.constraintDict[i] += d;
                try
                {
                    newNode.solution = env->computeSolution(newNode.constraintDict);
                }
                catch (const char *s)
                {
                    // DBG << "Warn: " << s << std::endl;
                    continue;
                }
                switch (CBS_STRATEGY)
                {
                case 0:
                {
#ifdef OPTIMIZE_MAKESPAN
                    newNode.cost = env->computeMakespan(newNode.solution);
#else
                    newNode.cost = env->computeSolutionCost(newNode.solution);
#endif
                    break;
                }
                case 1:
                {
                    int conflicts{env->countConflicts(newNode.solution)};
                    if (conflicts == 0)
                        return newNode.solution;
                    newNode.cost = conflicts;
                    break;
                }
                case 2:
                {
                    int conflicts{env->countConflicts(newNode.solution)};
                    if (conflicts == 0)
                        return newNode.solution;
                    newNode.cost = 1'000'000 * conflicts + env->computeSolutionCost(newNode.solution);
                    break;
                }
                default:
                {
                    throw "CBS_STRATEGY parameter not set";
                }
                }
                // ending condition
                // if (closedSet.find(newNode) == closedSet.cend())
                //  openSet.insert(newNode);
                openSet.push(newNode);
            }
        }

        return {};
    }
}
