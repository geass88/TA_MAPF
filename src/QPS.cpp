/**
 * @file QPS.cpp
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

#include "QPS.hpp"
#include <ilopl/iloopl.h>

namespace unisalento
{
    Solution QPS::search()
    {
        const int DISJUNCTIVE_CONSTRAINTS_MODELING{env->getParams()->getInt("DISJUNCTIVE_CONSTRAINTS_MODELING")};
        int n{this->env->size()};
        std::vector<std::list<int>> paths(n);
        for (int i{0}; i < n; ++i)
        {
            paths[i] = this->env->getPreprocessingData()->quickestPath(env->getStart(i), env->getAssignedGoal(i));
        }

        IloEnv iloEnv;
        iloEnv.setOut(ILOG_STREAM);
        iloEnv.setError(ILOG_STREAM);
        IloModel model{iloEnv};
        IloArray<IloNumVarArray> t{iloEnv, n};
        IloExpr objectiveExpr{iloEnv};
        for (int h{0}; h < n; h++)
        {
            int P{static_cast<int>(paths[h].size())};
            t[h] = IloNumVarArray{iloEnv, P, 0, IloInfinity};
            // t[h][0] = IloNumVar(iloEnv, 0, 0, IloNumVar::Float, "t_0_0");
            // objectiveExpr += t[h][0];
            for (int i{1}; i < P; i++)
            {
                // t[h][i] = IloNumVar(iloEnv, 0, IloInfinity, IloNumVar::Float, ("t_" + std::to_string(h) + "_" + std::to_string(i)).c_str());
                objectiveExpr += t[h][i];
                model.add(IloRange{t[h][i] - t[h][i - 1] >= 1}); // conjunctive constraint
            }
            model.add(IloRange{t[h][0] == 0});
        }
        for (int h{0}; h < n; h++)
        {
            for (int k{h + 1}; k < n; ++k)
            {
                int i{0};
                for (auto it1{paths[h].cbegin()}; it1 != paths[h].cend(); ++it1, ++i)
                {
                    bool notLastI{std::next(it1) != paths[h].cend()};
                    int j{0};
                    for (auto it2{paths[k].cbegin()}; it2 != paths[k].cend(); ++it2, ++j)
                    {
                        bool notLastJ{std::next(it2) != paths[k].cend()};
                        if (*it1 == *it2)
                        {
                            // vertex conflict
                            if (notLastI && notLastJ)
                            {
                                switch (DISJUNCTIVE_CONSTRAINTS_MODELING)
                                {
                                case 1:
                                {
                                    IloOr orConstraint{iloEnv};
                                    orConstraint.add(IloRange{t[h][i] - t[k][j + 1] >= 0});
                                    orConstraint.add(IloRange{t[k][j] - t[h][i + 1] >= 0});
                                    model.add(orConstraint);
                                    break;
                                }
                                case 2:
                                {
                                    model.add(IloRange{t[h][i] - t[k][j + 1] >= 0} !=
                                              IloRange{t[k][j] - t[h][i + 1] >= 0});
                                    break;
                                }
                                default:
                                    IloBoolVar y{iloEnv};
                                    model.add(IloRange{t[h][i] - t[k][j + 1] + BIG_M * y >= 0});
                                    model.add(IloRange{t[k][j] - t[h][i + 1] + BIG_M * (1. - y) >= 0});
                                }
                            }
                            else if (notLastJ)
                            {
                                // i is the last for agent h
                                model.add(IloRange{t[h][i] - t[k][j + 1] >= 0});
                            }
                            else if (notLastI)
                            {
                                // j is the last for agent k
                                model.add(IloRange{t[k][j] - t[h][i + 1] >= 0});
                            }
                            else
                            {
                                // i and j are both lasts
                                ERR << "Unavoidable vertex collision detected. No solution!" << std::endl;
                                return {};
                            }
                        }
                        if (notLastI && notLastJ)
                        {
                            if (*(std::next(it1)) == *it2 && *(std::next(it2)) == *it1)
                            {
                                // arc conflict
                                bool notSecondLastI{std::next(it1, 2) != paths[h].cend()};
                                bool notSecondLastJ{std::next(it2, 2) != paths[k].cend()};
                                if (notSecondLastI && notSecondLastJ)
                                {
                                    switch (DISJUNCTIVE_CONSTRAINTS_MODELING)
                                    {
                                    case 1:
                                    {
                                        IloOr orConstraint{iloEnv};
                                        orConstraint.add(IloRange{t[h][i] - t[k][j + 2] >= 0});
                                        orConstraint.add(IloRange{t[k][j] - t[h][i + 2] >= 0});
                                        model.add(orConstraint);
                                        break;
                                    }
                                    case 2:
                                    {
                                        model.add(IloRange{t[h][i] - t[k][j + 2] >= 0} !=
                                                  IloRange{t[k][j] - t[h][i + 2] >= 0});
                                        break;
                                    }
                                    default:
                                    {
                                        IloBoolVar y{iloEnv};
                                        model.add(IloRange{t[h][i] - t[k][j + 2] + BIG_M * y >= 0});
                                        model.add(IloRange{t[k][j] - t[h][i + 2] + BIG_M * (1. - y) >= 0});
                                    }
                                    }
                                }
                                else if (notSecondLastJ)
                                {
                                    // i is the second last for agent h
                                    model.add(IloRange{t[h][i] - t[k][j + 2] >= 0});
                                }
                                else if (notSecondLastI)
                                {
                                    // j is the second last for agent k
                                    model.add(IloRange{t[k][j] - t[h][i + 2] >= 0});
                                }
                                else
                                {
                                    // i and j are both second lasts
                                    ERR << "Unavoidable arc collision detected. No solution!" << std::endl;
                                    return {};
                                }
                            }
                        }
                    }
                }
            }
        }

        IloObjective obj{iloEnv, objectiveExpr, IloObjective::Minimize};
        model.add(obj);
        objectiveExpr.end();

        // Create the solver object
        IloCplex cplex{model};
        // Export model to file (useful for debugging!)
        // cplex.exportModel("model3.lp");

        bool solved;
        try
        {
            // Try to solve with CPLEX (and hope it does not raise an exception!)
            solved = cplex.solve();
        }
        catch (const IloException &e)
        {
            ERR << "\n\nCPLEX Raised an exception:" << std::endl;
            ERR << e << std::endl;
            iloEnv.end();
            throw;
        }
        Solution sol;
        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            DBG << "\nCplex success!" << std::endl;
            DBG << "\tStatus: " << cplex.getStatus() << std::endl;
            DBG << "\tObjective value: " << cplex.getObjValue() << std::endl;
            // cplex.writeSolution("assets/sol.txt");
            //  LOG << "\nOBJECTIVE: " << std::fixed << std::setprecision(2) << opl.getCplex().getObjValue() << std::endl;
            for (int h{0}; h < n; h++)
            {
                auto it{paths[h].cbegin()};
                std::vector<State> path;
                int time{0};
                int P{static_cast<int>(paths[h].size())};
                for (int i{1}; i < P; i++, it++)
                {
                    // auto &var = t[h][i];
                    // int val{static_cast<int>(cplex.isExtracted(var) ? std::lround(cplex.getValue(var)) : 0)};
                    int val{static_cast<int>(std::lround(cplex.getValue(t[h][i])))};
                    for (; time < val; ++time)
                    {
                        path.emplace_back(time, *it);
                    }
                }
                path.emplace_back(time, *it);
                sol.emplace_back(std::move(path));
            }
        }
        else
        {
            ERR << "No solution!" << std::endl;
            return {};
        }
        cplex.clear();
        cplex.end();
        model.end();
        iloEnv.end();
        /*if (status != CplexStatus)
        {
            DBG << "Final status: " << status << std::endl;
            return {};
        }*/
        return sol;
    }
}
