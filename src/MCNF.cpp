/**
 * @file MCNF.cpp
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
#include "MCNF.hpp"
#include "TimeExpandedGraph.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <forward_list>
// #define EXPORT_MCNF_DAG
// #define OPTIMIZE_MAKESPAN
// #define INITIAL_EXPAND

namespace unisalento
{
    MCNF::MCNF(Environment *const _env)
        : MAPFPolicy(_env), numAgents(_env->size()), numVertices(_env->getLayoutGraph()->getNumVertices()),
          numThreads(_env->getParams()->getInt("NUM_THREADS")),
          onlyFirstSolution(_env->getParams()->getBool("FIRST_SOLUTION")),
          fastForwardIteration(_env->getParams()->getInt("FAST_FORWARD_IT")),
          quickestPaths(_env->getPreprocessingData())
    {
        iloEnv.setOut(ILOG_STREAM);
        iloEnv.setError(ILOG_STREAM);
        prepare();
    }

    MCNF::~MCNF()
    {
        destroyVariables();
        iloEnv.end();
    }

    void MCNF::destroyVariables()
    {
        xMap.clear();
        xMap.resize(numAgents);
    }

    void MCNF::prepare()
    {
        delta.clear();
        S.clear();
        qpMakespan.clear();
        horizon = 0;
        // long flowTime{0l};
        flowTime = 0l;
        delta.resize(numAgents * numVertices);
        S.resize(numAgents);
        qpMakespan.resize(numAgents);
        // omega.clear();
        // omega.resize(numAgents);
        dag = std::make_unique<TimeExpandedGraph>(env->getLayoutGraph());
        int maxDelta{0};
#ifdef INITIAL_EXPAND
        int minHorizon{INF};
        int maxHorizon{0};
        for (int h{0}; h < numAgents; ++h)
        {
            const int start{env->getStart(h)};
            const int goal{env->getGoal(h)};
            const int temp{dag->quickestPaths->quickestCost(start, goal)};
            if (temp < minHorizon)
            {
                minHorizon = temp;
            }
            if (temp > maxHorizon)
            {
                maxHorizon = temp;
            }
        }
        fastForwardIteration += maxHorizon - minHorizon;
        DBG << "Initial expand enabled, minHorizon= " << minHorizon << " and maxHorizon= " << maxHorizon << ", fast-forward to iteration " << fastForwardIteration << std::endl;
#endif
        for (int h{0}; h < numAgents; ++h)
        {
            const int start{env->getStart(h)};
            const int goal{env->getGoal(h)};
            const int temp{quickestPaths->quickestCost(start, goal)};
            if (temp == INF)
            {
                ERR << "Attenzione: il goal non è raggiungibile a partire dallo start per l'agente " << h << std::endl;
                throw "Goal not reachable";
            }
            qpMakespan[h] = temp;
            flowTime += temp;
            if (horizon < temp)
            {
                horizon = temp;
            }
            for (int i{0}; i < numVertices; ++i)
            {
                const int q1{quickestPaths->quickestCost(start, i)};
                const int q2{quickestPaths->quickestCost(i, goal)};
                if (q1 < INF && q2 < INF)
                {
#ifdef INITIAL_EXPAND
                    const int offset{qpMakespan[h] - minHorizon};
                    const int d{q1 + q2 - temp + offset};
#else
                    const int d{q1 + q2 - temp};
#endif
                    assert(d >= 0);
                    delta[h * numVertices + i] = d;
                    S[h][d].emplace(i);
                    if (d > maxDelta)
                    {
                        maxDelta = d;
                    }
                }
                else
                {
                    delta[h * numVertices + i] = INF;
                    S[h][INF].emplace(i);
                }
            }
            goal2agent.emplace(goal, h);
            if (start == goal)
            {
                ERR << "Attenzione: il goal coincide con lo start per l'agente " << h << std::endl;
                // exit(2);
            }
        }
        DBG << "Individual quickest costs: ";
        printCollection(DBG_STREAM, qpMakespan);
        DBG << "Initial flowtime= " << flowTime << " and makespan= " << horizon << std::endl;
        DBG << "Max \u0394= " << maxDelta << std::endl;

        std::vector<int> offset(numAgents, 0);
        for (int h{0}; h < numAgents; ++h)
        {
            int min{INF};
            for (int k{0}; k < numAgents; ++k)
            {
                if (k != h)
                {
                    for (int i : S[h][0])
                    {
                        const int d{delta[k * numVertices + i]};
                        if (d < min)
                        {
                            min = d;
                        }
                    }
                }
            }
            if (0 < min && min < INF)
            {
                offset[h] = min - 1;
            }
        }
        DBG << "Individual \u0394 offsets: ";
        printCollection(DBG_STREAM, offset);        
    }

    const IloBoolVar &MCNF::x(int h, int id1, int id2)
    {
        auto &l{xMap[h][id1]};
        auto it{l.find(id2)};
        if (it == l.end()) // se non è presente crea la variabile decisionale
        {
            it = l.emplace(id2, iloEnv).first;
        }
        return it->second;
    }

    void MCNF::forEachVertex(int iteration, int h, const std::function<void(int)> &functor)
    {
        for (int t{0}; t <= dag->horizon; t++) // per ogni istante nell'orizzonte temporale corrente
        {
            for (auto [i, w] : dag->timeExpandedNodes.at(t))
            {
                const int u{dag->getVertex(i, t)}; // vertice tempo-espanso (i, t)
                if (agentReachNode(iteration, h, dag->getLocation(u),
                                   dag->getTime(u)))
                { // se il vertice tempo-espanso u è raggiunto da h
                    functor(u);
                }
            }
        }
    }

    void MCNF::forEachArc(int iteration, int h, const std::function<void(int, int)> &functor)
    {
        const int goal{env->getGoal(h)};       // destinazione dell'agente
        for (int t{0}; t <= dag->horizon; t++) // per ogni istante nell'orizzonte temporale corrente
        {
            for (auto [i, w] : dag->timeExpandedNodes.at(t))
            {
                const int u{dag->getVertex(i, t)}; // vertice tempo-espanso (i, t)
                if (agentReachNode(iteration, h, dag->getLocation(u), dag->getTime(u)))
                {                                      // se il vertice tempo-espanso u è raggiunto da h
                    for (int v : dag->outNeighbors(u)) // check reachable u
                    {                                  // se il vertice tempo-espanso v è raggiunto da h
                        if (agentReachNode(iteration, h, dag->getLocation(v), dag->getTime(v)))
                        {
                            functor(u, v);
                        }
                    }
                    if (goal == i) // se il vertice i è il goal di h
                    {
                        const int v{dag->getVertex(i, CF_HORIZON_UB)};
                        functor(u, v);
                    }
                }
            }
        }
    }

    bool MCNF::agentReachNode(int iteration, int h, int i, int t) const
    {
        // return true; // UNLOCK THIS IN ORDER TO MAKE EVERY T.E. NODE REACHABLE BY ANY AGENT
        const int start{env->getStart(h)};
        const int goal{env->getGoal(h)};
        assert(dag->exists(i, t));
        const int q1{quickestPaths->quickestCost(start, i)};
        const int q2{quickestPaths->quickestCost(i, goal)};
        // iteration >= q1 + q2 - qpMakespan[h];
        // if (h != 1) { iteration = 0; }
        return iteration >= delta[h * numVertices + i] &&
               t >= q1 && t <= qpMakespan[h] + iteration - q2;
    }

    void MCNF::timeExpansion() const
    {
        dag->timeExpand();
    }

    void MCNF::spaceExpansion(int iteration)
    {
        for (int h{0}; h < numAgents; ++h)
        {
            // if (iteration != 0 && h != 1) continue;
            const int start{env->getStart(h)};
            for (int i : S[h][iteration])
            {
                const int t{quickestPaths->quickestCost(start, i)};
                assert(t < INF);
                dag->insert(i, t);
                // omega[h].emplace(r); // add all S[h][iteration] to omega[h];
            }
        }
    }

    void MCNF::stop()
    {
        // DBG << "Timeout" << std::endl;
        iloAborter.abort(); // https://www.ibm.com/support/pages/how-terminate-cplex-optimization
    }

    Solution MCNF::search()
    {
        int iteration{0}; // \Delta in our report

#ifdef OPTIMIZE_MAKESPAN
        long initialLB{INF};
        const long makespanLB{horizon};
        for (int h{0}; h < numAgents; ++h)
        {
            if (qpMakespan[h] < initialLB)
            {
                initialLB = qpMakespan[h];
            }
        }
#else
        long initialLB{flowTime};
#endif
        while (iteration < fastForwardIteration) // fast-forward to iteration
        {
            this->spaceExpansion(iteration); // aggiungi i vertici S_h^{iteration} in dag->timeExpandedNodes
            this->timeExpansion();
            iteration++;
            horizon++;
        }
        destroyVariables();
        MIPStart mipStart; // https://www.ibm.com/docs/en/icos/22.1.1?topic=mip-starting-from-solution-starts
        bool foundUB{false};
        while (true)
        {
            // destroyVariables();
            LOG << "Iteration " << iteration << " horizon= " << horizon << std::endl;
            auto bt1{NOW};
            IloModel model{iloEnv};
            this->spaceExpansion(iteration); // aggiungi i vertici S_h^{iteration} in timeExpandedNodes
                                             // assert(dag->horizon == horizon);
#ifdef EXPORT_MCNF_DAG
            {
                std::string name{"dag"};
                std::ofstream fout{name + std::to_string(iteration) + ".dot"};
                dag->dot(fout);
                fout.close();
            }
#endif
            // DBG << *dag << std::endl;
            IloExpr objective{iloEnv};                               // espressione della funzione obiettivo
            std::vector<std::list<int>> reachedGoalTimes(numAgents); // per ogni agente h lista ordinata (DESC) degli istanti t raggiunti da h e tali che (d_h, t) \in V^T (utile per i vincoli di sosta a tempo indeterminato in d_h)
            std::vector<IloExpr> goalFlow(0);                        // LHS dei vincoli sul flusso finale nei sink
            goalFlow.reserve(numAgents);                             // riservo numAgents posizioni
            std::vector<IloExpr> z(0);                               // vettore delle durate delle singole rotte
            z.reserve(numAgents);
            for (int h{0}; h < numAgents; ++h)
            {
                IloExpr startFlow{iloEnv}; // LHS dei vincoli sul flusso iniziale dai source
                const int start{env->getStart(h)};
                const int u{dag->getVertex(start, 0)}; // vertice tempo-espanso (o_h, 0)
                for (int v : dag->outNeighbors(u))
                {
                    assert(1 == dag->getTime(v));
                    if (agentReachNode(iteration, h, dag->getLocation(v), dag->getTime(v))) // se il vertice tempo-espanso v è raggiungibile dall'agente h
                    {
                        const IloBoolVar &var{x(h, u, v)};
                        startFlow += var;
                    }
                }
                const int goal{env->getGoal(h)};
                if (start == goal) // gestisci il caso in cui l'origine e la destinazione dall'agente coincidono
                {
                    const int v{dag->getVertex(goal, CF_HORIZON_UB)}; // vertice tempo-espanso (d_h, F)
                    const IloBoolVar &var{x(h, u, v)};
                    startFlow += var;
                }
                model.add(IloRange{startFlow == 1.}); // aggiungo il vincolo sul flusso iniziale dall'agente h
                goalFlow.emplace_back(iloEnv);        // inizializzo il vettore di espressioni
                z.emplace_back(iloEnv);
            }

            // a questo punto itero sugli archi nel grafo tempo-espanso

            for (int t{0}; t <= horizon; t++) // per ogni istante nell'orizzonte temporale corrente
            {
                std::forward_list<Dim2D> conflictingArcs;        // lista di archi tempo-espansi su cui può avvenire un conflitto di tipo swap: ((i, t), (j, t+1)) \in A^T such that ((j, t), (i, t+1)) \in A^T
                mymap<int, mymap<int, IloExpr>> sum1;            // addendi nel LHS dei vincoli di swap conflict: \sum\limits_{h = 1, \dots, numAgents: \: a \in R_h^\Delta} x_a^h
                for (auto [i, w] : dag->timeExpandedNodes.at(t)) // per ogni vertice fisico tale che (i, t) \in V^T
                {
                    const int u{dag->getVertex(i, t)}; // vertice tempo-espanso (i, t)
                    const std::vector<int> outNeighbors{dag->outNeighbors(u)};
                    const std::vector<int> inNeighbors{dag->inNeighbors(u)};
                    for (int v : outNeighbors) // verifica se esiste il corrispondente arco tempo-espanso di swap ed in caso affermativo annota l'arco in conflictingArcs
                    {
                        const int j{dag->getLocation(v)};
                        if (i < j && dag->exists(i, dag->getTime(v)) && dag->exists(j, t))
                        {
                            conflictingArcs.emplace_front(u, v);
                        }
                    }
                    const auto position{
                        goal2agent.find(i)};           // verifica se il vertice i è la destinazione di qualche agente
                    IloExpr vertexConflict{iloEnv};    // LHS dei vincoli per la gestione dei conflitti di vertice
                    for (int h{0}; h < numAgents; ++h) // per ogni agente
                    {
                        const int start{env->getStart(h)};      // origine dell'agente
                        const int goal{env->getGoal(h)};        // destinazione dell'agente
                        if (agentReachNode(iteration, h, i, t)) // se il vertice tempo-espanso (i, t) è raggiungibile dall'agente h
                        {
                            IloExpr balanceFlow{iloEnv}; // LHS dei vincoli di bilanciamento del flusso
                            if (goal == i)               // se il vertice corrente (i, t) è la destinazione dell'agente h, gestisci l'arco dummy verso il vertice (d_h, F)
                            {
                                // assert(t >= qpMakespan[h]);
                                const int v{dag->getVertex(goal, CF_HORIZON_UB)};
                                const IloBoolVar &var{x(h, u, v)};
                                // objective += var;
                                // z[h] += var;
                                goalFlow[h] += var;
                                vertexConflict += var;
                                if (goal != start ||
                                    t > 0) // gestisci il caso in cui l'origine e la destinazione dall'agente coincidono
                                {
                                    balanceFlow += var;
                                }
                                sum1[u].emplace(v, IloExpr{iloEnv}).first->second += var;
                                /*if (position != goal2agent.cend()) // se (i, t) è il goal di qualcuno, aggiorna la sommatoria
                                {
                                    *sum2 += var;
                                }*/
                                reachedGoalTimes[h].push_front(t);
                            }
                            for (int v : outNeighbors) // per ogni arco uscente
                            {
                                assert(t + 1 == dag->getTime(v));
                                if (agentReachNode(iteration, h, dag->getLocation(v), dag->getTime(v))) // se il vertice tempo-espanso v è raggiungibile dall'agente h
                                {
                                    const IloBoolVar &var{x(h, u, v)};
                                    objective += var;
                                    z[h] += var;
                                    vertexConflict += var;
                                    if (i != start || t > 0)
                                    {
                                        balanceFlow += var;
                                    }
                                    auto &l{sum1[u]};
                                    auto it{l.find(v)};
                                    if (it == l.end()) // se non è presente inserisce l'arco
                                    {
                                        it = l.emplace(v, IloExpr{iloEnv}).first;
                                    }
                                    it->second += var;
                                    /*if (position != goal2agent.cend()) // se (i, t) è il goal di qualcuno, aggiorna la sommatoria
                                    {
                                        *sum2 += var;
                                    }*/
                                }
                            }
                            for (int v : inNeighbors) // per ogni arco entrante
                            {
                                if (agentReachNode(iteration, h, dag->getLocation(v), dag->getTime(v))) // se il vertice tempo-espanso v è raggiungibile dall'agente h
                                {
                                    const IloBoolVar &var{x(h, v, u)};
                                    if (i != start || t > 0)
                                    {
                                        balanceFlow -= var;
                                    }
                                }
                            }
                            if (!balanceFlow.isConstant())
                            {
                                model.add(IloRange{balanceFlow == 0.});
                            }
                        }
                    }
                    //                    assert(!vertexConflict.isConstant());
                    if (position != goal2agent.cend())
                    {
                        const auto &ref{reachedGoalTimes[position->second]};
                        if (ref.empty() || t <= ref.back())
                        {
                            model.add(IloRange{vertexConflict <= 1.});
                        }
                        else
                        { // aggiungo i vincoli per precludere l'accesso ad un vertice dopo il parcheggio a tempo indeterminato
                            auto it{ref.cbegin()};
                            assert(*it <= t);
                            if (*it == t)
                            {
                                ++it;
                            }
                            for (; it != ref.cend(); ++it)
                            {
                                const int goal{env->getGoal(position->second)};
                                const int v1{dag->getVertex(goal, *it)};
                                const int v2{dag->getVertex(goal, CF_HORIZON_UB)};
                                const IloBoolVar &var{x(position->second, v1, v2)};
                                model.add(IloRange{vertexConflict + var <= 1.});
                            }
                        }
                    }
                    else
                    {
                        model.add(IloRange{vertexConflict <= 1.});
                    }
                }
                for (const auto &p : conflictingArcs)
                {
                    const auto [i, t1]{dag->getPair(p.first)};
                    const auto [j, t2]{dag->getPair(p.second)};
                    assert(t1 < t2);
                    const IloExpr &s1{sum1[p.first][p.second]};
                    const IloExpr &s2{sum1[dag->getVertex(j, t1)][dag->getVertex(i, t2)]};
                    if (s1.isValid() && s2.isValid())
                    {
                        model.add(IloRange{s1 + s2 <= 1.});
                    } /*
                     else
                     {
                         ERR << "Invalidity " << s1.isValid() << " " << s2.isValid() << std::endl;
                     }*/
                }
            }
            IloNumVar zVar{iloEnv};

            for (int h{0}; h < numAgents; ++h)
            {
#ifdef OPTIMIZE_MAKESPAN
                model.add(IloRange{z[h] - zVar <= 0});
#endif
                model.add(IloRange{goalFlow[h] == 1.}); // aggiungo il vincolo sul flusso finale dall'agente h
            }
#ifdef OPTIMIZE_MAKESPAN
            IloObjective obj{iloEnv, zVar, IloObjective::Minimize};
#else
            IloObjective obj{iloEnv, objective, IloObjective::Minimize};
            // model.add(IloRange{objective <= initialLB + iteration});
#endif
            model.add(obj);
            // model.add(IloRange{objective >= (initialLB + iteration)});
            objective.end();
            auto bt2{NOW};
            DBG << "Model build time: " << ELAPSED_MS(bt1, bt2) << " ms" << std::endl;
            // solve model
            IloCplex cplex{model};
            DBG << "Model has " << cplex.getNcols() << " variables" << " and " << cplex.getNrows() << " constraints" << std::endl;
            if (foundUB)
            {
                cplex.addMIPStart(mipStart.first, mipStart.second); //, IloCplex::MIPStartEffort::MIPStartNoCheck
            }
            cplex.setOut(ILOG_STREAM);
            cplex.setError(ILOG_STREAM);
            // cplex.setParam(IloCplex::Param::RootAlgorithm, CPX_ALG_NET);
            if constexpr (DEBUG) // https://www.ibm.com/docs/en/icos/22.1.1?topic=mip-progress-reports-interpreting-node-log
            {
                cplex.setParam(IloCplex::Param::MIP::Interval, 100); // how frequently node log lines are printed
                cplex.setParam(IloCplex::Param::MIP::Display, 4);    // display mode
            }
            else
            {
                // cplex.setParam(IloCplex::Param::MIP::Interval, 10'000); // how frequently node log lines are printed
                // cplex.setParam(IloCplex::Param::MIP::Display, 0);       // display mode
            }
            cplex.setParam(IloCplex::Param::Threads, numThreads);
            // cplex.setParam(IloCplex::Param::TimeLimit, 100000);
            cplex.use(iloAborter);
#ifdef EXPORT_MCNF_MODEL
            { //  Export model to file (useful for debugging!)
                std::string name{"mcnf"};
                cplex.exportModel((name + std::to_string(iteration) + ".lp").c_str());
            }
#endif
            bool solved{false};
            try
            {
                // Try to solve with CPLEX (and hope it does not raise an exception!)
                solved = cplex.solve();
            }
            catch (const IloException &e)
            {
                ERR << "\n\nCPLEX Raised an exception:" << std::endl;
                ERR << e << std::endl;
                // iloEnv.end();
                throw;
            }

            // if infeasible expand and continue
            // else optimal solution found, exit
            if (solved)
            {
                LOG << "\nCplex success!" << std::endl;
                LOG << "\tStatus: " << cplex.getStatus() << std::endl;
                LOG << "\tInitial LB: " << initialLB << std::endl;
                LOG << "\tHorizon: " << horizon << std::endl;
#ifdef OPTIMIZE_MAKESPAN
                long lb{std::max(makespanLB, initialLB + iteration + 1)};
#else
                long lb{initialLB + iteration + 1};
#endif
                LOG << "\tLB: " << lb << std::endl;
                long ub{std::lround(cplex.getObjValue())};
                LOG << "\tUB: " << ub << std::endl;
                if (onlyFirstSolution)
                {
                    lb = ub; // termina alla prima soluzione ammissibile
                }
                if (ub - lb < 1)
                {
                    flowTime = ub;
                    Solution solution{buildSolution(cplex)};
                    cplex.end();
                    model.end();
                    return solution;
                }
                else
                {                    
                    ERR << "Optimality gap open: further expansion required" << std::endl;
                    foundUB = true;
                    mipStart = createMIPStart(cplex, iteration);                    
                }
            }
            else
            {
                ERR << "No solution!" << std::endl;
            }
            if (iloAborter.isAborted())
            {
                break;
            }
            this->timeExpansion();
            iteration++;
            horizon++;
            cplex.end();
            model.end();
            auto bt3{NOW};
            DBG << "Model solve time: " << ELAPSED_MS(bt2, bt3) << " ms" << std::endl;
            if constexpr (HEURISTIC)
            {
                DBG << "Heuristic enabled, exiting ..." << std::endl;
                break;
            }
        }
        return {};
    }

    MIPStart MCNF::createMIPStart(const IloCplex &cplex, int iteration)
    {
        IloNumVarArray variables{iloEnv};
        IloNumArray values{iloEnv};
        for (int h{0}; h < numAgents; ++h)
        {
            int u{dag->getVertex(env->getStart(h), 0)};
            bool found;
            do
            {
                found = false;
                for (int v : dag->outNeighbors(u))
                {
                    if (agentReachNode(iteration, h, dag->getLocation(v), dag->getTime(v)))
                    {
                        const IloBoolVar &var{xMap[h][u][v]};
                        if (cplex.getValue(var) > .5)
                        {
                            variables.add(var);
                            values.add(1.);
                            u = v;
                            found = true;
                            break;
                        }
                    }
                }
            } while (found);
            variables.add(xMap[h][u][dag->getVertex(dag->getLocation(u), CF_HORIZON_UB)]);
            values.add(1.);
        }
        return {std::move(variables), std::move(values)};
    }

    Solution MCNF::buildSolution(const IloCplex &cplex)
    {
        Solution solution(numAgents);
        for (int h{0}; h < numAgents; ++h)
        {

            int u{dag->getVertex(env->getStart(h), 0)};
            solution[h].emplace_back(dag->getTime(u), dag->getLocation(u));
            bool found;
            do
            {
                found = false;
                for (int v : dag->outNeighbors(u))
                {
                    const IloBoolVar &var{x(h, u, v)};
                    if (cplex.isExtracted(var) && cplex.getValue(var) > .5)
                    // if (xMap[h][u][v] != nullptr && cplex.getValue(*xMap[h][u][v]) > .5)
                    {
                        solution[h].emplace_back(dag->getTime(v), dag->getLocation(v));
                        u = v;
                        found = true;
                        break;
                    }
                }
            } while (found);
            assert(cplex.getValue(xMap[h][u][dag->getVertex(dag->getLocation(u), CF_HORIZON_UB)]) > .5);
        }
#ifdef EXPORT_MCNF_DAG
        {
            std::ofstream fout{"solution.dot"};
            dag->dot(fout, solution);
            fout.close();
        }
#endif
        return solution;
    }

}
