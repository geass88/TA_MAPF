/**
 * @file main.cpp
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

// #include "includes/Environment.hpp"
#include "includes/CBS.hpp"
#include "includes/GridGraph.hpp"
#include "includes/QPS.hpp"
#include "includes/InstanceDAO.hpp"
#include "includes/MCNF.hpp"
#include <thread>
#include <filesystem>
#include <boost/graph/graphviz.hpp>
using namespace unisalento;

void aborter(int seconds, const std::function<void()> &abortAction)
{
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    abortAction();
}

int main(int argc, const char **argv)
{
    DBG << "Registered working dir: " << WDIR << std::endl;
    std::string op{"mapf"};
    std::string folder{WDIR "data/M8/"};
    std::string instance{"M8_agents2_ex35.yaml"};
    std::string algo{"mcnf"};
    std::string outFile{"out.yaml"};
    std::string prepFile{""}; // "skip" to avoid, "" for in-memory
    Param params{argc == 1 ? WDIR "params.txt" : argv[1]};

    if (argc == 0)
    {        
        LOG << "Usage: \n./build/TA_MAPF paramFile\n"
            << std::endl;
        LOG
            << "Required keys: <op=prep|vis|mapf> <folder> <instance> <algo=qps|cbs|mcnf> <outFile> <prepFile>"
            << std::endl;
        LOG << "Available operations:" << std::endl;
        LOG << "prep: to preprocess an instance" << std::endl;
        LOG << "vis: to visualize a solution" << std::endl;
        LOG << "mapf: to solve the MAPF problem using the initial task assignment" << std::endl;
        LOG << "Available MAPF algorithms:" << std::endl;
        LOG << "CBS: for conflict-based search" << std::endl;
        LOG << "QPS: for quickest path search (fixed path + wait)" << std::endl;
        LOG << "MCNF: for multicommodity network flow on a partial time-expanded network" << std::endl;
        LOG << std::endl;
        return EXIT_FAILURE;
    }
    else
    {
        if (std::filesystem::exists(argv[1]))
        {
            op = params.getValue("op");
            folder = params.getValue("folder");
            instance = params.getValue("instance");
            algo = params.getValue("algo");
            outFile = params.getValue("outFile");
            prepFile = params.getValue("prepFile");
            DBG << "Fetched parameters: \n"
                << params << std::endl;
        }
        else
        {
            op = argv[1];
            folder = argv[2];
            instance = argv[3];
            algo = argv[4];
            outFile = argv[5];
            prepFile = argv[6];
            return EXIT_FAILURE;
        }
    }
    std::string preCommand{params.getValue("pre_command")};
    for (size_t pos{0u}; (pos = preCommand.find("#")) != std::string::npos;)
    {
        preCommand.replace(pos, 1, " ");
    }
    LOG << "Running precommand: " << preCommand << std::endl;
    system(preCommand.c_str());
    LOG << "Current command: \n./build/TA_MAPF \"" << op << "\" \"" << folder << "\" \"" << instance << "\" \"" << algo
        << "\" \"" << outFile << "\" \"" << prepFile << "\"\n"
        << std::endl;
    DBG << "Debug is " << (unisalento::DEBUG == 1 ? "on" : "off") << std::endl;
    DBG << "Timeout is " << params.getInt("TIMEOUT") << " seconds and CBS_STRATEGY is " << params.getInt("CBS_STRATEGY")
        << std::endl;
    DBG << "Using " << params.getInt("NUM_THREADS") << " threads" << std::endl;
#ifdef OPTIMIZE_MAKESPAN
    DBG << "Optimizing makespan" << std::endl;
#else
    DBG << "Optimizing flowtime" << std::endl;
#endif
    Environment env{readInstance(params, folder.data() + instance, prepFile)};
    if (op == "prep")
    {
        return EXIT_SUCCESS;
    }
    if (op == "vis")
    {
        std::string cmd;
        DBG << "Viewing solution: " << outFile << std::endl;

        system((cmd + "python3 visualize.py --video $(date +%Y_%m_%d_%H_%M_%S).mp4 " + folder.data() + instance.data() +
                " " + outFile.data())
                   .c_str());
        return EXIT_SUCCESS;
    }
    std::unique_ptr<MAPFPolicy> policy{nullptr};
    if (algo == "cbs")
    {
        policy = std::make_unique<CBS>(&env);
    }
    else if (algo == "qps")
    {
        policy = std::make_unique<QPS>(&env);
    }
    else if (algo == "mcnf")
    {
        policy = std::make_unique<MCNF>(&env);
    }
    else
    {
        ERR << "Unknown algorithm" << std::endl;
        return EXIT_FAILURE;
    }

    if (op == "mapf")
    {
        DBG << "Starting search with " << algo << " ..." << std::endl;
        std::function<void()> functor = [&policy]
        {
            DBG << "Timeout" << std::endl;
            policy->stop();
        };
        std::thread stopThread(aborter, params.getInt("TIMEOUT"), std::cref(functor));
        auto t1{NOW};
        auto solution{policy->search()};
        auto t2{NOW};
        stopThread.detach();
        if (solution.empty())
        {
            ERR << "No solution" << std::endl;
            LOG << "Runtime: " << ELAPSED_MS(t1, t2) << " ms" << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            int ub{env.computeSolutionCost(solution)};
            LOG << "Solution found with flowtime= " << ub << " and makespan= " << env.computeMakespan(solution)
                << std::endl;
            LOG << "Runtime: " << ELAPSED_MS(t1, t2) << " ms" << std::endl;
            LOG << "Individual costs: ";
            printCollection(LOG_STREAM, env.pathLens(solution));
            Conflict c{env.isValid(solution)};
            if (!c.absent())
            {
                ERR << "Invalid solution" << std::endl;
            }
            std::ofstream fout{outFile.data()};
            policy->exportPlan(fout, solution);
            fout.close();
        }
        return EXIT_SUCCESS;
    }

    return EXIT_SUCCESS;
}
