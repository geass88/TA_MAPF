/**
 * @file InstanceDAO.cpp
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

#include "InstanceDAO.hpp"
#include <filesystem>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <thread>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

namespace unisalento
{
    Solution readSolution(std::string_view solFile, std::shared_ptr<GridGraph> graph)
    {
        std::ifstream fin{solFile.data()};
        std::string line;
        int t, x, y, counter{0};
        Solution sol;
        bool schedule{false};
        while (std::getline(fin, line))
        {
            if (line.find("schedule") != line.npos)
            {
                schedule = true;
            }
            if (!schedule)
                continue;
            if (line.find("agent") != line.npos)
            {
                sol.emplace_back();
            }
            else if (line.find("t") != line.npos)
            {
                auto p{line.find(":")};
                t = std::stoi(line.substr(p + 1));
                counter++;
            }
            else if (line.find("x") != line.npos)
            {
                auto p{line.find(":")};
                x = std::stoi(line.substr(p + 1));
                counter++;
            }
            else if (line.find("y") != line.npos)
            {
                auto p{line.find(":")};
                y = std::stoi(line.substr(p + 1));
                counter++;
            }
            if (counter == 3)
            {
                sol.back().emplace_back(t, graph->getId(x, y));
                counter = 0;
            }
        }
        fin.close();
        return sol;
    }

#if false
    Environment readMap(std::string_view problemInstance)
    {
        std::ifstream fin(problemInstance.data());
        std::string token;
        Dim2D dimension;
        fin >> token >> token;
        fin >> token >> dimension.second; // height : # of rows/lines
        fin >> token >> dimension.first;  // width: # of columns
        fin >> token;
        myset<int> obstacles;
        for (int i{0}; i < dimension.second; i++) {
            fin >> token;
            for (int j{0}; j < dimension.first; j++) {
                char c{token.at(j)};
                if (c == '@' || c == 'O' || c == 'T') {
                    obstacles.emplace(i * dimension.second + j);
                }
            }
        }

        fin.close();
        std::vector<Agent> agents;
        std::shared_ptr<GridGraph> graph{std::make_shared<GridGraph>(dimension, obstacles)};
        std::shared_ptr<PreprocessingData> data{std::make_shared<PreprocessingData>(graph)};
        return {agents, graph, data};
    }
#endif

    Environment readInstance(const Param &params, std::string_view problemInstance, std::string_view preprocessing)
    {
        if (std::filesystem::exists(problemInstance.data()))
        {
            DBG << "Found instance file" << std::endl;
        }
        else
        {
            ERR << "Instance file not found" << std::endl;
            throw "Instance file not found";
        }
        std::ifstream fin(problemInstance.data());
        std::string line;
        Dim2D dimension;
        while (std::getline(fin, line))
        {
            if (line.find("dimensions") != line.npos)
            {
                auto p1{line.find("[")};
                auto p2{line.find(",")};
                auto p3{line.find("]")};
                int x{std::stoi(line.substr(p1 + 1, p2 - p1))};
                int y{std::stoi(line.substr(p2 + 1, p3 - p1))};
                if (x * x * y * y > INT_MAX)
                {
                    ERR << "Insufficient indices" << std::endl;
                    throw "Insufficient indices";
                }
                dimension = {x, y};
            }
        }
        fin.clear();
        fin.seekg(0, std::ios::beg);

        std::vector<Agent> agents;
        myset<int> obstacles;
        std::string name;
        std::vector<bool> checkOverlap(dimension.first * dimension.second, false);
        bool foundOverlap{false};
        bool foundObstacles{false};
        for (int start, goal, counter{0}; std::getline(fin, line);)
        {
            if (line.find("goal") != line.npos)
            {
                auto p1{line.find("[")};
                auto p2{line.find(",")};
                auto p3{line.find("]")};
                int x{std::stoi(line.substr(p1 + 1, p2 - p1))};
                int y{std::stoi(line.substr(p2 + 1, p3 - p1))};
                goal = x * dimension.second + y;
                counter++;
            }
            else if (line.find("start") != line.npos)
            {
                auto p1{line.find("[")};
                auto p2{line.find(",")};
                auto p3{line.find("]")};
                int x{std::stoi(line.substr(p1 + 1, p2 - p1))};
                int y{std::stoi(line.substr(p2 + 1, p3 - p1))};
                start = x * dimension.second + y;
                counter++;
            }
            else if (line.find("name") != line.npos)
            {
                auto p1{line.find(":")};
                name = line.substr(p1 + 1, line.size() - p1);
                counter++;
            }
            else if (line.find("obstacles") != line.npos)
            {
                foundObstacles = true;
            }
            else if (foundObstacles && line.find("[") != line.npos)
            {
                auto p1{line.find("[")};
                auto p2{line.find(",")};
                auto p3{line.find("]")};
                int x{std::stoi(line.substr(p1 + 1, p2 - p1))};
                int y{std::stoi(line.substr(p2 + 1, p3 - p1))};
                obstacles.emplace(x * dimension.second + y);
            }
            if (counter == 3)
            {
                agents.emplace_back(name, start, goal);
                counter = 0;
                if (checkOverlap[start] || checkOverlap[goal])
                {
                    foundOverlap = true;
                }
                else
                {
                    checkOverlap[start] = true;
                    checkOverlap[goal] = true;
                }
            }
        }
        fin.close();
        DBG << "Agents: " << agents.size() << std::endl;
        DBG << "Dimension: " << dimension.first << " x " << dimension.second << std::endl;
        DBG << "Obstacles: " << obstacles.size() << std::endl;
        DBG << (foundOverlap ? "Found endpoints overlap, search potentially infeasible" : "Endpoints don't overlap")
            << std::endl;
        myset<int> endpoints;
        for (int i{0}; i < static_cast<int>(agents.size()); i++)
        {
            int start{agents[i].getStart()};
            if (endpoints.find(start) == endpoints.cend())
            {
                endpoints.emplace(start);
            }
            else
            {
                DBG << "Found agent " << i << " start " << start << std::endl;
            }
            int goal{agents[i].getGoal()};
            if (endpoints.find(goal) == endpoints.cend())
            {
                endpoints.emplace(goal);
            }
            else
            {
                DBG << "Found agent " << i << " goal " << goal << std::endl;
            }
        }

        std::shared_ptr<GridGraph> graph{std::make_shared<GridGraph>(dimension, obstacles)};
        // graph->describe(DBG_STREAM);
        // std::ofstream fout("grafo.dot");
        // graph->dot(fout);
        // fout.close();

        /*std::ofstream fout("assets/instance.dat");
        fout << "m=30;" << std::endl;
        fout << "n="<<graph->getNumVertices()<<";" << std::endl;
        fout << "k="<<agents.size() <<";" << std::endl;
        fout << "source=[";
        for (int i{0}; i < static_cast<int>(agents.size()); i++)
        {
            fout << agents[i].getStart() << " ";
        }
        fout << "];\ntarget=[" << std::endl;
        for (int i{0}; i < static_cast<int>(agents.size()); i++)
        {
            fout << agents[i].getGoal() << " ";
        }
        fout << "];\nG=";
        graph->printLayout(fout);
        fout << ";" << std::endl;
        fout.close();*/

        // remap origin-destination xy pairs to graph_id pairs
        for (int i{0}; i < static_cast<int>(agents.size()); i++)
        {
            agents[i].setStart(graph->getId(agents[i].getStart()));
            agents[i].setGoal(graph->getId(agents[i].getGoal()));
        }

        int numVertices{graph->getNumVertices()};
        DBG << "Graph has " << numVertices << " vertices and " << graph->getNumArcs() << " arcs." << std::endl;
        DBG << "Graph has k=" << graph->countBifurcatedVertices() << " bifurcated vertices (solvable with k+1 agents)."
            << std::endl;

        std::shared_ptr<PreprocessingData> data{nullptr};
        if (preprocessing.empty())
        {
            DBG << "Preprocessing reader disabled, running in-memory preprocessing ..." << std::endl;
            data = std::make_shared<PreprocessingData>(graph);
        }
        else if (preprocessing == "skip")
        {
            DBG << "Preprocessing skipped" << std::endl;
        }
        else
        {
            if (std::filesystem::exists(preprocessing.data()))
            {
                DBG << "Found preprocessing file" << std::endl;
                data = std::make_shared<PreprocessingData>(preprocessing, numVertices);
            }
            else
            {
                DBG << "Creating preprocessing file ..." << std::endl;
                data = std::make_shared<PreprocessingData>(graph);
                std::ofstream fout(preprocessing.data());
                fout << (*data);
                fout.close();
            }
        }
        // graph->parallelAllPairsShortestPath();
        // DBG << graph->getWeight(graph->getOutNeighbors(1).first) << std::endl;
        return {agents, graph, data, std::make_shared<Param>(params)};
    }

    void writeRec2Instance(std::string_view folder, std::string_view name, const Environment &env,
                           const Solution &solution)
    {
        const int customStartingTime{STARTING_TIME + 20};
        Solution compSolution(env.size());
        int counter{0};
        std::vector<int> firstIndex(env.size());
        for (int h{0}; h < env.size(); h++)
        {
            firstIndex[h] = counter;
            State p{solution.at(h).front()};
            counter++;
            compSolution.at(h).emplace_back(p);
            for (const auto &s : solution.at(h))
            {
                if (!s.sameLocation(p))
                {
                    counter++;
                    compSolution.at(h).emplace_back(s);
                    p = s;
                }
            }
        }
        std::string s{folder};
        std::ofstream fout{s + name.data()};
        fout << "numVehicles: " << env.size() << std::endl;
        fout << "numVertices: " << counter << std::endl;
        fout << "startingTimes: ";
        for (int h{0}; h < env.size(); h++)
        {
            fout << customStartingTime << " ";
        }
        fout << std::endl;
        fout << "paths:" << std::endl;

        for (int h{0}; h < env.size(); h++)
        {
            const auto &path{compSolution.at(h)};
            int index{firstIndex[h]};
            fout << path.size() << " ";
            for (const auto &state : path)
            {
                std::ignore = state;
                fout << index++ << " ";
            }
            fout << std::endl;
        }
        fout << "i j w" << std::endl;
        for (int h{0}; h < env.size(); h++)
        {
            const auto &pathH{compSolution.at(h)};
            const int pH{static_cast<int>(pathH.size())};
            for (int i{0}; i < pH; i++)
            {
                if (i < pH - 1)
                {
                    // conjunctive arcs
                    int cost{env.transitionCost(pathH.at(i), pathH.at(i + 1))};
                    assert(cost == 1);
                    assert(!pathH.at(i).sameLocation(pathH.at(i + 1)));
                    fout << (firstIndex[h] + i) << " " << (firstIndex[h] + i + 1) << " " << cost << std::endl;
                }
                // disjunctive arcs
                for (int k{h + 1}; k < env.size(); k++)
                {
                    const auto &pathK{compSolution.at(k)};
                    const int pK{static_cast<int>(pathK.size())};
                    for (int j{0}; j < pK; j++)
                    {
                        if (pathH.at(i).sameLocation(pathK.at(j)))
                        {
                            // vertex conflict
                            int diff{pathH.at(i).getTime() - pathK.at(j).getTime()};
                            if (diff < 0)
                            {
                                assert(i + 1 < pH);
                                int cost{env.transitionCost(pathH.at(i), pathH.at(i + 1))};
                                assert(cost == 1);
                                fout << (firstIndex[h] + i + 1) << " " << (firstIndex[k] + j) << " " << (1 - cost)
                                     << std::endl;
                            }
                            else if (diff > 0)
                            {
                                assert(j + 1 < pK);
                                int cost{env.transitionCost(pathK.at(j), pathK.at(j + 1))};
                                assert(cost == 1);
                                fout << (firstIndex[k] + j + 1) << " " << (firstIndex[h] + i) << " " << (1 - cost)
                                     << std::endl;
                            }
                            else
                            {
                                throw "Invalid plan";
                            }
                        }
                        // arc conflict should be just included!
                        /*if (i < pH - 1 && j < pK - 1)
                        {
                            if (pathH.at(i).sameLocation(pathK.at(j + 1)) && pathH.at(i + 1).sameLocation(pathK.at(j))) // arc conflict
                            {
                                if (pathH.at(i).getTime() < pathK.at(j).getTime())
                                {
                                    fout << firstIndex[h] + i + 2 << " " << firstIndex[k] + j << " " << 1 - env.transitionCost(pathH.at(i), pathH.at(i + 1)) << std::endl;
                                }
                                else
                                {
                                    fout << firstIndex[k] + j + 2 << " " << firstIndex[h] + i << " " << 1 - env.transitionCost(pathK.at(j), pathK.at(j + 1)) << std::endl;
                                }
                            }
                        }*/
                    }
                }
            }
            // add connection from the dummy source vertex
            fout << counter << " " << firstIndex[h] << " " << customStartingTime << std::endl;
        }
        fout.close();
    }

    void do_session(tcp::socket socket, const Environment &env, const Solution &solution)
    {
        try
        {
            // Construct the stream by moving in the socket
            websocket::stream<tcp::socket> ws{std::move(socket)};

            // Set a decorator to change the Server of the handshake
            ws.set_option(websocket::stream_base::decorator(
                [](websocket::response_type &res)
                {
                    res.set(http::field::server,
                            std::string(BOOST_BEAST_VERSION_STRING) +
                                " websocket-server-sync");
                }));

            // Accept the websocket handshake
            ws.accept();
            long index{0l};
            auto g{env.getLayoutGraph()};
            for (;;)
            {
                // This buffer will hold the incoming message
                beast::flat_buffer buffer;

                // Read a message
                ws.read(buffer);

                // Echo the message back
                ws.text(ws.got_text());
                // ws.write(buffer.data());
                DBG << beast::make_printable(buffer.data()) << std::endl;
                std::string s{""};
                for (int h{0}; h < env.size(); h++)
                {
                    State state{env.getState(h, solution, index / 10)};
                    State nextState{env.getState(h, solution, index / 10 + 1)};
                    auto [x0, y0]{g->getXY(state.getLocation())};
                    auto [x1, y1]{g->getXY(nextState.getLocation())};
                    // s += std::to_string(x0 * 10 + (index % 10) * (x1 - x0)) + "," + std::to_string(y0 * 10 + (index % 10) * (y1 - y0)) + ",";
                    s += std::to_string(y0 * 10 + (index % 10) * (y1 - y0)) + "," +
                         std::to_string(x0 * 10 + (index % 10) * (x1 - x0)) + ",";
                }
                s = s.substr(0, s.length() - 1);
                DBG << s << std::endl;
                ws.write(net::buffer(s));
                index++;
            }
        }
        catch (beast::system_error const &se)
        {
            // This indicates that the session was closed
            if (se.code() != websocket::error::closed)
                ERR << "Error: " << se.code().message() << std::endl;
        }
        catch (std::exception const &e)
        {
            ERR << "Error: " << e.what() << std::endl;
        }
    }

    void runServer(const Environment &env, const Solution &solution)
    {
        auto const address{net::ip::make_address("0.0.0.0")};
        auto const port{8080};
        net::io_context ioc{1};

        // The acceptor receives incoming connections
        tcp::acceptor acceptor{ioc, {address, port}};
        for (;;)
        {
            // This will receive the new connection
            tcp::socket socket{ioc};

            // Block until we get a connection
            acceptor.accept(socket);

            // Launch the session, transferring ownership of the socket
            std::thread(&do_session, std::move(socket), std::cref(env), std::cref(solution)).detach();
        }
    }

} // namespace unisalento
