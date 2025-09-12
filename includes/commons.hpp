/**
 * @file commons.hpp
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

#include <tuple>
#include <list>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <string_view>
#include <fstream>
#include <cassert>
#include <chrono>

#define myset std::unordered_set
#define mymap std::unordered_map
#ifndef WDIR
#define WDIR "/home/tommaso/workspace/E80/TA_MAPF/"
#endif
#ifndef NUM_THREADS
#define NUM_THREADS 4
#endif
// #define ASTAR_CONFLICT_BASED

namespace unisalento
{
    constexpr static const char *COLORS[]{"red", "green", "blue", "cyan", "yellow", "magenta", "navy", "silver",
                                          "orange", "pink", "purple", "brown", "black", "white"};
    constexpr static int BIG_M{1'000};
    constexpr static int INF{10'000'000};
    constexpr static int STEP_COST{1};
    constexpr static int STARTING_TIME{0};

    class State;

    class BoostDirectedGraph;

    typedef std::pair<int, int> Dim2D;
    typedef std::vector<std::vector<State>> Solution;
    typedef BoostDirectedGraph MyDirectedGraph;
    // class DirectedGraph;
    // typedef DirectedGraph MyDirectedGraph;
    constexpr char VERTEX_CONFLICT{1};
    constexpr char EDGE_CONFLICT{2};

    template <typename T>
    std::ostream &
    printCollection(std::ostream &out, const T &collection, std::string_view sep = " ", std::string_view end = "\n")
    {
        for (const auto &item : collection)
        {
            /*if(std::is_same_v<decltype(item), const std::vector<int>&>) {
                printCollection(out, item);
                out<<"texs"<<std::endl;
                continue;
            }*/
            out << item << sep;
        }
        out << end << std::flush;
        return out;
    }

#ifdef NDEBUG
    constexpr bool DEBUG{false};
#else
    constexpr bool DEBUG{true};
#endif
}
#define NOW std::chrono::high_resolution_clock::now()
#define ELAPSED(start, stop) std::chrono::duration_cast<std::chrono::nanoseconds>((stop) - (start)).count()
#define ELAPSED_MS(start, stop) (ELAPSED(start, stop) / 1'000'000)

#ifdef NDEBUG
/*#define DBG \
    if constexpr (true) {} \
    else (std::cout << "\033[0;36m")*/
#define DBG (std::cout << "\033[0;36m")
#define DBG_STREAM (std::cout << "\033[0;36m")
// #define ILOG_STREAM iloEnv.getNullStream()
#define ILOG_STREAM (std::cout << "\033[0;32m")
#else
#define DBG (std::cout << "\033[0;36m")
#define DBG_STREAM (std::cout << "\033[0;36m")
#define ILOG_STREAM (std::cout << "\033[0;32m")
#endif
#define LOG (std::cout << "\033[0m")
#define LOG_STREAM (std::cout << "\033[0m")
#define ERR (std::cout << "\033[1;31m")
#define ERR_STREAM (std::cout << "\033[1;31m")

#ifdef ILO_WINDOWS
#define DIRSEP "\\"
#else
#define DIRSEP "/"
#endif
#ifndef DATADIR
#define DATADIR "opl" DIRSEP
#endif
//".." DIRSEP ".."  DIRSEP ".." DIRSEP ".." DIRSEP "opl" DIRSEP
// ILOSTLBEGIN

/*
# Black        0;30     Dark Gray     1;30
# Red          0;31     Light Red     1;31
# Green        0;32     Light Green   1;32
# Brown/Orange 0;33     Yellow        1;33
# Blue         0;34     Light Blue    1;34
# Purple       0;35     Light Purple  1;35
# Cyan         0;36     Light Cyan    1;36
# Light Gray   0;37     White         1;37
*/
