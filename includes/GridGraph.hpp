/**
 * @file GridGraph.hpp
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

#include "commons.hpp"
#include "BoostDirectedGraph.hpp"

namespace unisalento
{
    /**
     * @brief Specializza il grafo di layout per topologie a griglia.
     *
     */
    class GridGraph : public MyDirectedGraph
    {
    private:
        Dim2D dimension;          // m x n
        std::vector<bool> layout; // (m+2) * (n+2) positions
        std::vector<int> loc2id;
        std::vector<int> id2loc;

        void set(const int i, const int j, const bool value);

    public:
        GridGraph(const Dim2D &d, const myset<int> &obstacles);

        /*virtual const std::vector<AdjacentNode> getOutNeighbors(const int u) const;*/

        bool at(const int i, const int j) const;
        /*
        int row(int id) const;
        int rrow(int id) const;
        int column(int id) const;
        int ccolumn(int id) const;
        */

        /**
         * @brief The L1 Distance, also called the Cityblock Distance, the Manhattan Distance, the Taxicab Distance, the Rectilinear Distance or the Snake Distance,
         * does not go in straight lines but in blocks. L1 distance measures city block distance: distance along straight lines only.
         *
         * @param u
         * @param v
         * @return int
         */
        int L1norm(const int u, const int v) const;

        Dim2D getDimension() const;

        virtual Dim2D getXY(const int u) const override;

        int getId(Dim2D xy) const;

        int getId(const int x, const int y) const;

        int getId(const int xy) const;

        int getRows() const;

        int getColumns() const;

        /**
         * @brief Gives the number of bifurcated vertices in the graph
         *
         * Lemma 2: A connected graph containing k bifurcated vertices is a solvable workspace of k + 1 agents.
         * from Liao et al. (2023), "A decoupling method for solving the multi-agent path finding problem"
         */
        int countBifurcatedVertices() const;

        void dot(std::ostream &os) const;
    };

}
