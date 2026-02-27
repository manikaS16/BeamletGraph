#include "rclcpp/rclcpp.hpp"

#include "beamlet_star_path_planner/quadtree.h"
#include "beamlet_star_path_planner/DyadicObject.h"
#include "beamlet_star_path_planner/Point.h"

#include <cmath>

QuadTree::QuadTree(const std::vector<std::vector<int>>& grid, int rows, int cols, int smax, double alpha)
    : grid(grid), rows(rows), cols(cols), smax(smax), alpha(alpha), root(nullptr)
{
}

DyadicObject* QuadTree::getQTRoot() const{
    return root;
}

int QuadTree::countObstacles(DyadicObject* node) const
{
    Key k = node->GetKey();

    int scale = k.scale;
    int size = static_cast<int>(pow(2, scale));

    int x = k.position[0];
    int y = k.position[1];
    int rowStart = (x-1)*size;
    int colStart = (y-1)*size;

    int count = 0;
    for (int i = rowStart; i < rowStart + size; ++i)
        for (int j = colStart; j < colStart + size; ++j)
            if (i >= 0 && i < rows && j >= 0 && j < cols)
                if (grid[i][j] == 1)
                    ++count;

    return count;
}

DyadicObject* QuadTree::buildQuadTree(int scale, int x, int y)
{
    DyadicObject* node = new DyadicObject(scale,x,y);

    if(root == nullptr)
        root = node;

    node->setBoundaries(rows, cols);

    int totalObstacles = countObstacles(node);
    node->SetObstacleCount(totalObstacles);

    int totalCells = pow(2, 2*scale);

    if(totalObstacles == 0) {
        node->SetColor("WHITE");
        leaves.push_back(node);
    } else if(totalObstacles == totalCells) {
        node->SetColor("BLACK");
        leaves.push_back(node);
    } else{
        node->SetColor("GRAY");

        if(scale == 0)
        {
            leaves.push_back(node);
            return node;
        }

        double lowerLimit = 1 + alpha*(pow(2,2*scale-1) - 1);
        double upperLimit = totalCells - 1 - alpha*(pow(2,2*scale-1) - 1);

        if((totalObstacles < lowerLimit || totalObstacles > upperLimit))
        {
            leaves.push_back(node);
            return node;
        }

        else{
            node->setChild(0, buildQuadTree(scale-1, 2*x-1, 2*y-1));
            node->setChild(1, buildQuadTree(scale-1, 2*x-1, 2*y));
            node->setChild(2, buildQuadTree(scale-1, 2*x, 2*y-1));
            node->setChild(3, buildQuadTree(scale-1, 2*x, 2*y));
        }
    }

    return node;
}

void QuadTree::printFormalDyadicForm(DyadicObject* node) const
{
    if(node == nullptr) 
        return;

    auto logger = rclcpp::get_logger("Formal Dyadic Form");

    Key k = node->GetKey();
    int scale = k.scale;
    int x = k.position[0];
    int y = k.position[1];

    if(node->isLeaf()) {
        RCLCPP_INFO(logger, "q(%d;%d,%d) = %s",
                    scale,
                    x,
                    y,
                    node->GetColor().c_str());
        return;
    } else{
        std::string line = "q(" + std::to_string(scale) + ";" +
                        std::to_string(x) + "," +
                        std::to_string(y) + ") = " +
                        "q(" + std::to_string(scale-1) + ";" +
                        std::to_string(2*x-1) + "," +
                        std::to_string(2*y-1) + ") ∪ " +
                        "q(" + std::to_string(scale-1) + ";" +
                        std::to_string(2*x-1) + "," +
                        std::to_string(2*y) + ") ∪ " +
                        "q(" + std::to_string(scale-1) + ";" +
                        std::to_string(2*x) + "," +
                        std::to_string(2*y-1) + ") ∪ " +
                        "q(" + std::to_string(scale-1) + ";" +
                        std::to_string(2*x) + "," +
                        std::to_string(2*y) + ")";

        RCLCPP_INFO(logger, "%s", line.c_str());

        for(int i=0; i<4; i++)
            printFormalDyadicForm(node->getChild(i));
    }
}

const std::vector<DyadicObject*>& QuadTree::getLeaves() const { return leaves; }

void QuadTree::printLeafBoundaries() const
{
    auto logger = rclcpp::get_logger("Leaf Boundaries");

    for(const auto* node : leaves)
    {
        int** b = node->getBoundaries();

        if (!b)
            continue;

        Key k = node->GetKey();

        RCLCPP_INFO(logger,
            "Leaf q(%d;%d,%d,%s) boundaries:",
            k.scale,
            k.position[0],
            k.position[1],
            node->GetColor().c_str());

        RCLCPP_INFO(logger, "  TL: (%d,%d)", b[0][0], b[0][1]);
        RCLCPP_INFO(logger, "  TR: (%d,%d)", b[1][0], b[1][1]);
        RCLCPP_INFO(logger, "  BL: (%d,%d)", b[2][0], b[2][1]);
        RCLCPP_INFO(logger, "  BR: (%d,%d)", b[3][0], b[3][1]);
    }
}

// void QuadTree::generateUniqueLeafPoints()
// {
//     auto logger = rclcpp::get_logger("Leaf Points");

//     for(const auto* node : leaves)
//     {
//         int** b = node->getBoundaries();
//         if(!b) continue;

//         Key k = node->GetKey();

//         RCLCPP_INFO(logger,
//             "Processing leaf q(%d;%d,%d)",
//             k.scale,
//             k.position[0],
//             k.position[1]);

//         for(int i = 0; i < 4; ++i)
//         {
//             std::vector<int> coords = {b[i][0], b[i][1]};
//             Point::getOrCreatePoint(uniquePoints, coords);

//             RCLCPP_INFO(logger,
//                 "  Corner %d -> (%d,%d)",
//                 i,
//                 coords[0],
//                 coords[1]);
//         }
//     }
// }




void QuadTree::generateUniqueLeafPoints()
{
    auto logger = rclcpp::get_logger("Leaf Points");

    for (const auto* node : leaves)
    {
        int** b = node->getBoundaries();
        if (!b) continue;

        Key k = node->GetKey();

        RCLCPP_INFO(logger,
            "Processing leaf q(%d;%d,%d)",
            k.scale,
            k.position[0],
            k.position[1]);

        int rowStart = b[0][0];  // TL row
        int colStart = b[0][1];  // TL col
        int rowEnd   = b[2][0];  // BL row
        int colEnd   = b[1][1];  // TR col

        // ---- LEFT EDGE ----
        for (int r = rowStart; r <= rowEnd; ++r)
        {
            std::vector<int> coords = {r, colStart};
            Point::getOrCreatePoint(uniquePoints, coords);
        }

        // ---- RIGHT EDGE ----
        for (int r = rowStart; r <= rowEnd; ++r)
        {
            std::vector<int> coords = {r, colEnd};
            Point::getOrCreatePoint(uniquePoints, coords);
        }

        // ---- TOP EDGE ----
        for (int c = colStart; c <= colEnd; ++c)
        {
            std::vector<int> coords = {rowStart, c};
            Point::getOrCreatePoint(uniquePoints, coords);
        }

        // ---- BOTTOM EDGE ----
        for (int c = colStart; c <= colEnd; ++c)
        {
            std::vector<int> coords = {rowEnd, c};
            Point::getOrCreatePoint(uniquePoints, coords);
        }
    }

    RCLCPP_INFO(logger, "Total unique boundary points: %ld", uniquePoints.size());
}

void QuadTree::printUniquePoints(std::map<std::vector<int>, Point*>& pointMap) const{
    auto logger = rclcpp::get_logger("Unique Points");

    int idx = 0;
    for (const auto& kv : pointMap)
    {
        const std::vector<int>& coords = kv.first;

        if (coords.size() >= 2)
        {
            RCLCPP_INFO(logger,
                        "Point %d: (%d, %d)",
                        idx++,
                        coords[0],
                        coords[1]);
        }
    }
}

std::map<std::vector<int>, Point*> QuadTree::getUniqueLeafPoints() const{
    return uniquePoints;
}