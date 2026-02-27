#include "rclcpp/rclcpp.hpp"
#include "beamlet_star_path_planner/quadtree.h"
#include "beamlet_star_path_planner/Key.h"
#include "beamlet_star_path_planner/DyadicObject.h"
#include "beamlet_star_path_planner/Point.h"
#include "beamlet_star_path_planner/Beamlet.h"
#include "beamlet_star_path_planner/BeamletGraph.h"

#include <vector>

class BeamletStarNode : public rclcpp::Node
{
private:
    std::map<std::vector<int>, Point*> pointMap;

public:
    BeamletStarNode() : Node("beamlet_star_node")
    {
        // Input variables
        declare_parameter("rows", 0);
        declare_parameter("cols", 0);
        declare_parameter("grid", std::vector<int64_t>());
        declare_parameter("smax", 0);
        declare_parameter("alpha", 0.0);

        int rows = get_parameter("rows").as_int();
        int cols = get_parameter("cols").as_int();
        int smax = get_parameter("smax").as_int();
        double alpha = get_parameter("alpha").as_double();

        auto flat = get_parameter("grid").as_integer_array();

        std::vector<std::vector<int>> grid(rows, std::vector<int>(cols));

        int k = 0;
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                grid[i][j] = flat[k++];
            }
        }

        // Print input
        RCLCPP_INFO(get_logger(), "Beamlet* config loaded");
        RCLCPP_INFO(get_logger(), "rows=%d cols=%d smax=%d alpha=%f",
                    rows, cols, smax, alpha);

        RCLCPP_INFO(get_logger(), "Grid:");

        for (int i = 0; i < rows; ++i)
        {
            std::stringstream row_stream;

            for (int j = 0; j < cols; ++j)
            {
                row_stream << grid[i][j] << " ";
            }

            RCLCPP_INFO(get_logger(), "%s", row_stream.str().c_str());
        }

        // Build quadtree and print formal dyadic form
        QuadTree qt(grid, rows, cols, smax, alpha);

        RCLCPP_INFO(get_logger(), "\nBuilding QuadTree. . . .");
        DyadicObject* qtRoot = qt.buildQuadTree(smax, 1, 1); // starting at top-left corner, 1 based indexing
        RCLCPP_INFO(get_logger(), "QuadTree Built!");

        RCLCPP_INFO(get_logger(), "\nPrinting Formal Dyadic Form. . . .");
        qt.printFormalDyadicForm(qtRoot);

        RCLCPP_INFO(get_logger(), "\nPrinting leaves with boundaries");
        qt.printLeafBoundaries();

        const auto& leaves = qt.getLeaves();
        RCLCPP_INFO(get_logger(), "\nTotal leaves: %ld", leaves.size());

        RCLCPP_INFO(get_logger(), "\nPrinting corners of leaves");
        qt.generateUniqueLeafPoints();

        std::map<std::vector<int>, Point*> points = qt.getUniqueLeafPoints();
        RCLCPP_INFO(get_logger(), "\nTotal unique boundary points = %ld", points.size());
        RCLCPP_INFO(get_logger(), "Printing unique points. . . .");
        qt.printUniquePoints(points);

        BeamletGraph graph;
        RCLCPP_INFO(get_logger(), "\nGenerating Beamlets. . . .");
        graph.generateBeamlets(points);
        RCLCPP_INFO(get_logger(), "Beamlets generated!");
        RCLCPP_INFO(get_logger(), "Total beamlets generated: %d", graph.getBeamletCount());
        RCLCPP_INFO(get_logger(), "Displaying beamlets. . . .");
        graph.printBeamlets();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BeamletStarNode>());
    rclcpp::shutdown();
}
