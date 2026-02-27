/*
 * BeamletGraph.cpp
 *
 *  Created on: Jun 12, 2010
 *      Author: Kocher
 */

#include "beamlet_star_path_planner/BeamletGraph.h"
#include "beamlet_star_path_planner/Point.h"
#include "beamlet_star_path_planner/quadtree.h"
#include "beamlet_star_path_planner/DyadicObject.h"

#include "rclcpp/rclcpp.hpp"

#include <map>
#include <fstream>
#include <filesystem>

BeamletGraph::BeamletGraph() {
	// TODO Auto-generated constructor stub

}

void BeamletGraph::generateBeamlets(const std::map<std::vector<int>, Point*>& points){
    std::vector<Point*> pointList;
    pointList.reserve(points.size());

    for (auto& kv : points)
        pointList.push_back(kv.second);

    int n = pointList.size();

    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            // if (isBeamletFeasible(grid, pointList[i], pointList[j]))
            // {
                Beamlet* b = new Beamlet(pointList[i], pointList[j]);
                beamlets.push_back(b);
            // }
        }
    }

    for (Beamlet* b : beamlets)
        b->GetNeighborBeamlets();
}

int BeamletGraph::getBeamletCount() const{
    return beamlets.size();
}

const std::vector<Beamlet*>& BeamletGraph::getBeamlets() const
{
    return beamlets;
}

void BeamletGraph::printBeamlets() const
{
    auto logger = rclcpp::get_logger("BeamletGraph");

    // Get current working directory (PWD)
    std::filesystem::path cwd = std::filesystem::current_path();
    std::filesystem::path filePath = cwd/"BeamletGraph"/"beamlets.txt";

    std::ofstream outFile(filePath);

    if (!outFile.is_open())
    {
        RCLCPP_ERROR(logger, "Failed to open beamlets.txt for writing!");
        return;
    }

    int idx = 0;

    for (const Beamlet* b : beamlets)
    {
        if (!b) continue;

        const auto& p1 = b->GetPoint1()->GetCoordinates();
        const auto& p2 = b->GetPoint2()->GetCoordinates();

        // File
        outFile << "Beamlet "
                << idx << ": ("
                << p1[0] << ","
                << p1[1] << ") -> ("
                << p2[0] << ","
                << p2[1] << ")"
                << std::endl;

        idx++;
    }

    outFile.close();

    RCLCPP_INFO(logger, "Beamlets written to: %s", filePath.string().c_str());
}

BeamletGraph::~BeamletGraph() {
	// TODO Auto-generated destructor stub

    for (Beamlet* b : beamlets)
        delete b;

    beamlets.clear();
}