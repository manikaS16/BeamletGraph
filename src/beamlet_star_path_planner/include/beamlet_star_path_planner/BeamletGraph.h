/*
 * BeamletGraph.h
 *
 *  Created on: Jun 12, 2010
 *      Author: Kocher
 */

#ifndef BEAMLETGRAPH_H_
#define BEAMLETGRAPH_H_

#include "beamlet_star_path_planner/Beamlet.h"
#include "beamlet_star_path_planner/Point.h"
#include "beamlet_star_path_planner/quadtree.h"
#include "beamlet_star_path_planner/DyadicObject.h"

#include <vector>
#include <map>

class BeamletGraph {
private:
    std::vector<Beamlet*> beamlets;

public:
	BeamletGraph();

    void generateBeamlets(const std::map<std::vector<int>, Point*>& points);

    int getBeamletCount() const;

    const std::vector<Beamlet*>& getBeamlets() const;

    // bool isBeamletFeasible(std::vector<std::vector<int>>& grid, Point* p1, Point* p2);

    void printBeamlets() const;

	~BeamletGraph();
};

#endif /* BEAMLETGRAPH_H_ */