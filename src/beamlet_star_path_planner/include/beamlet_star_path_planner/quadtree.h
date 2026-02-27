#ifndef QUADTREE_H
#define QUADTREE_H

#include "beamlet_star_path_planner/DyadicObject.h"
#include "beamlet_star_path_planner/Point.h"

#include <vector>
#include <map>

class QuadTree
{
private:
    std::vector<std::vector<int>> grid;
    int rows;
    int cols;
    int smax;
    double alpha;
    DyadicObject* root;
    std::vector<DyadicObject*> leaves;
    std::map<std::vector<int>, Point*> uniquePoints;

public:

    QuadTree(const std::vector<std::vector<int>>& grid, int rows, int cols, int smax, double alpha);

    DyadicObject* getQTRoot() const;

    int countObstacles(DyadicObject* node) const;
    
    DyadicObject* buildQuadTree(int scale, int x, int y);

    void printFormalDyadicForm(DyadicObject* node) const;

    const std::vector<DyadicObject*>& getLeaves() const;

    void printLeafBoundaries() const;

    void generateUniqueLeafPoints();

    std::map<std::vector<int>, Point*> getUniqueLeafPoints() const;

    void printUniquePoints(std::map<std::vector<int>, Point*>& pointMap) const;
};

#endif