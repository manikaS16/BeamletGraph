#ifndef POINT_H_
#define POINT_H_

#include <vector>
#include <list>
#include <algorithm>
// #include "Beamlet.h"
// #include "Cluster.h"
// #include "BeamletGraph.h"

#include <map>

class Beamlet;
// class Cluster;

class Point
{

private:
	std::vector<int> coordinates;
	std::list<Beamlet*> beamletList;

	// Cluster *parentCluster;
	std::list<Point*> pairPointList;
	bool pairPointsComputed;

public:

	Point();
	Point(const std::vector<int> & coordinates);
	// Point(const std::vector<int> & coordinates, Cluster *parentCluster);

	void
	AddBeamlet(Beamlet* beamlet);

	void
	RemoveBeamlet(const std::vector<int> & coordinates);

	std::list<Beamlet*>
	GetBeamletList() const;

	std::vector<int>
	GetCoordinates() const;

	void
	AddPairPoints(Point *pairPoint);

	void
	AddPairPoints(std::list<Point*> const& pairPoints);

	void
	SetPairPointsComputed(bool value);

	//TODO: test GetPairPoints function
	// std::list<Point*> *
	// GetPairPoints(BeamletGraph *beamletGraph);

    static void getOrCreatePoint(std::map<std::vector<int>, Point*>& pointMap, std::vector<int> coordinates);

	// friend std::ostream& operator<<(std::ostream& output, const Point& point);

    void printUniquePoints(std::map<std::vector<int>, Point*>& pointMap) const;

	~Point(void);

};

#endif /* POINT_H_ */
