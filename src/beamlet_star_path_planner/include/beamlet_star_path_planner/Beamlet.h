#ifndef BEAMLET_H_
#define BEAMLET_H_

#include "beamlet_star_path_planner/Point.h"

#include <vector>

class Point;
class BeamletGraph;

class Beamlet
{

private:
	Point *point1;
	Point *point2;

	std::list<Beamlet*> neighborBeamletList;
    bool neighborBeamletsComputed;

public:
	Beamlet();

	Beamlet(Point* point1, Point* point2);

	Point*
	GetPoint1() const
	{	return this->point1;};

	Point*
	GetPoint2() const
	{	return this->point2;};

	void
	SetPoint1(Point *point1)
	{this->point1 = point1;}

	void
	SetPoint2(Point *point2)
	{this->point2 = point2;}

	//TODO: test GetNeighborBeamlet function

	std::list<Beamlet*>
	GetNeighborBeamlets();

	double getLength() const;
	
	// friend std::ostream& operator<<(std::ostream & output, const Beamlet & beamlet);

	~Beamlet();

};

#endif /* BEAMLET_H_ */