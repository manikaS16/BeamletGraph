#include "beamlet_star_path_planner/Beamlet.h"
// #include "utilities/PrintContent.h"

#include <cmath>

Beamlet::Beamlet(): neighborBeamletList()
{
	this->point1 = NULL;
	this->point2 = NULL;
	//this->neighborBeamletList = NULL;
	this->neighborBeamletsComputed = false;
}


Beamlet::Beamlet(Point* point1, Point* point2): neighborBeamletList()
{
	// this->point1 = point1;
	// this->point2 = point2;
	//this->neighborBeamletList = NULL;

    this->point1 = point1;
    this->point2 = point2;

    if (point1) point1->AddBeamlet(this);
    if (point2) point2->AddBeamlet(this);

	this->neighborBeamletsComputed = false;
}

std::list<Beamlet*> Beamlet::GetNeighborBeamlets()
{
    if (neighborBeamletsComputed)
        return neighborBeamletList;

    std::list<Beamlet*> neighbors;

    if (point1) {
        std::list<Beamlet*> bList = point1->GetBeamletList();
        neighbors.insert(neighbors.end(), bList.begin(), bList.end());
    }

    if (point2) {
        std::list<Beamlet*> bList = point2->GetBeamletList();
        neighbors.insert(neighbors.end(), bList.begin(), bList.end());
    }

    // Remove self
    neighbors.remove(this);

    // Cache the result
    neighborBeamletList = neighbors;
    neighborBeamletsComputed = true;

    return neighborBeamletList;
}

double Beamlet::getLength() const
{
    if (!point1 || !point2)
        return 0.0;

    const std::vector<int>& c1 = point1->GetCoordinates();
    const std::vector<int>& c2 = point2->GetCoordinates();

    double dx = static_cast<double>(c1[0] - c2[0]);
    double dy = static_cast<double>(c1[1] - c2[1]);

    return std::sqrt(dx*dx + dy*dy);
}

/*
std::list<Beamlet*> *
Beamlet::GetNeighborBeamlets(BeamletGraph* beamletGraph)
{
	if (this->neighborBeamletsComputed == true)
        return this->neighborBeamletList;
    else
	{
		std::list<Beamlet*> *neighborBeamletList;
        std::list<Point*> *pairPointList;
		std::list<Beamlet*> *beamletList;
		int len;

		neighborBeamletList = new std::list<Beamlet*>();


		pairPointList =	this->GetPoint1()->GetPairPoints(beamletGraph);


		len = pairPointList->size();

		// use iterators


        for(std::list<Point*>::iterator it = pairPointList->begin();
        	it != pairPointList->end(); ++it)
		{
            beamletList = (*it)->GetBeamletList();
			neighborBeamletList->insert(neighborBeamletList->end(), beamletList->begin(), beamletList->end());
		}

        pairPointList = this->GetPoint2()->GetPairPoints(beamletGraph);

		len = pairPointList->size();

		// use iterators
		for(std::list<Point*>::iterator it = pairPointList->begin();
			it != pairPointList->end(); ++it)
		{
            beamletList = (*it)->GetBeamletList();
			neighborBeamletList->insert(neighborBeamletList->end(), beamletList->begin(), beamletList ->end());
		}

        // TODO: think about necessity of sycronize all beamlets'
        // neighborBeamletList
        this->neighborBeamletList = neighborBeamletList;
        this->neighborBeamletsComputed = true;

		return this->neighborBeamletList;
	}

}
*/


// std::ostream&
// operator<<(std::ostream & output, const Beamlet & beamlet)
// {
// 	if (beamlet.point1)
// 	{
// 		std::vector<int> const & coor1 = beamlet.point1->GetCoordinates();

// 		if (!coor1.empty())
// 			output << coor1;
// 		else
// 			output << "[ .. ]";
// 	}
// 	else
// 		output << "[ .. ]";

// 	output << " - ";

// 	if (beamlet.point2)
// 	{
// 		std::vector<int> const & coor2 = beamlet.point2->GetCoordinates();

// 		if (!coor2.empty())
// 			output << coor2;
// 		else
// 			output << "[ .. ]";
// 	}
// 	else
// 		output << "[ .. ]";

// 	return output;
// }

Beamlet::~Beamlet()
{
}


