#include "beamlet_star_path_planner/Point.h"
// #include "utilities/PrintContent.h"
#include "beamlet_star_path_planner/Beamlet.h"

Point::Point(): coordinates(), beamletList(), pairPointList()
{
	//this->coordinates = NULL;
	//this->beamletList = new std::list<Beamlet*>();
	// this->parentCluster = NULL;

	//this->pairPointList = new std::list<Point*>();
	this->pairPointsComputed = false;
}

Point::Point(const std::vector<int> & coordinates): coordinates(), beamletList(), pairPointList()
{

	//this->coordinates = coordinates;

	std::vector<int>::const_iterator it;

	for (it = coordinates.begin(); it!= coordinates.end(); ++it)
		this->coordinates.push_back((*it));

	//this->beamletList = new std::list<Beamlet*>();
	// this->parentCluster = NULL;

	//this->pairPointList = new std::list<Point*>();
	this->pairPointsComputed = false;

}


// Point::Point(const std::vector<int> & coordinates, Cluster *parentCluster): coordinates(), beamletList(), pairPointList()
// {
// 	//this->coordinates = coordinates;

// 	std::vector<int>::const_iterator it;

// 	for (it = coordinates.begin(); it!= coordinates.end(); ++it)
// 		this->coordinates.push_back((*it));

// 	//this->beamletList = NULL;
// 	this->parentCluster = parentCluster;

// 	//this->pairPointList = NULL;
// 	this->pairPointsComputed = false;
// }


void
Point::AddBeamlet(Beamlet* beamlet)
{
	std::vector<int> const& coordinates = this->coordinates;

	std::vector<int> const&  coor1 = beamlet->GetPoint1()->GetCoordinates();
	std::vector<int> const&  coor2 = beamlet->GetPoint2()->GetCoordinates();


	if (std::equal( coordinates.begin(), coordinates.end(), coor1.begin() ) ||
	    std::equal( coordinates.begin(), coordinates.end(), coor2.begin() ) )
	{
		this->beamletList.push_back(beamlet);
	}
}

void
Point::RemoveBeamlet(std::vector<int> const& coordinates)
{
	if (coordinates.size() == this->coordinates.size())
	{
		if (!this->beamletList.empty())
		{
			std::list<Beamlet*>::iterator it;
		    std::vector<int> coor1, coor2;

			for (it = this->beamletList.begin(); it != this->beamletList.end(); ++it)
			{
				coor1 = (*it)->GetPoint1()->GetCoordinates();
				coor2 = (*it)->GetPoint2()->GetCoordinates();

				if (std::equal( coordinates.begin(), coordinates.end(), coor1.begin() ) ||
					std::equal( coordinates.begin(), coordinates.end(), coor2.begin() ) )
				{
					it = this->beamletList.erase(it);
					--it;
				}
			}
		}
	}
}



std::list<Beamlet*>
Point::GetBeamletList() const
{
	return this->beamletList;
}

std::vector<int>
Point::GetCoordinates() const
{
	return this->coordinates;
}

void
Point::AddPairPoints(Point *pairPoint)
{
	this->pairPointList.push_back(pairPoint);
}

void
Point::AddPairPoints(std::list<Point*> const& pairPoints)
{
	std::list<Point*>::iterator it = this->pairPointList.end();
	this->pairPointList.insert(it, pairPoints.begin(),pairPoints.end());
}


void
Point::SetPairPointsComputed(bool value)
{
	this->pairPointsComputed = value;
}

void Point::getOrCreatePoint(std::map<std::vector<int>, Point*>& pointMap,
                               std::vector<int> coordinates)
{
    auto it = pointMap.find(coordinates);
    if (it != pointMap.end())
        return;

    Point* p = new Point(coordinates);
    pointMap[coordinates] = p;
}

// void printUniquePoints(std::map<std::vector<int>, Point*>& pointMap) const{
//     for (const auto& kv : pointMap)
//         {
//             const std::vector<int>& coords = kv.first;

//             RCLCPP_INFO(
//                 "Point: (%d, %d)",
//                 coords[0],
//                 coords[1]
//             );
//         }
// }

/*
std::list<Point*> *
Point::GetPairPoints(BeamletGraph *beamletGraph)
{
	if (this->pairPointsComputed)
	{
		return this->pairPointList;
	}
	else
	{
		std::list<Cluster*> *pairClusterList;
		std::list<Point*> *pairPointList;

		pairPointList = new std::list<Point*>();

		pairClusterList = this->parentCluster->GetPairClusters(beamletGraph);

		std::list<Cluster*>::iterator it;
		Point *pairPoint;

		for (it = pairClusterList->begin(); it != pairClusterList->end(); ++it)
		{
			pairPoint = it->GetPoint(this->coordinates);
			pairPointList->push_back(pairPoint);
		}

		this->pairPointList = pairPointList;
		this->pairPointsComputed = true;

		// synchronization

		std::list<Point*>::iterator itp;
		Point *point;


		for (itp = pairPointList->begin(); itp != pairPointList->end(); ++itp)
		{
			point = (*itp);

			(*itp) = this;
			point->AddPairPoints(pairPointList);
			point->SetPairPointsComputed(true);

			(*itp) = point;
		}

		return this->pairPointList;
	}

}

*/

// std::ostream&
// operator<<(std::ostream& output, const Point & point)
// {

// 	if (!point.coordinates.empty())
// 		output << "Coordinates : " << point.coordinates << std::endl;
// 	else
// 		output << "Coordinates : " << "[ .. ]" << std::endl;

// 	if(!point.beamletList.empty())
// 		output << "Beamlets : " << point.beamletList << std::endl;
// 	else
// 		output << "Beamlets : " << "{ .. }" << std::endl;

// 	if(!point.pairPointList.empty())
// 		output << "PairPoints : " << point.pairPointList << std::endl;
// 	else
// 		output << "PairPoints : " << "{ .. }" << std::endl;

// 	return output;
// }

Point::~Point()
{
	//TODO: Think about whether define point or value based members
	//delete this->coordinates;
	//delete this->beamletList;
	//delete this->pairPointList;

}
