#ifndef DYADICOBJECT_H_
#define DYADICOBJECT_H_

// #include "Cluster.h"
#include "Key.h"
#include <string>

class DyadicObject
{
private:
	Key key;
	// std::vector<Cluster*> clusterList;
    int obstacleCount;
    DyadicObject* children[4];
	std::string color;
	int **boundaries;

public:
	DyadicObject();

    DyadicObject(int scale, int posX, int posY);

	DyadicObject(const Key & key);

	Key
	GetKey() const;

	void
	SetKey(int scale, const std::vector<int> & position);

	void
	SetKey(const Key& key);

	void setBoundaries(int rows, int cols);

	int** getBoundaries() const;

	void SetObstacleCount(int totalObstacles);

	int GetObstacleCount() const;

	void SetColor(const std::string& c);

	std::string GetColor() const;

	void setChild(int i, DyadicObject* child);
	
	DyadicObject* getChild(int i) const;

	bool isLeaf() const;

	~DyadicObject();
};

#endif /* DYADICOBJECT_H_ */