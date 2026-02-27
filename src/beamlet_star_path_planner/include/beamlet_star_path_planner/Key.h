/*
 * Key.h
 *
 *  Created on: Jun 15, 2010
 *      Author: oarslan3
 */

#ifndef KEY_H_
#define KEY_H_
#include <vector>

struct Key
{
public:
	int scale;
	std::vector<int> position;

public:
	Key();
	Key(int scale, const std::vector<int> & position);
	~Key();
};

#endif /* KEY_H_ */
