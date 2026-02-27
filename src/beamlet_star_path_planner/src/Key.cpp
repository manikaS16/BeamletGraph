/*
 * Key.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: oarslan3
 */

#include "beamlet_star_path_planner/Key.h"

Key::Key(): scale(0), position()
{
	// TODO Auto-generated constructor stub

}


Key::Key(int scale, const std::vector<int> & position): scale(scale), position(position)
{}


Key::~Key()
{
	// TODO Auto-generated destructor stub
}
