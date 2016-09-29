/*
 * World.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "World.hpp"

World::World()
{
}

World::~World()
{
  // TODO Auto-generated destructor stub
}

Primitive *
World::intersect(Ray & r) const
{
	std::vector<Primitive *>::const_iterator i = primitivesBegin();
	std::vector<Primitive *>::const_iterator j = primitivesEnd();
	std::vector<Primitive *>::const_iterator k;
	bool intersect = false;
	double min = 10000000000;
	bool some;
	while(i!= j)
	{
		if((*i)->intersect(r,some))
		{
			if (min > r.minT() && r.minT() != 0)
			{
				intersect = true;
				min = r.minT();
				k = i;
			}
		}
		i++;
	}
	if (intersect)
	{
		r.setMinT(min);
		return (*k);
	}
	else
	{
		return NULL;
	}
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

void
World::addPrimitive(Primitive * p)
{
  primitives_.push_back(p);
}

void
World::addLight(Light * l)
{
  lights_.push_back(l);
}

void
World::setAmbientLightColor(RGB ambientColor)
{
  ambientLight_.setColor(ambientColor);
}

RGB
World::getAmbientLightColor() const
{
  return ambientLight_.getColor();
}

void
World::printStats() const
{
  std::cout << "World data:" << std::endl;
  std::cout << " primitives: " << primitives_.size() << std::endl;
  std::cout << " lights: " << lights_.size() << std::endl;
}
