/*
 * Lights.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "Lights.hpp"

Light::Light()
{
  RGB black(0, 0, 0);
  illumination_ = black;
  falloff_ = 0;
  angular_falloff_ = 0;
  dead_distance_ = 1;
}

Light::Light(RGB const & illumination)
{
  illumination_ = illumination;
}

Light::Light(RGB const & illumination, double falloff, double dead_distance)
{
  illumination_ = illumination;
  falloff_ = falloff;
  dead_distance_ = dead_distance;
}

Light::~Light()
{}

RGB Light::getColor() const
{
  return illumination_;
}

RGB Light::getColor(Vec3 const & p) const
{
  return illumination_;
}

void
Light::setColor(RGB const & c)
{
  illumination_ = c;
}

AmbientLight::AmbientLight()
{
  // intentionally empty
}

AmbientLight::AmbientLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

Vec3
AmbientLight::getIncidenceVector(Vec3 const & position) const
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION`";
}

Ray AmbientLight::getShadowRay(Vec3 const & position, bool & use_dist) const
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION";
}

Ray AmbientLight::getLightRay(Vec3 const & position, bool & use_dist)
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION";
}

bool
AmbientLight::intersect(Ray &r) const
{
  return false;
}

PointLight::PointLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

PointLight::PointLight(RGB const & illumination, double falloff, double dead_distance)
: Light(illumination, falloff, dead_distance)
{
  // intentionally empty
}

RGB
PointLight::getColor(Vec3 const & p) const
{
  // TODO for 3a
  return illumination_/(pow(((p-pos_).length())+dead_distance_, falloff_));
  //IMPLEMENT_ME(__FILE__, __LINE__);
}

bool
PointLight::intersect(Ray &r) const
{
  //TODO foor 3a
  Vec3 dir = (pos_ - r.start());
  dir = dir/dir.length();
  if(dir == r.direction())
  {
    return true;
  }
return false;
}
void
PointLight::setPosition(Vec3 const & pos)
{
  pos_ = pos;
}

Vec3
PointLight::getIncidenceVector(Vec3 const & position) const
{
  // TODO for 3a
  return ( position - pos_ )/((pos_-position).length());
  //IMPLEMENT_ME(__FILE__, __LINE__);
}

Ray
PointLight::getShadowRay(Vec3 const & position, bool & use_dist) const
{
  // TODO for 3a
  use_dist = true;
  Vec3 dir = pos_ - position;
  Ray r = Ray::fromOriginAndDirection(position, dir);
  return r;  
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

Ray
PointLight::getLightRay(Vec3 const & position, bool & use_dist)
{
  // TODO for 3a
  use_dist = true;
  Vec3 dir = position - pos_;
  Ray r = Ray::fromOriginAndDirection(pos_, dir.normalize());
  return r;  
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

bool
DirectionalLight::intersect(Ray &r) const
{
  return false;
}
DirectionalLight::DirectionalLight(RGB const & illumination) : Light(illumination)
{
  // intentionally empty
}

void
DirectionalLight::setDirection(Vec3 const & dir)
{
  dir_ = dir;
  dir_.normalize();
}

Vec3
DirectionalLight::getIncidenceVector(Vec3 const & position) const
{
  // TODO for 3a
  //IMPLEMENT_ME(__FILE__, __LINE__);
  Vec3 dir = dir_;
  return dir/dir.length();
}

Ray
DirectionalLight::getShadowRay(Vec3 const & position, bool & use_dist) const
{
  // TODO for 3a
  use_dist =false;
  Ray r = Ray::fromOriginAndDirection(position, -1*dir_);
  return r;
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

Ray DirectionalLight::getLightRay(Vec3 const & position, bool & use_dist)
{
  throw "AMBIENT LIGHTS DO NOT HAVE A SENSE OF DIRECTION OR POSITION";
}

