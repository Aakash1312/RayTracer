/*
 * Primitive.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 *      Modified: sidch
 */

#include "Primitives.hpp"

Primitive::Primitive(RGB const & c, Material const & m, Mat4 const & modelToWorld)
{
  c_ = c;
  m_ = m;
  modelToWorld_ = modelToWorld;
  worldToModel_ = modelToWorld.inverse();
}

Primitive::~Primitive()
{
}

Cylinder::Cylinder(double radius, Vec3 top, Vec3 bottom, RGB const & c, Material const & m, Mat4 const & modelToWorld): Primitive(c, m, modelToWorld)
{
  r_ = radius;
  top_ = top;
  bottom_ = bottom;
}

bool
Cylinder::intersect(Ray &ray, bool &inside) const
{
  bool inside2 = false;
  Vec3 top = Vec3(modelToWorld_ * Vec4(top_, 1.0));
  Vec3 bottom = Vec3(modelToWorld_ * Vec4(bottom_, 1.0));
  inside = false;
  Vec3 normal = (bottom - top).normalize();  
  Vec3 nray = ((ray.direction())*normal)*normal;
  Vec3 xyray = (ray.direction() - nray);
  double ratio;
  if ((ray.direction())*normal)
  {
    if(((ray.direction())*normal) > 0)
    {
      ratio = nray.length() / xyray.length();
    }
  else
  {
    ratio = -1 * nray.length() / xyray.length();
  }
  }
  else
  {
    ratio = 0;
  }


  Vec3 xyrayn = xyray.normalize();
  Vec3 center = top + (((ray.start() - top)*normal)*normal);
  Vec3 U = center - ray.start();
  double m = U*(xyrayn);
  if (U*U <= r_*r_)
  {
    inside2 = true;
    if ((((ray.start() - top)*normal) > 0) && (((ray.start() - top)*normal) < ((bottom-top).length())))
    {
      inside = true;
    }
  }
  if (m < 0 && (!inside))
  {
    return false;
  }

  Vec3 ndisv = ((top - ray.start())*normal)*normal;
  if (ndisv*normal > 0 && ((ray.direction())*normal) > 0)
  {
    double t = ndisv.length()/((ray.direction())*normal);
    Vec3 pos = ray.start() + t*(ray.direction());
    if ((pos - top).length() < r_)
    {
      ray.setMinT(1 *t);
      return true;
    }
  }
  if (((ray.direction())*normal) < 0 && (inside))
  {

    double t = ndisv.length()/((ray.direction())*normal);
    Vec3 pos = ray.start() - t*(ray.direction());
    if ((pos - top).length() < r_)
    {
      ray.setMinT(-1*t);
      return true;
    }
  }
  Vec3 ndish = ((bottom - ray.start())*normal)*normal;
  if (ndish*normal < 0 && ((ray.direction())*normal) < 0)
  {

    double t = ndish.length()/((ray.direction())*normal);
    Vec3 pos = ray.start() - t*(ray.direction());
    if ((pos - bottom).length() < r_)
    {
      ray.setMinT(-1*t);
      return true;
    }
  }

  if (((ray.direction())*normal) > 0  && (inside))
  {

    double t = ndish.length()/((ray.direction())*normal);
    Vec3 pos = ray.start() + t*(ray.direction());
    if ((pos - bottom).length() < r_)
    {
      ray.setMinT(1*t);
      return true;
    }
  }

  if ((U*U - m*m) > r_*r_)
  {
    return false;
  }
  else
  {
    double offset = sqrt(r_*r_ + m*m - U*U);
    double depth;
    double gap;
    if (inside2)
    {
      gap = (m + offset);
    }
    else
    {
      gap = m - offset;
    }
    depth = gap*ratio;

    Vec3 dposition = depth*normal + top-(((top-(ray.start()))*normal)*normal);
    if (((dposition-bottom).length() < (top-bottom).length()) && ((dposition-bottom)*(top-bottom)) > 0 )
    {
      ray.setMinT(sqrt(depth*depth + gap*gap));
      return true;
    }
    else
    {
      return false;
    }
  }
}

Vec3 Cylinder::calculateNormal(Vec3 const & position) const
{
  Vec3 top = Vec3(modelToWorld_ * Vec4(top_, 1.0));
  Vec3 bottom = Vec3(modelToWorld_ * Vec4(bottom_, 1.0));
  Vec3 normal = (bottom-top).normalize();
  if ((((position - top).length()) < r_)) 
  {
    return -1 * normal;
  }
  if ((((position - bottom).length()) < r_)) 
  {
    return normal;
  }
  // if ((((position - top)* normal) < 0.001) && (((position - top)* normal) > -0.001))
  // {
  //   return -1 * normal;
  // }
  // if ((((position - bottom)* normal) < 0.001) && (((position - bottom)* normal) > -0.001))
  // {
  //   return normal;
  // }
  Vec3 temp = position - top;
  double dist = temp*normal;
  Vec3 c = top + dist*normal;
  Vec3 n = (position - c).normalize();
  return n;
}

Sphere::Sphere(double radius, RGB const & c, Material const & m, Mat4 const & modelToWorld): Primitive(c, m, modelToWorld)
{
  r_ = radius;
}

Lens::Lens(double radius, double width, RGB const & c, Material const & m, Mat4 const & modelToWorld): Primitive(c, m, modelToWorld)
{
  r_ = radius;
  w_ = width;
}

bool
Lens::intersect(Ray & ray, bool &inside) const
{
  inside = false;
  Vec3 center = Vec3(modelToWorld_ * Vec4(Vec3(0,0,0), 1.0));
  Vec3 c2 = center + Vec3(0,0,-1*(r_-(w_/2)));
  Vec3 c1 = center + Vec3(0,0,r_-(w_/2));
  // std::cout << c1 << " " << c2 << std::endl;
  // Vec3 c1 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,-1*(r_-(w_/2))), 1.0));
  // Vec3 c2 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,r_-(w_/2)), 1.0));
  Vec3 d = ray.direction();
  Vec3 e = ray.start();
  double disc = ((d*(e-c1))*(d*(e -c1))) - (d*d)*(((e-c1)*(e-c1))-(r_*r_));
  if (disc < 0)
  {
    return false;
  }
  else
  {
    double disc2 = ((d*(e-c2))*(d*(e -c2))) - (d*d)*(((e-c2)*(e-c2))-(r_*r_));
    if (disc2 < 0)
    {
      return false;
    }
    else
    {
    double t1 = (((-1 * d)* (e-c2)) + sqrt(disc2))/(d*d);
    double t2 = (((-1 * d)* (e-c2)) - sqrt(disc2))/(d*d);
    double t3 = (((-1 * d)* (e-c1)) + sqrt(disc))/(d*d);
    double t4 = (((-1 * d)* (e-c1)) - sqrt(disc))/(d*d);
    double mint;
    if (t1 > 0 && t2 > 0)
    {
      if (t1 < t2)
      {
        if (t3 > t4)
        {
          if (t3 < t1)
          {
            return false;
          }
        }
        else
        {
          if (t4 < t1)
          {
            return false;
          }
        }
        mint = t1;
      }
      else
      {
        if (t3 > t4)
        {
          if (t3 < t2)
          {
            return false;
          }
        }
        else
        {
          if (t4 < t2)
          {
            return false;
          }
        }

        mint = t2;
      }
      ray.setMinT(mint);
      inside = false;
      return true;
    }
    else
    {
    double mint;
    if (t3 < 0 && t4 < 0)
    {
      return false;
    }
    else
    {
      if (t3 > t4)
      {
        mint = t3;
      }
      else
      {
        mint = t4;
      }
      ray.setMinT(mint);
      inside = true;
      return true;
    }

    }

    }

  }
}


// bool
// Lens::intersect(Ray & ray, bool &inside) const
// {
//   inside = false;
//   Vec3 center = Vec3(modelToWorld_ * Vec4(Vec3(0,0,0), 1.0));
//   Vec3 c1 = center + Vec3(0,0,-1*(r_-(w_/2)));
//   Vec3 c2 = center + Vec3(0,0,r_-(w_/2));
//   std::cout << c1 << " " << c2 << std::endl;
//   // Vec3 c1 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,-1*(r_-(w_/2))), 1.0));
//   // Vec3 c2 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,r_-(w_/2)), 1.0));
//   Vec3 d = ray.direction();
//   Vec3 e = ray.start();
//   double disc = ((d*(e-c1))*(d*(e -c1))) - (d*d)*(((e-c1)*(e-c1))-(r_*r_));
//   if (disc < 0)
//   {
//     return false;
//   }
//   else
//   {
//     double disc2 = ((d*(e-c2))*(d*(e -c2))) - (d*d)*(((e-c2)*(e-c2))-(r_*r_));
//     if (disc2 < 0)
//     {
//       return false;
//     }
//     else
//     {
//     double t1 = (((-1 * d)* (e-c2)) + sqrt(disc2))/(d*d);
//     double t2 = (((-1 * d)* (e-c2)) - sqrt(disc2))/(d*d);
//     double mint;
//     if (t1 > 0 && t2 > 0)
//     {
//       return false;
//     }
//     else
//     {
//     double t3 = (((-1 * d)* (e-c1)) + sqrt(disc))/(d*d);
//     double t4 = (((-1 * d)* (e-c1)) - sqrt(disc))/(d*d);
//     double mint;
//     if (t3 >0 && t4 > 0)
//     {
//       inside = false;
//       return true;
//     }
//     else
//     { if(t1 < 0 || t2 < 0)
//       {
//       if (t1 > t2)
//       {
//         mint = t1;
//       }
//       else
//       {
//         mint = t2;
//       }
//       ray.setMinT(mint);
//       inside = true;
//       return true;
//     }
//     }

//     }

//     }

//   }
//   return false;
// }

Vec3
Lens::calculateNormal(Vec3 const & position) const
{
  // TODO for 3a
  Vec3 center = Vec3(modelToWorld_ * Vec4(Vec3(0,0,0), 1.0));
  Vec3 c1 = center + Vec3(0,0,-1*(r_-(w_/2)));
  Vec3 c2 = center + Vec3(0,0,r_-(w_/2));

  // Vec3 c2 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,r_-(w_/2)), 1.0));
  // Vec3 c1 = Vec3(modelToWorld_ * Vec4(Vec3(0,0,-1*(r_-(w_/2))), 1.0));
  Vec3 f;
  if (((c2 - position).length()) >= ((c1 - position).length()))
  {
    f = position - c2;
  }
  else
  {
    f = position -c1;
  }
  // f = Vec3(modelToWorld_.transpose() * Vec4(f, 0.0));
  f = f/f.length();
  return f;
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

bool
Sphere::intersect(Ray & ray, bool &inside) const
{
  // TODO for 3a
  inside = false;
  Vec3 c = Vec3(modelToWorld_ * Vec4(Vec3(0,0,0), 1.0));
  Vec3 d = ray.direction();
  Vec3 e = ray.start();
  // d = d / d.length();
  double disc = ((d*(e-c))*(d*(e -c))) - (d*d)*(((e-c)*(e-c))-(r_*r_));
  if (disc < 0)
  {
    return false;
  }
  else
  {
    double t1 = (((-1 * d)* (e-c)) + sqrt(disc))/(d*d);
    double t2 = (((-1 * d)* (e-c)) - sqrt(disc))/(d*d);
    double mint;
    if (t1 < 0 && t2 < 0)
    {
      return false;
    }
    if (t1 >0 && t2 > 0)
    {
      if (t1 < t2)
      {
        mint = t1;
      }
      else
      {
        mint = t2;
      }
    }
    else
    {
      inside = true;
      if (t1 < t2)
      {
        mint = t2;
      }
      else
      {
        mint = t1;
      }

    }
      // ray.transform(modelToWorld_);
      // if (mint < ray.minT())
      // {
      //   return false;
      // }
      // else
      // {
        ray.setMinT(mint);
      // }
    return true;
  }
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

Vec3
Sphere::calculateNormal(Vec3 const & position) const
{
  // TODO for 3a
  Vec3 c = Vec3(modelToWorld_ * Vec4(Vec3(0,0,0), 1.0));
  Vec3 f = position - c;
  // f = Vec3(modelToWorld_.transpose() * Vec4(f, 0.0));
  f = f/f.length();
  return f;
  // IMPLEMENT_ME(__FILE__, __LINE__);
}

//=============================================================================================================================
// Triangle and other primitives are for Assignment 3b, after the midsem. Do not do this for 3a.
//=============================================================================================================================

Triangle::Triangle(Vec3 const & v0, Vec3 const & v1, Vec3 const & v2, RGB const & c, Material const & m,
                   Mat4 const & modelToWorld)
: Primitive(c, m, modelToWorld)
{
  verts[0] = v0;
  verts[1] = v1;
  verts[2] = v2;
}

bool
Triangle::intersect(Ray & ray, bool& inside) const
{
  // TODO for 3b, NOT 3a
  // Vec3 v0 = Vec3(modelToWorld_ * Vec4(verts[0], 1.0));
  // Vec3 v1 = Vec3(modelToWorld_ * Vec4(verts[1], 1.0));
  // Vec3 v2 = Vec3(modelToWorld_ * Vec4(verts[2], 1.0));

  // Vec3 n = (v1-v0)^(v2-v0);
  // Vec3 p = ray.start();
  // double r = (n*(v0 - p))/(n*(ray.direction()));
  // Vec3 p1 = p + r*(ray.direction());
  // Vec3 w = p1 - v0;
  // Vec3 u = v1 - v0;
  // Vec3 v = v2 - v0;
  // double s1 = ((u*v)*(w*v) - (v*v)*(w*u))/(((u*v)*(u*v)) - ((u*u)*(v*v)));
  // double t1 = ((u*v)*(w*u) - (u*u)*(w*v))/(((u*v)*(u*v)) - ((u*u)*(v*v)));
  // if (s1 >= 0 && t1 >=0 && ((s1 + t1) <= 1) && r > 0)
  // {
  //   ray.setMinT(r);
  //   return true;
  // }
  // else
  // {
  //   return false;
  // }
  IMPLEMENT_ME(__FILE__, __LINE__);
}

Vec3
Triangle::calculateNormal(Vec3 const & position) const
{
  // TODO for 3b, NOT 3a
  IMPLEMENT_ME(__FILE__, __LINE__);
}
