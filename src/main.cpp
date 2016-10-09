#define _GLIBCXX_USE_CXX11_ABI 0
#include "Globals.hpp"
#include "View.hpp"
#include "World.hpp"
#include "Frame.hpp"
#include "Lights.hpp"
#include "core/Scene.hpp"
#include <algorithm>
using namespace std;
//****************************************************
// Global Variables
//****************************************************

Scene * scene = NULL;
World * world = NULL;
View * view = NULL;
Mat4 viewToWorld = identity3D();
Frame * frame = NULL;
int max_trace_depth = 2;

// Get the shaded appearance of the primitive at a given position, as seen along a ray. The returned value should be the sum of
// the shaded colors w.r.t. each light in the scene. DO NOT include the result of recursive raytracing in this function, just
// use the ambient-diffuse-specular formula. DO include testing for shadows, individually for each light.
RGB
getShadedColor(Primitive const & primitive, Vec3 const & pos, Ray const & ray)
{
  //IMPLEMENT_ME(__FILE__, __LINE__);
  // std::cout << "itwashere" << std::endl;
  Vec3 d = ray.direction();
  d = d/d.length();
  // d = -1 * d;
  Material m = primitive.getMaterial();
  if (m.getMT() == 1 && m.getMR() == 0)
  {
    return RGB(0,0,0);
  }
  //   if (m.getMT() == 2 && m.getMR() == 0)
  // {
  //   return RGB(20,20,20);
  // }
  RGB C = primitive.getColor();
  RGB a = ((m.getMA())*(C))*(world->getAmbientLightColor());
  RGB S = (m.getMSM())*C + (1.0 - m.getMSM())*RGB(1.0,1.0,1.0);
  RGB F(0.,0.,0.);
  std::vector<Light *>::const_iterator i = world->lightsBegin();
  bool isPoint;
  while(i != world->lightsEnd())
  {
    Vec3 ur = (*i)->getIncidenceVector(pos);

    Ray rsh = (*i)->getShadowRay(pos - 0.001*ur,isPoint);
    Primitive* nm;
    if (isPoint)
    {
      if ((nm = world->intersect(rsh)) != NULL)
      {
        if (rsh.minT() > 0 && rsh.minT() <= 1 && (nm->getMaterial()).getMT() == 0)
        {
          i++;
          continue;
        }
      }
    }
    else
    {
      if ((nm = world->intersect(rsh)) != NULL)
      {
        if (rsh.minT() > 0 && (nm->getMaterial()).getMT() == 0)
        {
          i++;
          continue;
        }
      }

    }

    ur = -1*ur;
    Vec3 nr = primitive.calculateNormal(pos);
    Vec3 rr = -1*ur + (2*(ur*nr))*nr;
    rr = (1*rr)/rr.length();
    double first = (ur)*(primitive.calculateNormal(pos));
    if (first < 0)
    {
      first = 0;
    }
    double second = ((d*(-1*(rr))));
    if (second < 0)
    {
      second = 0;
    }
    // std::cout << m.getML() << std::endl;
    // std::cout << C << std::endl;
    // std::cout << ((*i)->getColor()) << std::endl;

    F += (m.getML()*(C*(((*i)->getColor(pos))*first)) + (((m.getMS())*S)*((*i)->getColor(pos)))*pow(second,m.getMSP()));
    i++;
  }
  RGB result = F+a;
  // result.scaleToMax(1);
  return result;
  // Use the "world" global variable to access the lights in the input file.
  //  Please be sure to read the following classes in Types.hpp:
  //   - RGB
  //   - Material
  //   - Ray
}

// Raytrace a single ray backwards into the scene, calculating the total color (summed up over all reflections/refractions) seen
// along this ray.
RGB
traceRay(Ray & ray, int depth, int presentMaterial, int of)
{
  // std::cout << ray.start() << " " << depth << std::endl;
  if (depth > max_trace_depth)
    return RGB(0, 0, 0);
  std::vector<Primitive *>::const_iterator i = world->primitivesBegin();
  std::vector<Primitive *>::const_iterator j;
  double min = 100000000000;
  bool intersected = false;
  bool tmpinside;
  bool inside;
  while(i != world->primitivesEnd())
  {
    if((*i)->intersect(ray, tmpinside))
    {
      if (ray.minT() < min && ray.minT() != 0)
      {
        intersected = true;
        j = i;
        min = ray.minT();
        inside = tmpinside;
      }
    }
    i++;
  }
  if (intersected)
  {
      ray.setMinT(min);
      Material m = (*j)->getMaterial();
      RGB C = (*j)->getColor();
      double n2 = m.getMTN();
      double mt = m.getMT();
      Vec3 pos = ray.getPos(ray.minT());
      // std::cout << pos << " " << depth << std::endl;
      Vec3 nr = (*j)->calculateNormal(pos);
      Vec3 rfd;
      Vec3 ur;
      Ray rf;
      if (of)
      {
        of = 0;
      }
      if (inside)
      {
        n2 = 1/n2;
        presentMaterial = 0;
        of = 1;
        nr = -1*nr;
        ur = ray.direction();
        double ct1 = -1*(ur*nr);
        double ss2 = (1-(ct1*ct1))/(n2*n2);
        rfd = ur/n2 + ((ct1/n2)-sqrt(1-ss2))*nr;
        rfd = rfd/rfd.length();
        rf = Ray::fromOriginAndDirection(pos+0.001*rfd,rfd);
        std::vector<Light *>::const_iterator li = world->lightsBegin();
      }
      else
      {
        if (m.getMT() != 0)
        {
          presentMaterial = 1;
        }
      ur = ray.direction();
      double ct1 = -1*(ur*nr);
      double ss2 = (1-(ct1*ct1))/(n2*n2);
      rfd = ur/n2 + ((ct1/n2)-sqrt(1-ss2))*nr;
      rfd = rfd/rfd.length();
      rf = Ray::fromOriginAndDirection(pos+0.001*rfd,rfd);
      }

      Vec3 rr = ur - (2*(ur*nr))*nr;
      rr = rr/rr.length();
      Ray r = Ray::fromOriginAndDirection(pos+0.001*rr,rr);
      return getShadedColor((*(*j)), pos, ray) + (C*(((*j)->getMaterial()).getMR())*traceRay(r, depth+1, presentMaterial, of)) + (C*mt*traceRay(rf, depth+1, presentMaterial, of));
  }
  return RGB(0,0,0);
  // IMPLEMENT_ME(__FILE__, __LINE__);

  // Use the "world" global variable to access the primitives in the input file.
  //  IMPORTANT:
  //  Please start all bounce rays at a small non-zero t value such as 0.001 - this has the effect of slightly offsetting
  //  bounce rays from the surface they're bouncing from, and prevents bounce rays from being occluded by their own surface.
}


RGB
reverseTraceRay(Ray & ray, int depth, int presentMaterial, int of, Vec3 &position)
{
  if (depth > max_trace_depth)
    return RGB(0, 0, 0);
  // Vec3 point = ray.start() - ray.direction()*((ray.start()).z())/((ray.direction()).z());
  // std::cout << point << std::endl;
  RGB result(0,0,0);
  std::cout << ray.start() << " " << depth << std::endl;
  std::vector<Primitive *>::const_iterator i = world->primitivesBegin();
  std::vector<Primitive *>::const_iterator j;
  double min = 100000000000;
  bool intersected = false;
  bool inside;
  while(i != world->primitivesEnd())
  {
    if((*i)->intersect(ray, inside))
    {
      if (ray.minT() < min && ray.minT() != 0)
      {
        intersected = true;
        j = i;
        min = ray.minT();
      }
    }
    i++;
  }
  if (intersected)
  {
      ray.setMinT(min);
      Material m = (*j)->getMaterial();
      RGB C = (*j)->getColor();
      double n2 = m.getMTN();
      double mt = m.getMT();
      Vec3 pos = ray.getPos(ray.minT());
      Vec3 nr = (*j)->calculateNormal(pos);
      Vec3 rfd;
      Vec3 ur;
      Ray rf;
      if (of)
      {
        of = 0;
        position = pos;
        std::cout << pos << " " << depth << std::endl;
        return RGB(1,1,1);
      }
      if (presentMaterial)
      {
        n2 = 1/n2;
        presentMaterial = 0;
        of = 1;
        nr = -1*nr;
        ur = ray.direction();
        double ct1 = -1*(ur*nr);
        double ss2 = (1-(ct1*ct1))/(n2*n2);
        rfd = ur/n2 + ((ct1/n2)-sqrt(1-ss2))*nr;
        rfd = rfd/rfd.length();
        rf = Ray::fromOriginAndDirection(pos+0.001*rfd,rfd);
      }
      else
      {
        if (m.getMT() != 0)
        {
          std::cout << "intersected" << std::endl;
          presentMaterial = 1;
        }
        else
        {
          position = Vec3(0,0,0);
          return RGB(0,0,0);
        }
      ur = ray.direction();
      double ct1 = -1*(ur*nr);
      double ss2 = (1-(ct1*ct1))/(n2*n2);
      rfd = ur/n2 + ((ct1/n2)-sqrt(1-ss2))*nr;
      rfd = rfd/rfd.length();
      rf = Ray::fromOriginAndDirection(pos+0.001*rfd,rfd);
      }

      Vec3 rr = ur - (2*(ur*nr))*nr;
      rr = rr/rr.length();
      Ray r = Ray::fromOriginAndDirection(pos+0.001*rr,rr);
      result += mt*reverseTraceRay(rf, depth+1, presentMaterial, of, position);
  }
  if (result != RGB(0,0,0))
  {
    return result;
  }
  position = Vec3(0,0,0);
  return RGB(0,0,0);
  // IMPLEMENT_ME(__FILE__, __LINE__);

  // Use the "world" global variable to access the primitives in the input file.
  //  IMPORTANT:
  //  Please start all bounce rays at a small non-zero t value such as 0.001 - this has the effect of slightly offsetting
  //  bounce rays from the surface they're bouncing from, and prevents bounce rays from being occluded by their own surface.
}
// Main rendering loop.
void
renderWithRaytracing(int focus)
{
  Sample sample;   // Point on the view being sampled.
  Ray ray;         // Ray being traced from the eye through the point.
  RGB c;           // Color being accumulated per pixel.

  int const rpp = view->raysPerPixel();
  // int const rpp = 10;
  // double r = 2;
  // double d = 0.5;
  // double w = 0.2;
  // double rp = sqrt((r*r) - ((r - w/2)*(r - w/2)));
  double rp = 2.0;
  double d = 11.2;
  for (int yi = 0; yi < view->height(); ++yi)
  {
    for (int xi =0; xi < view->width(); ++xi)
    {
      c = RGB(0, 0, 0);
      if (focus)
      {
        view->getSample(xi, yi, 0, sample);
        int count = 0;
        for (double xp = -1*rp; xp < rp; xp = xp + 0.1)
        {
          for (double yp = -1*sqrt(rp*rp - (xp*xp)); yp < sqrt(rp*rp - (xp*xp)); yp = yp + 0.1)
          {
            // double yp = -1*sqrt(rp*rp - (xp*xp))+0.1;
            ray = view -> createViewingRay2(sample, xp, yp, d);
            ray.transform(viewToWorld);            // transform this to world space
            c += traceRay(ray, 0, 0, 0);
            count++;
            // yp = sqrt(rp*rp - (xp*xp)) - 0.1;
            // ray = view -> createViewingRay2(sample, xp, yp, d);
            // ray.transform(viewToWorld);            // transform this to world space
            // c += traceRay(ray, 0, 0, 0);
            // count++;
          }
        // ray = view->createViewingRay(sample);  // convert the 2d sample position to a 3d ray
        // ray.transform(viewToWorld);            // transform this to world space
        // c += traceRay(ray, 0, 0, 0);
      }
          frame->setColor(sample, c / (double)count);
    }
    else{
      // std::cout << xi << " " << yi << std::endl;
        // c = RGB(0, 0, 0);

      for (int ri = 0; ri < rpp; ++ri)
      {

        view->getSample(xi, yi, ri, sample);
        ray = view->createViewingRay(sample);  // convert the 2d sample position to a 3d ray
        ray.transform(viewToWorld);            // transform this to world space
        c += traceRay(ray, 0, 0, 0);
      }
          frame->setColor(sample, c / (double)rpp);

    }
  }
}
}

// This traverses the loaded scene file and builds a list of primitives, lights and the view object. See World.hpp.
void
importSceneToWorld(SceneInstance * inst, Mat4 localToWorld, int time)
{
  if (inst == NULL)
    return;

  Mat4 nodeXform;
  inst->computeTransform(nodeXform, time);
  localToWorld = localToWorld * nodeXform;
  SceneGroup * g = inst->getChild();

  if (g == NULL)   // for example if the whole scene fails to load
  {
    std::cout << "ERROR: We arrived at an instance with no child?!" << std::endl;
    return;
  }

  int ccount = g->getChildCount();

  for (int i = 0; i < ccount; i++)
  {
    importSceneToWorld(g->getChild(i), localToWorld, 0);
  }

  CameraInfo f;

  if (g->computeCamera(f, time))
  {
    viewToWorld = localToWorld;

    if (view != NULL)
      delete view;

    Vec3 eye(0.0, 0.0, 0.0);
    Vec3 LL(f.sides[FRUS_LEFT], f.sides[FRUS_BOTTOM], -f.sides[FRUS_NEAR]);
    Vec3 UL(f.sides[FRUS_LEFT], f.sides[FRUS_TOP], -f.sides[FRUS_NEAR]);
    Vec3 LR(f.sides[FRUS_RIGHT], f.sides[FRUS_BOTTOM], -f.sides[FRUS_NEAR]);
    Vec3 UR(f.sides[FRUS_RIGHT], f.sides[FRUS_TOP], -f.sides[FRUS_NEAR]);
    view = new View(eye, LL, UL, LR, UR, IMAGE_WIDTH, IMAGE_HEIGHT, RAYS_PER_PIXEL_EDGE);
  }

  LightInfo l;

  if (g->computeLight(l, time))
  {
    if (l.type == LIGHT_AMBIENT)
    {
      RGB amb = world->getAmbientLightColor();
      world->setAmbientLightColor(amb + l.color);
    }
    else if (l.type == LIGHT_DIRECTIONAL)
    {
      DirectionalLight * li = new DirectionalLight(l.color);
      Vec3 dir(0, 0, -1);
      li->setDirection(localToWorld * dir);
      world->addLight(li);
    }
    else if (l.type == LIGHT_POINT)
    {
      PointLight * li = new PointLight(l.color, l.falloff, l.deadDistance);
      Vec3 pos(0, 0, 0);
      li->setPosition(localToWorld * pos);
      world->addLight(li);
    }
    else if (l.type == LIGHT_SPOT)
    {
      throw "oh no";
    }
  }

  double r;
  MaterialInfo m;

  if (g->computeSphere(r, m, time))
  {
    Material mat(m.k[0], m.k[1], m.k[2], m.k[3], m.k[4], m.k[MAT_MS], m.k[5], m.k[6]);
    Sphere * sph = new Sphere(r, m.color, mat, localToWorld);
    world->addPrimitive(sph);
  }

  double w;
  if (g->computeLens(r, w, m, time))
  {
    Material mat(m.k[0], m.k[1], m.k[2], m.k[3], m.k[4], m.k[MAT_MS], m.k[5], m.k[6]);
    Lens * lens = new Lens(r, w, m.color, mat, localToWorld);
    world->addPrimitive(lens);
  }

  TriangleMesh * t;

  if (g->computeMesh(t, m, time))
  {
    Material mat(m.k[0], m.k[1], m.k[2], m.k[3], m.k[4], m.k[MAT_MS], m.k[5], m.k[6]);

    for (vector<MeshTriangle *>::iterator it = t->triangles.begin(); it != t->triangles.end(); ++it)
    {
      Triangle * tri = new Triangle(
        t->vertices[ (**it).ind[0] ]->pos,
        t->vertices[ (**it).ind[1] ]->pos,
        t->vertices[ (**it).ind[2] ]->pos,
        m.color, mat, localToWorld);
      world->addPrimitive(tri);
    }
  }

  std::cout << "Imported scene file" << std::endl;
}

int
main(int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cout << "Usage: " << argv[0] << " scene.scd output.png [max_trace_depth]" << std::endl;
    return -1;
  }

  if (argc >= 5)
    max_trace_depth = atoi(argv[4]);

  int focus = atoi(argv[3]);

  cout << "Max trace depth = " << max_trace_depth << endl;

  // Load the scene from the disk file
  scene = new Scene(argv[1]);

  // Setup the world object, containing the data from the scene
  world = new World();
  importSceneToWorld(scene->getRoot(), identity3D(), 0);
  world->printStats();

  // Set up the output framebuffer
  frame = new Frame(IMAGE_WIDTH, IMAGE_HEIGHT);

  // Render the world
  renderWithRaytracing(focus);

  // Save the output to an image file
  frame->save(argv[2]);
  std::cout << "Image saved!" << std::endl;
}
