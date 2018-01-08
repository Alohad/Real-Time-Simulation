#include <vector>
#include <CImg.h>
#include "vector3.h"

typedef Vector3 Vector3f;

struct Particle {

  Particle();

  void Update();

  bool Reflect();

  bool CheckAndReset();

  Vector3 p0; // Initial Location
  Vector3 v0; // Initial Velocity
  Vector3 p;  // Current Location
  double t0;   // Creation Time
};

void TransformPoint(const Vector3& point, Vector3* p_img);

void DrawParticles(const std::vector<Particle>& particles,
                   cimg_library::CImg<unsigned char>* img_ptr);

void PointShadow(const Vector3& point, Vector3* p_shadow);

void DrawShadows(const std::vector<Particle>& particles,
                     cimg_library::CImg<unsigned char>* img_ptr);

double GetMonotonicTime();
double Rand();
