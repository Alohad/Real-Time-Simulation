#include <iostream>
#include <time.h>
#include <unistd.h>
#include "assignment9.h"

using cimg_library::CImg;
using cimg_library::CImgDisplay;
using std::vector;
using std::sin;
using std::cos;
using std::cout;
using std::endl;

double GetMonotonicTime() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const double time =
  static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec)*(1.0E-9);
  return time;
}

double Rand() {
  return rand() / static_cast<double>(RAND_MAX);
}
const unsigned char color[] = {0};


Particle::Particle() {
  t0 = GetMonotonicTime();
  p0.SetX(0);
  p0.SetY(0);
  p0.SetZ(0); //Setting the initial locations and time for the new particle

  double rand_dis1 = Rand()*2;
  double rand_dis2 = Rand()*M_PI/18.0;
  double rand_dis3 = Rand()*2*M_PI;

  float alpha = 9.0 + rand_dis1;
  float phi = rand_dis2;
  float theta = rand_dis3;


  v0.SetX(alpha * (sin(phi) * cos(theta)));
  v0.SetY(alpha * (sin(phi) * sin(theta)));
  v0.SetZ(alpha * cos(phi)); //Setting  random initial velocities for the new particle


}

void Particle::Update() {
  double dt = GetMonotonicTime() - t0;
  Vector3 g(0,0,-9.8);

  p = p0 + v0 * dt + 0.5 * g * dt * dt; //Updating location of particle 

}

//Reflect checks if the particle has reached the ground
//If particle is at ground, set vertical velocity to positive and reduce to 25%
bool Particle::Reflect() { 
  if(std::signbit(p.GetZ())) {
    t0 = GetMonotonicTime();
    p.SetZ(p.GetZ()*-1);
    p0 = p;
    v0.SetZ(v0.GetZ() * 0.25);
    return true;
  }
  else {return false;}
}

//If the particle no longer has vertical velocity, reset it to the starting position and a new random set of velocities
bool Particle::CheckAndReset() { 

  if(v0.GetZ() < 0.05) {
    t0 = GetMonotonicTime();
    p0.SetX(0);
    p0.SetY(0);
    p0.SetZ(0);

    double rand_dis1 = Rand()*2;
    double rand_dis2 = Rand()*M_PI/18.0;
    double rand_dis3 = Rand()*2*M_PI;

    float alpha = 9.0 + rand_dis1;
    float phi = rand_dis2;
    float theta = rand_dis3;

    v0.SetX(alpha * (sin(phi) * cos(theta)));
    v0.SetY(alpha * (sin(phi) * sin(theta)));
    v0.SetZ(alpha * cos(phi));


    return true;
  }
  else {return false;}
}

//Converting the 3 dimensional position into a 2 dimensional point for the animation 
void TransformPoint(const Vector3& p, Vector3* p_img) {
  float gamma = 60.0;
  p_img->SetX(gamma * p.GetX() + 300);
  p_img->SetY(400 - gamma * (0.9396926 * p.GetZ() - 0.342020143 * p.GetY()));

}

//Add a shadow for the point in the animation
void PointShadow(const Vector3& point, Vector3* p_shadow) {
  float gamma = 60.0;
  p_shadow->SetX(gamma * point.GetX() + 300);
  p_shadow->SetY(400 - gamma * (-0.342020143 * point.GetY()));
}

void DrawParticles(const std::vector<Particle>& particles,
                   CImg<unsigned char>* img_ptr) {
  int size = particles.size();
  Vector3 transformed_point(0.0,0.0,0.0);
  for(int i = 0; i < size; ++i) {
    TransformPoint(particles.at(i).p, &transformed_point);
    img_ptr->draw_circle(transformed_point.GetX(),
                         transformed_point.GetY(),
                         1,
                         color);
  }

}

void DrawShadows(const std::vector<Particle>& particles,
                     CImg<unsigned char>* img_ptr) {
  int size = particles.size();
  Vector3 transformed_point(0.0,0.0,0.0);
  for(int i = 0; i < size; ++i) {
    PointShadow(particles.at(i).p, &transformed_point);
    img_ptr->draw_circle(transformed_point.GetX(),
                         transformed_point.GetY(),
                         1,
                         color,
                         0.2);
  }  
}


