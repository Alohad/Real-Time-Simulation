#include <iostream>
#include <time.h>
#include <unistd.h>
#include "assignment9.h"

using cimg_library::CImg;
using cimg_library::CImgDisplay;
using std::vector;
using std::uniform_real_distribution;
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
  p0.SetZ(0);
  // std::random_device rng;
  // std::mt19937 gen(Rand()*RAND_MAX);
  // uniform_real_distribution<double> dis1(0.0,2.0);
  // uniform_real_distribution<double> dis2(0.0,pi/18.0);
  // uniform_real_distribution<double> dis3(0.0,2.0*pi);
  double rand_dis1 = Rand()*2;
  double rand_dis2 = Rand()*M_PI/18.0;
  double rand_dis3 = Rand()*2*M_PI;

  float alpha = 9.0 + rand_dis1;
  float phi = rand_dis2;
  float theta = rand_dis3;
  // float alpha = 9.0 + dis1(gen);
  // float phi = dis2(gen);
  // float theta = dis3(gen);

  v0.SetX(alpha * (sin(phi) * cos(theta)));
  v0.SetY(alpha * (sin(phi) * sin(theta)));
  v0.SetZ(alpha * cos(phi));
  // v0 = {alpha * (sin(phi) * cos(theta)), alpha * (sin(phi) * sin(theta)),
        // alpha * cos(phi)};

}

void Particle::Update() {
  double dt = GetMonotonicTime() - t0;
  Vector3 g(0,0,-9.8);
  // p.SetX(p0.GetX() + v0.GetX() * dt + 0.5 * g.GetX() * dt * dt);
  // p.SetY(p0.GetY() + v0.GetY() * dt + 0.5 * g.GetY() * dt * dt);
  // p.SetZ(p0.GetZ() + v0.GetZ() * dt + 0.5 * g.GetZ() * dt * dt);
  p = p0 + v0 * dt + 0.5 * g * dt * dt;

}

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

bool Particle::CheckAndReset() { //might have to do more work here

  if(v0.GetZ() < 0.05) {
    t0 = GetMonotonicTime();
    p0.SetX(0);
    p0.SetY(0);
    p0.SetZ(0);
    // p0 = {0,0,0};
    // float pi = 3.141592653589793238462643383;
    // std::random_device rng;
    // std::mt19937 gen(Rand() * RAND_MAX);
    // uniform_real_distribution<float> dis1(0.0,2.0);
    // uniform_real_distribution<float> dis2(0.0,pi/18.0);
    // uniform_real_distribution<float> dis3(0.0,2.0*pi);

    // float alpha = 9.0 + dis1(gen);
    // float phi = dis2(gen);
    // float theta = dis3(gen);
    double rand_dis1 = Rand()*2;
    double rand_dis2 = Rand()*M_PI/18.0;
    double rand_dis3 = Rand()*2*M_PI;

    float alpha = 9.0 + rand_dis1;
    float phi = rand_dis2;
    float theta = rand_dis3;

    v0.SetX(alpha * (sin(phi) * cos(theta)));
    v0.SetY(alpha * (sin(phi) * sin(theta)));
    v0.SetZ(alpha * cos(phi));
    // v0 = {alpha * (sin(phi) * cos(theta)), alpha * (sin(phi) * sin(theta)), alpha * cos(phi)};

    return true;
  }
  else {return false;}
}

void TransformPoint(const Vector3& p, Vector3* p_img) {
  float gamma = 60.0;
  p_img->SetX(gamma * p.GetX() + 300);
  p_img->SetY(400 - gamma * (0.9396926 * p.GetZ() - 0.342020143 * p.GetY()));

}

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


