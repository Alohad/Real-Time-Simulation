#include <iostream>
#include <time.h>
#include <unistd.h>
#include "assignment9.h"

using cimg_library::CImg;
using cimg_library::CImgDisplay;
using std::vector;

int main(int num_args, char* args[]) {
 vector<Particle> particles_;
 CImg<unsigned char> image(600, 600, 1, 3, 255);
 CImgDisplay display(image, "Simulation");
 int iterations = 0;
 const int num_particles = 20000;
 int particles_handled = 0;
 double time_elapsed = 0;
 while (!display.is_closed() &&
        !display.is_keyESC()) {

   display = image;
   image.fill(255);
   if (particles_.size() < num_particles) {
    particles_.resize(particles_.size() + 4);
   }

   const double start_time = GetMonotonicTime();
   for (Particle& p : particles_) {
     p.Update();
     if (p.Reflect()) {
      p.CheckAndReset();
     }
   }
   DrawParticles(particles_, &image);
   DrawShadows(particles_, &image);
   const double end_time = GetMonotonicTime();

   iterations++;
   particles_handled += particles_.size();

   time_elapsed += end_time - start_time;
   const double particles_per_second = particles_handled / time_elapsed;
   std::cout << "Time Run: " << time_elapsed << std::endl;
   std::cout << "Particles Handled: " << particles_handled << std::endl;
   std::cout << "Particles Per Second " << particles_per_second << std::endl << std::endl;
 }

 return 0;
}