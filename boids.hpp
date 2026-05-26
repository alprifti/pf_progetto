#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <array>
#include <vector>

namespace boids {

class Flock;

class Boid {
  std::array<double, 2> pos_ = {0., 0.};
  std::array<double, 2> vel_ = {0., 0.};


  Boid operator+(const Boid&);

  Boid operator-(const Boid&);

  double pos_squared() const;

  double distance(Boid const&, Boid const&);

 public:
  Boid(std::array<double, 2>, std::array<double, 2>);

  Boid();

  Boid update(Flock const& flock);
};

class Flock {
  std::vector<Boid> flock_;

 public:
   Boid operator[](int i) const;

  void update();

  int size() const;

  void push_back(Boid a);
};
}  // namespace boids
#endif