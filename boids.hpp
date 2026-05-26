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


  double distance(Boid const&);

  std::array<double,2> distance_diff_array(Boid const&);

  std::array<double,2> velocity_diff_array(Boid const&);

  std::array<double,2> new_vel1(Flock &);
  std::array<double,2> new_vel2(Flock &);
  std::array<double,2> new_vel3(Flock &);

 public:
  Boid(std::array<double, 2>, std::array<double, 2>);

  Boid();

  Boid update_boid(Flock const& flock);
};

class Flock {
  std::vector<Boid> flock_;

 public:
   Boid operator[](int i) const;

  void update_flock();

  int size() const;

  void push_back(Boid a);

  void init(int);

  std::vector<Boid>::iterator begin();

  std::vector<Boid>::iterator end();
};
}  // namespace boids
#endif