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

  std::array<double, 2> sum_arr(std::array<double, 2>, std::array<double, 2>);

  double distance(Boid const&);

  std::array<double, 2> distance_diff_array(Boid const&);

  std::array<double, 2> velocity_diff_array(Boid const&);

  std::array<double, 2> new_vel1(Flock&);
  std::array<double, 2> new_vel2(Flock&);
  std::array<double, 2> new_vel3(Flock&);

 public:
  Boid(std::array<double, 2>, std::array<double, 2>);

  Boid();

  Boid update_boid(Flock const& flock);
};

class Flock {
  std::vector<Boid> flock_;
  double dist_{};
  double separation_{};
  double alignment_{};
  double coesion_{};

 public:
  Boid operator[](int i) const;

  void update_flock();

  int size() const;

  void push_back(Boid a);

  void init(int);

  double get_dist();
  double get_separation();
  double get_alignment();
  double get_coesion();

  std::vector<Boid>::iterator begin();

  std::vector<Boid>::iterator end();
};
}  // namespace boids
#endif