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

  double orientation();
  double getPosX();
  double getPosY();
};

class Flock {
  friend Boid;

  std::vector<Boid> flock_;
  static double dist_;
  static double separation_;
  static double alignment_;
  static double coesion_;
  static double dt_;

 public:
  Boid operator[](long unsigned int i) const;

  void update_flock();

  long unsigned int size() const;

  void push_back(Boid a);

  void init(long unsigned int);

  std::vector<Boid>::iterator begin();

  std::vector<Boid>::iterator end();
};
}  // namespace boids
#endif