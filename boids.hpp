#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <array>
#include <fstream>
#include <vector>

namespace boids {

class Flock;

class Boid {
  friend Flock;
  std::array<double, 2> pos_ = {0., 0.};
  std::array<double, 2> vel_ = {0., 0.};

  std::array<double, 2> sum_arr(std::array<double, 2> const&,
                                std::array<double, 2> const&) const;

  double distance(Boid const&) const;
  double distance(std::array<double, 2> const& arr) const;
  double true_distance(Boid const& a) const;

  double velocity() const;

  std::array<double, 2> distance_diff_array(Boid const&) const;

  std::array<double, 2> velocity_diff_array(Boid const&) const;

  std::array<double, 2> separation(std::vector<const Boid*> const&);
  std::array<double, 2> alignment(std::vector<const Boid*> const&);
  std::array<double, 2> cohesion(std::vector<const Boid*> const&);

  void clamp_speed();

  void wrap_borders();

  Boid update_boid(Flock const& flock);

 public:
  Boid(std::array<double, 2>, std::array<double, 2>);

  Boid();

  double orientation() const;
  double getPosX() const;
  double getPosY() const;
};

class Flock {
  friend Boid;

  std::vector<Boid> flock_;
  static double max_speed_;
  static double min_speed_;
  static double dist_;
  static double d_s_;
  static double s_;
  static double a_;
  static double c_;
  static double dt_;

 public:
  Flock();
  Flock(long unsigned int);

  const Boid& operator[](long unsigned int i) const;

  // Updates positions and velocities of the boids in the flock
  void update_flock();

  long unsigned int size() const;

  void push_back(Boid a);

  void init(long unsigned int);

  void flight_parameters(std::ifstream&);

  // returns a const_iterator
  std::vector<Boid>::const_iterator begin() const;

  // returns a const_iterator
  std::vector<Boid>::const_iterator end() const;

  // returns an array with {mean_distance, distance_stdev, mean_velocity,
  // velocity_stdev}
  std::array<double, 4> flock_statistics() const;

  double get_dt() const;
};
}  // namespace boids
#endif