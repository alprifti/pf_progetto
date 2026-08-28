#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <array>
#include <fstream>
#include <vector>

namespace boids {

class Flock;

class Boid {
  friend Flock;
  std::array<double, 2> pos_;
  std::array<double, 2> vel_;

  // returns sum of two arrays
  std::array<double, 2> sum_arr(std::array<double, 2> const&,
                                std::array<double, 2> const&) const;

  // return the distance between two boids, taking into account the torrus wrap
  double true_distance(Boid const& a) const;

  // returns a Flock filled with the closest boids with their positions wrapped
  // so that they are next to the boid the method gets called on
  Flock find_closest_boids(Flock const&, double) const;

  // returns a's position minus this boid's position.
  std::array<double, 2> distance_diff_array(Boid const&) const;

  // returns a's velocity minus this boid's velocity.
  std::array<double, 2> velocity_diff_array(Boid const&) const;

  // calculates the velocity change caused by separation
  std::array<double, 2> separation(Flock const&);
  // calculates the velocity change caused by alignment
  std::array<double, 2> alignment(Flock const&);
  // calculates the velocity change caused by coesion
  std::array<double, 2> cohesion(Flock const&);

  // keeps the speed of a boid between Flock::min,Flock::max
  void clamp_speed();

  // kees a boid inside the window
  void wrap_borders();

  // returns an updated boid based on those near it
  Boid update_boid(Flock const& flock);

 public:
  // returns magnitude of the velocity of the boid
  double velocity() const;

  Boid(std::array<double, 2>, std::array<double, 2>);

  Boid();

  // returns orientation of the boid
  double orientation() const;
  // returns x positon of the boid
  double get_pos_x() const;
  // returns y position of the boid
  double get_pos_y() const;
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
  // default constructor
  Flock();
  Flock(long unsigned int);

  // retuns the flock_[i] boid
  const Boid& operator[](long unsigned int i) const;

  // Updates positions and velocities of the boids in the flock
  void update_flock();

  // returns the size of the flock_ vector
  long unsigned int size() const;

  // pushes back a given boid inside the flock_ vector
  void push_back(Boid a);

  // initializes the flock by generating n random boids
  void init(long unsigned int n);

  // inserts the flight parameters from an input stream
  void flight_parameters(std::ifstream&);

  // returns a const_iterator
  std::vector<Boid>::const_iterator begin() const;

  // returns a const_iterator
  std::vector<Boid>::const_iterator end() const;

  // returns an array with {mean_distance, distance_stdev, mean_velocity,
  // velocity_stdev}
  std::array<double, 4> flock_statistics() const;

  // returns FLock::dt_
  double get_dt() const;
};
}  // namespace boids
#endif