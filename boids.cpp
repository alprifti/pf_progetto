#include "boids.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>

using namespace boids;

// Boid memmber functions

std::array<double, 2> Boid::sum_arr(std::array<double, 2> const& add1,
                                    std::array<double, 2> const& add2) const {
  return {add1[0] + add2[0], add1[1] + add2[1]};
}

double Boid::true_distance(Boid const& a) const {
  double dx{std::abs(pos_[0] - a.pos_[0])};
  double dy{std::abs(pos_[1] - a.pos_[1])};
  if (dx > 400) {
    dx = 800 - dx;
    assert(dx<400);
  }
  if (dy > 300) {
    dy = 600 - dy;
    assert(dy<300);
  }
  return {std::sqrt((dx * dx) + (dy * dy))};
}

Flock Boid::find_closest_boids(Flock const& flock, double distance) const {
  Flock closest{};
  for (const Boid& b : flock.flock_) {
  if (this->pos_ == b.pos_) continue;
    if (this->true_distance(b) < distance) {
      Boid other{b};
      double dx{pos_[0] - other.pos_[0]};
      double dy{pos_[1] - other.pos_[1]};

      if (dx > 400) {
        other.pos_[0] += 800;
      } else if (dx < -400) {
        other.pos_[0] -= 800;
      }

      if (dy > 300) {
        other.pos_[1] += 600;
      } else if (dy < -300) {
        other.pos_[1] -= 600;
      }
      closest.push_back(other);
    }
  }
  return closest;
}

double Boid::velocity() const {
  return {std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1])};
}

std::array<double, 2> Boid::distance_diff_array(Boid const& a) const {
  return std::array<double, 2>{a.pos_[0] - pos_[0], a.pos_[1] - pos_[1]};
}

std::array<double, 2> Boid::velocity_diff_array(Boid const& a) const {
  return std::array<double, 2>{a.vel_[0] - vel_[0], a.vel_[1] - vel_[1]};
}

std::array<double, 2> Boid::separation(Flock const& close) {
  std::array<double, 2> sum{};
  std::array<double, 2> distance_array{};
  for (auto it = close.begin(); it != close.end(); it++) {
    distance_array = this->distance_diff_array(*it);
    sum = sum_arr(sum, distance_array);
  }
  return {-Flock::s_ * sum[0], -Flock::s_ * sum[1]};
}

std::array<double, 2> Boid::alignment(Flock const& close) {
  std::array<double, 2> sum{};
  const long unsigned int n{close.size()};
  if (n == 0) {
    return {0.0, 0.0};
  }

  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, this->velocity_diff_array(*it));
  }
  const double num{1.0 / static_cast<double>(n)};
  return {Flock::a_ * num * sum[0], Flock::a_ * num * sum[1]};
}

std::array<double, 2> Boid::cohesion(Flock const& close) {
  std::array<double, 2> sum{};
  const long unsigned int n{close.size()};
  if (n == 0) {
    return {0.0, 0.0};
  }

  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, (*it).pos_);
  }

  const double num{1.0 / static_cast<double>(n)};
  sum[0] = (num * sum[0]);
  sum[1] = (num * sum[1]);
  return {Flock::c_ * (sum[0] - pos_[0]), Flock::c_ * (sum[1] - pos_[1])};
}

Boid::Boid(std::array<double, 2> pos, std::array<double, 2> vel)
    : pos_{pos}, vel_{vel} {};

Boid::Boid()
    : Boid(std::array<double, 2>{0., 0.}, std::array<double, 2>{0., 0.}) {};

// returns an updated boid based on those near it
Boid Boid::update_boid(Flock const& flock) {
  Flock near{};
  Flock separation{};

  near = this->find_closest_boids(flock, Flock::dist_);
  separation = this->find_closest_boids(near, Flock::d_s_);

  Boid return_boid{*this};
  return_boid.clamp_speed();
  return_boid.pos_[0] = pos_[0] + vel_[0] * Flock::dt_;
  return_boid.pos_[1] = pos_[1] + vel_[1] * Flock::dt_;

  if (near.size() == 0) {
    if (return_boid.pos_[0] >= 0 && return_boid.pos_[1] >= 0 &&
        return_boid.pos_[0] <= 800 && return_boid.pos_[1] <= 600) {
      return return_boid;
    } else {
      return_boid.wrap_borders();
      return return_boid;
    }
  }
  std::array<double, 2> new_vel1{this->separation(separation)};
  std::array<double, 2> new_vel2{this->alignment(near)};
  std::array<double, 2> new_vel3{this->cohesion(near)};
  return_boid.vel_[0] += new_vel1[0] + new_vel2[0] + new_vel3[0];
  return_boid.vel_[1] += new_vel1[1] + new_vel2[1] + new_vel3[1];

  return_boid.pos_[0] = pos_[0] + return_boid.vel_[0] * Flock::dt_;
  return_boid.pos_[1] = pos_[1] + return_boid.vel_[1] * Flock::dt_;

  if (return_boid.pos_[0] >= 0 && return_boid.pos_[1] >= 0 &&
      return_boid.pos_[0] <= 800 && return_boid.pos_[1] <= 600) {
    return return_boid;
  } else {
    return_boid.wrap_borders();
    return return_boid;
  }
}

double Boid::orientation() const {
  if (vel_[0] == 0 && vel_[1] == 0) {
    return 0;
  }
  return std::atan2(vel_[1], vel_[0]);
};

double Boid::get_pos_x() const { return pos_[0]; };
double Boid::get_pos_y() const { return pos_[1]; };

void Boid::clamp_speed() {
  double velocity{std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1])};
  if (velocity > 0.01) {
    if (velocity > Flock::max_speed_) {
      double scale = Flock::max_speed_ / velocity;
      vel_[0] *= scale;
      vel_[1] *= scale;
    } else if (velocity < Flock::min_speed_) {
      double scale = Flock::min_speed_ / velocity;
      vel_[0] *= scale;
      vel_[1] *= scale;
    }
  } else {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> velocity_distribution(
        Flock::min_speed_, Flock::max_speed_);
    std::uniform_real_distribution<double> orientation(0.0, 2.0 * M_PI);
    double speed{velocity_distribution(eng)};
    double angle{orientation(eng)};
    vel_[0] = speed * std::cos(angle);
    vel_[1] = speed * std::sin(angle);
  }
  assert(std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1]) >
         Flock::min_speed_ - 0.1);
  assert(std::sqrt(vel_[0] * vel_[0] + vel_[1] * vel_[1]) <
         Flock::max_speed_ + 0.1);
}

void Boid::wrap_borders() {
  if (pos_[0] < 0 && pos_[1] < 0) {
    pos_[0] += 800;
    pos_[1] += 600;
  } else if (pos_[0] < 0 && pos_[1] > 600) {
    pos_[0] += 800;
    pos_[1] -= 600;
  } else if (pos_[0] > 800 && pos_[1] < 0) {
    pos_[0] -= 800;
    pos_[1] += 600;
  } else if (pos_[0] > 800 && pos_[1] > 600) {
    pos_[0] -= 800;
    pos_[1] -= 600;
  } else if (pos_[0] < 0) {
    pos_[0] += 800;
  } else if (pos_[1] < 0) {
    pos_[1] += 600;
  } else if (pos_[0] > 800) {
    pos_[0] -= 800;
  } else if (pos_[1] > 600) {
    pos_[1] -= 600;
  }
  assert(pos_[0] > 0 && pos_[0] < 800.0);
  assert(pos_[1] > 0 && pos_[1] < 600.0);
}

//
// Flock member functions

double Flock::max_speed_{100.0};
double Flock::min_speed_{40.0};
double Flock::dist_{20.0};
double Flock::d_s_{10.0};
double Flock::s_{2.0};
double Flock::a_{0.3};
double Flock::c_{0.1};
double Flock::dt_{0.05};

Flock::Flock() : flock_{} {}

Flock::Flock(long unsigned int dim) {
  flock_.reserve(dim);
  for (long unsigned int i = 0; i < dim; i++) {
    flock_.push_back(Boid{});
  }
}

const Boid& Flock::operator[](long unsigned int i) const { return flock_[i]; }

// Updates positions and velocities of the boids in the flock
void Flock::update_flock() {
  Flock updated{this->size()};

  std::transform(flock_.begin(), flock_.end(), updated.flock_.begin(),
                 [this](Boid& b) { return b.update_boid(*this); });

  for (long unsigned int i = 0; i != flock_.size(); i++) {
    flock_[i] = updated[i];
  }
}

long unsigned int Flock::size() const { return flock_.size(); }

void Flock::push_back(Boid a) { flock_.push_back(a); }

void Flock::init(long unsigned int n) {
  flock_.clear();
  flock_.reserve(n);

  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> x_distribution(0.0, 800.0);
  std::uniform_real_distribution<double> y_distribution(0.0, 600.0);
  std::uniform_real_distribution<double> velocity_distribution(
      Flock::min_speed_, Flock::max_speed_);
  std::uniform_real_distribution<double> orientation(0.0, 2.0 * M_PI);

  //filling flock_with randomly generated boids
  for (long unsigned int i = 0; i != n; i++) {
    double speed{velocity_distribution(eng)};
    double angle{orientation(eng)};
    flock_.push_back(Boid{{x_distribution(eng), y_distribution(eng)},
                          {speed * std::cos(angle), speed * std::sin(angle)}});
    assert(flock_[i].velocity() > Flock::min_speed_ &&
           flock_[i].velocity() < Flock::max_speed_);
  }
}

void Flock::flight_parameters(std::ifstream& is) {
  // lambda used to read parameters without reusing the same code
  auto read_param = [&](double& param, const char* name) {
    if (!(is >> param)) {
      throw std::runtime_error{
          std::string("Input stream fail while reading: ") + name};
    }
  };

  read_param(max_speed_, "max_speed_");
  read_param(min_speed_, "min_speed_");
  read_param(dist_, "dist_");
  read_param(d_s_, "d_s_");
  read_param(s_, "s_");
  read_param(a_, "a_");
  read_param(c_, "c_");
  read_param(dt_, "dt_");

  // checks if input values are in range
  if (max_speed_ > 500 || max_speed_ < 100) {
    std::cout << "max_speed value error, should be [100,500]" << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (min_speed_ > 99 || min_speed_ < 0) {
    std::cout << "min_speed value error, should be [0,99]" << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (dist_ > 250 || dist_ < 10) {
    std::cout << "dist_ value error, should be [10,250]" << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (d_s_ > 100 || d_s_ < 1 || d_s_ >= dist_) {
    std::cout << "d_s_ value error, should be [1,100] and < dist_ "
              << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (s_ > 10 || s_ < 0) {
    std::cout << "s_ value error, should be [0,10] " << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (a_ > 10 || a_ < 0) {
    std::cout << "a_ value error, should be [0,10] " << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (c_ > 10 || c_ < 0) {
    std::cout << "c_ value error, should be [0,10] " << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (dt_ > 0.1 || dt_ <= 0) {
    std::cout << "dt_value error, should be (0,0.1] " << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
}

std::vector<Boid>::const_iterator Flock::begin() const {
  return flock_.cbegin();
}

std::vector<Boid>::const_iterator Flock::end() const { return flock_.cend(); }

// returns an array with {mean_distance, distance_stdev, mean_velocity,
// velocity_stdev}
std::array<double, 4> Flock::flock_statistics() const {
  std::vector<double> mean_distance_vector{};
  double mean_distance{};
  std::vector<double> velocity_vector{};
  double mean_velocity{};
  std::vector<double> square_deviation{};

  long unsigned int n_int{flock_.size()};
  double n{static_cast<double>(n_int)};

  mean_distance_vector.reserve(n_int);
  velocity_vector.reserve(n_int);

  if (n <= 1) {
    return {0.0, 0.0, 0.0, 0.0};
  }

  // calculates the mean_distance and mean_velocity
  for (const Boid& i : flock_) {
    double sum_dist{};

    // calculates the mean distance between a boid and every other boid
    // (except itself)
    for (const Boid& j : flock_) {
      if (&i != &j) {
        sum_dist += i.true_distance(j);
      }
    }
    sum_dist /= n - 1.0;
    mean_distance_vector.push_back(sum_dist);
    mean_distance += sum_dist;

    double velocity{i.velocity()};
    velocity_vector.push_back(velocity);
    mean_velocity += velocity;
  }
  mean_velocity /= n;
  mean_distance /= n;

  double distance_stdev{};
  double velocity_stdev{};

  double distance_variance_sum = 0.0;
  double velocity_variance_sum = 0.0;

  // calculates the standard deviation of distance and speed
  for (long unsigned int i = 0; i < n_int; ++i) {
    double diff_dist = mean_distance_vector[i] - mean_distance;
    distance_variance_sum += diff_dist * diff_dist;

    double diff_vel = velocity_vector[i] - mean_velocity;
    velocity_variance_sum += diff_vel * diff_vel;
  }

  distance_stdev = std::sqrt(distance_variance_sum / (n - 1.0));
  velocity_stdev = std::sqrt(velocity_variance_sum / (n - 1.0));

  return {mean_distance, distance_stdev, mean_velocity, velocity_stdev};
}

double Flock::get_dt() const { return dt_; }