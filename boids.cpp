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
  auto check_min_dist = [&](double width, double height, double current_min) {
    double distance{this->distance({a.pos_[0] + width, a.pos_[1] + height})};
    current_min= std::min(distance,current_min);
    assert(current_min<=distance);
    assert(current_min>0);
    return current_min;
    /*if (current_min > distance) {
      assert(current_min<=distance);
    assert(current_min>0);
      return distance;
    } else {
      return current_min;
      assert(current_min<=distance);
    assert(current_min>0);
    }*/
    
  };
  double min_dist{this->distance(a)};
  min_dist = check_min_dist(-800, -600, min_dist);
  min_dist = check_min_dist(-800, +600, min_dist);
  min_dist = check_min_dist(800, -600, min_dist);
  min_dist = check_min_dist(800, 600, min_dist);
  min_dist = check_min_dist(-800, 0, min_dist);
  min_dist = check_min_dist(0, -600, min_dist);
  min_dist = check_min_dist(+800, 0, min_dist);
  min_dist = check_min_dist(0, +600, min_dist);
  return min_dist;
}

double Boid::distance(Boid const& a) const {
  return {std::sqrt((pos_[0] - a.pos_[0]) * (pos_[0] - a.pos_[0]) +
                    (pos_[1] - a.pos_[1]) * (pos_[1] - a.pos_[1]))};
}

double Boid::distance(std::array<double, 2> const& arr) const {
  return {std::sqrt((pos_[0] - arr[0]) * (pos_[0] - arr[0]) +
                    (pos_[1] - arr[1]) * (pos_[1] - arr[1]))};
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

std::array<double, 2> Boid::separation(std::vector<const Boid*> const& close) {
  std::array<double, 2> sum{};
  std::array<double, 2> distance_array{};
  for (auto it = close.begin(); it != close.end(); it++) {
    distance_array = this->distance_diff_array(**it);
    /*double distance {std::sqrt(this->distance(distance_array))};
    double factor {10.0/(distance+0.5)+1.0};
    distance_array[0]*= factor;
    distance_array[1]*= factor;*/
    sum = sum_arr(sum, distance_array);
  }
  return {-Flock::s_ * sum[0], -Flock::s_ * sum[1]};
}

std::array<double, 2> Boid::alignment(std::vector<const Boid*> const& close) {
  std::array<double, 2> sum{};
  const long unsigned int n{close.size()};

  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, this->velocity_diff_array(**it));
  }
  if (n <= 1) {
    return {0.0, 0.0};
  }
  const double num{1.0 / static_cast<double>(n)};
  return {Flock::a_ * num * sum[0], Flock::a_ * num * sum[1]};
}

std::array<double, 2> Boid::coesion(std::vector<const Boid*> const& close) {
  std::array<double, 2> sum{};
  const long unsigned int n{close.size()};

  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, (**it).pos_);
    /*sum[0] += pos_[0];
    sum[1] += pos_[1];*/
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

// Updates the state of a boid based on those near it
Boid Boid::update_boid(Flock const& flock) {
  std::vector<const Boid*> near{};
  std::vector<const Boid*> separation{};

  for (long unsigned int it = 0; it != flock.size(); it++) {
    double distance{this->distance(flock[it])};
    if (this != &flock[it] && distance <= Flock::dist_) {
      near.push_back(&flock[it]);
      if (distance <= Flock::d_s_) {
        separation.push_back(&flock[it]);
      }
    }
  }
  if (near.size() == 0) {
    if (pos_[0] > 0 && pos_[1] > 0 && pos_[0] < 800 && pos_[1] < 600) {
      return Boid{
          {pos_[0] + vel_[0] * Flock::dt_, pos_[1] + vel_[1] * Flock::dt_},
          {vel_[0], vel_[1]}};
    } else {
      this->wrap_borders();
      return Boid{
          {pos_[0] + vel_[0] * Flock::dt_, pos_[1] + vel_[1] * Flock::dt_},
          {vel_[0], vel_[1]}};
    }
  }
  std::array<double, 2> new_vel1{this->separation(separation)};
  std::array<double, 2> new_vel2{this->alignment(near)};
  std::array<double, 2> new_vel3{this->coesion(near)};
  vel_[0] = vel_[0] + new_vel1[0] + new_vel2[0] + new_vel3[0];
  vel_[1] = vel_[1] + new_vel1[1] + new_vel2[1] + new_vel3[1];

  this->clamp_speed();
  if (pos_[0] > 0 && pos_[1] > 0 && pos_[0] < 800 && pos_[1] < 600) {
    return Boid{
        {pos_[0] + vel_[0] * Flock::dt_, pos_[1] + vel_[1] * Flock::dt_},
        {vel_[0], vel_[1]}};
  } else {
    this->wrap_borders();
    return Boid{
        {pos_[0] + vel_[0] * Flock::dt_, pos_[1] + vel_[1] * Flock::dt_},
        {vel_[0], vel_[1]}};
  }
}

double Boid::orientation() const {
  if (vel_[0] == 0 && vel_[1] == 0) {
    return 0 + M_PI;
  }
  return std::atan2(vel_[1], vel_[0]);
};

double Boid::getPosX() const { return pos_[0]; };
double Boid::getPosY() const { return pos_[1]; };

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
    vel_[0] = Flock::min_speed_;
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
    pos_[1] -= 800;
  } else if (pos_[1] > 600) {
    pos_[1] -= 600;
  }
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

Flock::Flock() : flock_{} {};

Flock::Flock(long unsigned int dim) {
  for (long unsigned int i = 0; i < dim; i++) {
    flock_.push_back(Boid{});
  }
};

const Boid& Flock::operator[](long unsigned int i) const { return flock_[i]; }

// Updates positions and velocities of the boids in the flock
void Flock::update_flock() {
  Flock updated{this->size()};

  std::transform(flock_.begin(), flock_.end(), updated.flock_.begin(),
                 [this](Boid& b) { return b.update_boid(*this); });

  /*for (long unsigned int i = 0; i != flock_.size(); i++) {
    updated.flock_.push_back(flock_[i].update_boid(*this));
  }*/

  for (long unsigned int i = 0; i != flock_.size(); i++) {
    flock_[i] = updated[i];
  }
}

long unsigned int Flock::size() const { return flock_.size(); }

// Appends one boid to the flock vector
void Flock::push_back(Boid a) { flock_.push_back(a); }

void Flock::init(long unsigned int n) {
  flock_.clear();

  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> x_distribution(0.0, 800.0);
  std::uniform_real_distribution<double> y_distribution(0.0, 600.0);
  std::uniform_real_distribution<double> velocity_distribution(
      Flock::min_speed_, Flock::max_speed_);
  std::uniform_real_distribution<double> orientation(0.0, 360);
  for (long unsigned int i = 0; i != n; i++) {
    flock_.push_back(
        Boid{{x_distribution(eng), y_distribution(eng)},
             {velocity_distribution(eng) * std::cos(orientation(eng)),
              velocity_distribution(eng) * std::sin(orientation(eng))}});
    flock_[i].clamp_speed();
  }
}

void Flock::flight_parameters(std::ifstream& is) {
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

  if (max_speed_ > 500 || max_speed_ < 50) {
    std::cout << "max_speed value error, should be [50,500]" << std::endl;
    throw std::runtime_error{"Value out of range"};
  }
  if (min_speed_ > 50 || min_speed_ < 10) {
    std::cout << "min_speed value error, should be [10,50]" << std::endl;
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
  std::cout << max_speed_ << std::endl
            << min_speed_ << std::endl
            << dist_ << std::endl
            << d_s_ << std::endl
            << s_ << std::endl
            << a_ << std::endl
            << c_ << std::endl
            << dt_ << std::endl;
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
  double n{static_cast<double>(flock_.size())};

  // calculates the mean_distance and mean_velocity
  for (const Boid& i : flock_) {
    double sum_dist{};

    // calculates the mean distance between a boid and every other boid (except
    // itself)
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

  // reuses mean_distance_vector and velocity_vector to calculate the sqaure
  // deviation
  /*for (long unsigned int i = 0; i < flock_.size(); i++) {
    mean_distance_vector[i] = (mean_distance_vector[i] - mean_distance) *
                              (mean_distance_vector[i] - mean_distance);
    distance_stdev += mean_distance_vector[i];

    velocity_vector[i] = (velocity_vector[i] - mean_velocity) *
                         (velocity_vector[i] - mean_velocity);
    velocity_stdev += velocity_vector[i];
  }
  distance_stdev /= n - 1.0;
  velocity_stdev /= n - 1.0;*/

  double distance_variance_sum = 0.0;
  double velocity_variance_sum = 0.0;
  long unsigned int n_int{flock_.size()};

  for (std::size_t i = 0; i < n_int; ++i) {
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

/*std::array<double, 4> Flock::flock_statistics() const {
  const std::size_t n = flock_.size();
  if (n <= 1) {
    // Handle edge case where statistics cannot be meaningfully computed
    return {0.0, 0.0, 0.0, 0.0};
  }

  const double n_double = static_cast<double>(n);

  // 1. Calculate mean distance per boid and mean velocity
  std::vector<double> mean_distances;
  mean_distances.reserve(n);

  double total_mean_distance = 0.0;
  double total_mean_velocity = 0.0;

  for (const Boid& i : flock_) {
    double sum_dist = 0.0;
    for (const Boid& j : flock_) {
      if (&i != &j) {
        sum_dist += i.distance(j);
      }
    }
    // Correctly average over n - 1 other boids
    double mean_dist = sum_dist / (n_double - 1.0);
    mean_distances.push_back(mean_dist);
    total_mean_distance += mean_dist;

    total_mean_velocity += i.velocity();
  }

  total_mean_distance /= n_double;
  total_mean_velocity /= n_double;

  // 2. Calculate standard deviations
  double distance_variance_sum = 0.0;
  double velocity_variance_sum = 0.0;

  for (std::size_t i = 0; i < n; ++i) {
    double diff_dist = mean_distances[i] - total_mean_distance;
    distance_variance_sum += diff_dist * diff_dist;

    double vel = flock_[i].velocity();
    double diff_vel = vel - total_mean_velocity;
    velocity_variance_sum += diff_vel * diff_vel;
  }

  // Sample standard deviation (Bessel's correction: N - 1)
  double distance_stdev = std::sqrt(distance_variance_sum / (n_double - 1.0));
  double velocity_stdev = std::sqrt(velocity_variance_sum / (n_double - 1.0));

  return {total_mean_distance, distance_stdev, total_mean_velocity,
velocity_stdev};
}*/