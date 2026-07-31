#include "boids.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <vector>

using namespace boids;

// Boid memmber functions

Boid Boid::operator+(const Boid& add) {
  return Boid{
      std::array<double, 2>{pos_[0] + add.pos_[0], pos_[1] + add.pos_[1]},
      std::array<double, 2>{vel_[0] + add.vel_[0], vel_[1] + add.vel_[1]}};
}

Boid Boid::operator-(const Boid& add) {
  return Boid{
      std::array<double, 2>{pos_[0] - add.pos_[0], pos_[1] - add.pos_[1]},
      std::array<double, 2>{vel_[0] - add.vel_[0], vel_[1] - add.vel_[1]}};
}

std::array<double, 2> Boid::sum_arr(std::array<double, 2> add1,
                                    std::array<double, 2> add2) {
  return {add1[0] + add2[0], add1[1] + add2[1]};
}

double Boid::distance(Boid const& a) {
  return {std::sqrt((pos_[0] - a.pos_[0]) * (pos_[0] - a.pos_[0]) +
                    (pos_[1] - a.pos_[1]) * (pos_[1] - a.pos_[1]))};
}

std::array<double, 2> Boid::distance_diff_array(Boid const& a) {
  return {std::array<double, 2>{a.pos_[0] - pos_[0], a.pos_[1] - pos_[1]}};
}

std::array<double, 2> Boid::velocity_diff_array(Boid const& a) {
  return {std::array<double, 2>{a.vel_[0] - vel_[0], a.vel_[1] - vel_[1]}};
}

std::array<double, 2> Boid::new_vel1(Flock& close) {
  std::array<double, 2> sum;
  for (auto it = close.flock_.begin(); it != close.flock_.end(); it++) {
    sum = sum_arr(sum, this->distance_diff_array(*it));
  }
  return {-close.separation_ * sum[0], -close.separation_ * sum[2]};
}

std::array<double, 2> Boid::new_vel2(Flock& close) {
  std::array<double, 2> sum{};
  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, this->velocity_diff_array(*it));
  }
  return {close.alignment_ * (1 / (close.size() - 1)) * sum[0],
          -close.alignment_ * (1 / (close.size() - 1)) * sum[1]};
}

std::array<double, 2> Boid::new_vel3(Flock& close) {
  std::array<double, 2> sum{};
  for (auto it = close.begin(); it != close.end(); it++) {
    sum[0] += pos_[0];
    sum[1] += pos_[1];
  }
  sum[0] = (1 / (close.size() - 1) * sum[0]);
  sum[1] = (1 / (close.size() - 1) * sum[1]);
  return {close.coesion_ * (sum[0]), close.coesion_ * sum[1]};
}

Boid::Boid(std::array<double, 2> pos, std::array<double, 2> vel)
    : pos_{pos}, vel_{vel} {};

Boid::Boid()
    : Boid(std::array<double, 2>{0., 0.}, std::array<double, 2>{0., 0.}) {};

// Updates the state of a boid based on those near it
Boid Boid::update_boid(Flock const& flock) {
  Flock near{};
  /*near.dist_ = flock.dist_;
  near.separation_ = flock.separation_;
  near.alignment_ = flock.alignment_;
  near.coesion_ = flock.coesion_;*/
  for (long unsigned int it = 0; it != flock.size(); it++) {
    if (this->distance(flock[it]) <= near.dist_) {
      near.push_back(flock[it]);
    }
  }
  std::array<double, 2> new_vel1 = this->new_vel1(near);
  std::array<double, 2> new_vel2 = this->new_vel2(near);
  std::array<double, 2> new_vel3 = this->new_vel3(near);
  vel_[0] += new_vel1[0] + new_vel2[0] + new_vel3[0];
  vel_[1] += new_vel1[1] + new_vel2[1] + new_vel3[1];
  return Boid{{pos_[0] + vel_[0] * near.dt_, pos_[1] + vel_[1] * near.dt_},
              {vel_[0], vel_[1]}};
}

double Boid::orientation() {
  if (vel_[0] == 0 && vel_[1] > 0) {
    constexpr double pi = 3.14159265358979323846;
    return pi / 2;
  } else if (vel_[0] == 0 && vel_[1] < 0) {
    constexpr double pi = 3.14159265358979323846;
    return 3 * pi / 2;
  }
  return std::atan(vel_[1] / vel_[0]);
};
double Boid::getPosX() { return pos_[0]; };
double Boid::getPosY() { return pos_[1]; };

//
// Flock member functions

double Flock::dist_{};
double Flock::separation_{};
double Flock::alignment_{};
double Flock::coesion_{};
double Flock::dt_{};

Boid Flock::operator[](long unsigned int i) const { return flock_[i]; }

void Flock::update_flock() {
  Flock updated{};
  for (long unsigned int i = 0; i != flock_.size(); i++) {
    updated.flock_.push_back(flock_[i].update_boid(*this));
  }
  for (long unsigned int i = 0; i != flock_.size(); i++) {
    flock_[i] = updated[i];
  }
}

long unsigned int Flock::size() const { return flock_.size(); }

//Appends one boid to the flock vector
void Flock::push_back(Boid a) { flock_.push_back(a); }

void Flock::init(long unsigned int n) {
  std::random_device rd;
  std::default_random_engine eng;
  std::uniform_real_distribution<double> x_distribution(0.0, 800.0);
  std::uniform_real_distribution<double> y_distribution(0.0, 800.0);
  std::uniform_real_distribution<double> velocity_distribution(0.0, 2.0);
  for (long unsigned int i = 0; i != n; i++) {
    flock_.push_back(
        Boid{{x_distribution(eng), y_distribution(eng)},
             {velocity_distribution(eng), velocity_distribution(eng)}});
  }
  dist_ = 1;
  separation_= 1;
  alignment_= 1;
  coesion_= 1;
  dt_= 0.1;
}

std::vector<Boid>::iterator Flock::begin() { return flock_.begin(); }

std::vector<Boid>::iterator Flock::end() { return flock_.end(); }