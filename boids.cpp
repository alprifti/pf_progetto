#include "boids.hpp"

#include <array>
#include <cmath>
#include <vector>

using namespace boids;

double dt{};

Boid Boid::operator+(const Boid& add) {
  return Boid{
      std::array<double, 2>{pos_[1] + add.pos_[1], pos_[2] + add.pos_[2]},
      std::array<double, 2>{vel_[1] + add.vel_[1], vel_[2] + add.vel_[2]}};
}

Boid Boid::operator-(const Boid& add) {
  return Boid{
      std::array<double, 2>{pos_[1] - add.pos_[1], pos_[2] - add.pos_[2]},
      std::array<double, 2>{vel_[1] - add.vel_[1], vel_[2] - add.vel_[2]}};
}

std::array<double, 2> sum_arr(std::array<double, 2> add1,
                              std::array<double, 2> add2) {
  return {add1[1] + add2[1], add1[2] + add2[2]};
}

double Boid::distance(Boid const& a) {
  return {std::sqrt((pos_[1] - a.pos_[1]) * (pos_[1] - a.pos_[1]) +
                    (pos_[2] - a.pos_[2]) * (pos_[2] - a.pos_[2]))};
}

std::array<double, 2> Boid::distance_diff_array(Boid const& a) {
  return {std::array<double, 2>{a.pos_[1] - pos_[1], a.pos_[2] - pos_[2]}};
}

std::array<double, 2> Boid::velocity_diff_array(Boid const& a) {
  return {std::array<double, 2>{a.vel_[1] - vel_[1], a.vel_[2] - vel_[2]}};
}

std::array<double, 2> Boid::new_vel1(Flock& close) {
  std::array<double, 2> sum;
  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, this->distance_diff_array(*it));
  }
  return {-close.get_separation() * sum[1], -close.get_separation() * sum[2]};
}

std::array<double, 2> Boid::new_vel2(Flock& close) {
  std::array<double, 2> sum;
  for (auto it = close.begin(); it != close.end(); it++) {
    sum = sum_arr(sum, this->velocity_diff_array(*it));
  }
  return {close.get_alignment() * (1 / (close.size() - 1)) * sum[1],
          -close.get_alignment() * (1 / (close.size() - 1)) * sum[2]};
}

std::array<double, 2> Boid::new_vel3(Flock& close) {
  std::array<double, 2> sum;
  for (auto it = close.begin(); it != close.end(); it++) {
    sum[1] += pos_[1];
    sum[2] += pos_[2];
  }
  sum[1] = (1 / (close.size() - 1) * sum[1]);
  sum[2] = (1 / (close.size() - 1) * sum[2]);
  return {c * (sum[1]), c * sum[2]};
}

Boid::Boid(std::array<double, 2> pos, std::array<double, 2> vel) {
  pos_ = pos;
  vel_ = vel;
}

Boid::Boid() {
  Boid(std::array<double, 2>{0., 0.}, std::array<double, 2>{0., 0.});
}

Boid Boid::update_boid(Flock const& flock) {
  Flock close{};
  for (int it = 0; it != flock.size(); it++) {
    if (this->distance(flock[it]) <= close.get_dist()) {
      close.push_back(flock[it]);
    }
  }
  std::array<double, 2> new_vel1 = this->new_vel1(close);
  std::array<double, 2> new_vel2 = this->new_vel2(close);
  std::array<double, 2> new_vel3 = this->new_vel3(close);
}

Boid Flock::operator[](int i) const { return flock_[i]; }

void Flock::update_flock() {
  Flock updated{};
  for (int i = 0; i != flock_.size(); i++) {
    updated.flock_.push_back(flock_[i].update_boid(*this));
  }
}

int Flock::size() const { return flock_.size(); }

void Flock::push_back(Boid a) { flock_.push_back(a); }

void Flock::init(int n) {
  for (auto const& i : flock_) {
    flock_.push_back(Boid{});
  }
}

double Flock::get_dist() { return dist_; }
double Flock::get_separation() { return separation_; }
double Flock::get_alignment() { return alignment_; }
double Flock::get_coesion() { return coesion_; }

std::vector<Boid>::iterator Flock::begin() { return flock_.begin(); }

std::vector<Boid>::iterator Flock::end() { return flock_.end(); }