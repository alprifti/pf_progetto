#include "boids.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace boids {

double dt{};
double s{};
double r{};

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

double Boid::pos_squared() const {
  return pos_[1] * pos_[2] + pos_[1] * pos_[2];
}

double Boid::distance(Boid const& a, Boid const& b) {
  return {a.pos_squared() - b.pos_squared()};
}

Boid::Boid(std::array<double, 2> pos, std::array<double, 2> vel) {
  pos_ = pos;
  vel_ = vel;
}

Boid::Boid() {
  Boid(std::array<double, 2>{0., 0.}, std::array<double, 2>{0., 0.});
}

Boid Boid::update(Flock const& flock) {
  Flock close{};
  for (int it = 0; it != flock.size(); it++) {
    if (distance(*this, flock[it]) <= s) {
      close.push_back(flock[it]);
    }
  }
}

Boid Flock::operator[](int i) const { return flock_[i]; }

void Flock::update() {
  Flock updated{};
  for (int i = 0; i != flock_.size(); i++) {
    updated.flock_.push_back(flock_[i].update(*this));
  }
}

int Flock::size() const { return flock_.size(); }

void Flock::push_back(Boid a) { flock_.push_back(a); }

}  // namespace boids