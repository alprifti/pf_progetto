#include <array>
#include <vector>

namespace boids {

double dt{};
double s{};
double r{};

class Bird {
  double x_{};
  double y_{};
  // std::array<double, 2> velocity = {0.,0.};
  double vx_{};
  double vy_{};

  std::array<double, 2> vcalc1() {}
  std::array<double, 2> vcalc2() {}
  std::array<double, 2> vcalc3() {}

 public:
  void new_velocity(Bird* b) {
    std::array<double, 2> vmod1;
    std::array<double, 2> vmod2;
    std::array<double, 2> vmod3;
    if (/*condizione v1*/ true) {
      vmod1 = vcalc1();
    }
    if (/*condizione v2*/ true) {
      vmod2 = vcalc2();
    }
    if (/*condizione v3*/ true) {
      vmod3 = vcalc2();
    }
    vx_ = vx_ + vmod1[1] + vmod2[1] + vmod3[1];
    vy_ = vy_ + vmod1[2] + vmod2[2] + vmod3[2];
  }
  void new_position() {
    x_ = x_ + vx_ * dt;
    x_ = x_ + vx_ * dt;
  }
};
}  // namespace Boids