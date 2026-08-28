#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boids.hpp"
#include <cmath> 
using namespace boids;

TEST_CASE("Testing Boid orientation method") {
    
    SUBCASE("Boid with zero velocity returns zero orientation") {
        boids::Boid b({0.0, 0.0}, {0.0, 0.0});
        CHECK(b.orientation() == 0.0);
    }

    SUBCASE("Boid moving along the positive X-axis (Angle 0)") {
        boids::Boid b({0.0, 0.0}, {5.0, 0.0});
        CHECK(b.orientation() == doctest::Approx(0.0));
    }

    SUBCASE("Boid moving along the positive Y-axis (Angle pi/2 or 90 degrees)") {
        boids::Boid b({0.0, 0.0}, {0.0, 5.0});
        CHECK(b.orientation() == doctest::Approx(M_PI / 2.0));
    }

    SUBCASE("Boid moving along the negative X-axis (Angle pi or 180 degrees)") {
        boids::Boid b({0.0, 0.0}, {-5.0, 0.0});
        CHECK(b.orientation() == doctest::Approx(M_PI));
    }

    SUBCASE("Boid moving along the negative Y-axis (Angle -pi/2 or -90 degrees)") {
        boids::Boid b({0.0, 0.0}, {0.0, -5.0});
        CHECK(b.orientation() == doctest::Approx(-M_PI / 2.0));
    }

    SUBCASE("Boid moving diagonally (45 degrees / pi/4 radians)") {
        boids::Boid b({0.0, 0.0}, {3.0, 3.0});
        CHECK(b.orientation() == doctest::Approx(M_PI / 4.0));
    }
}

TEST_CASE("Testing Boid position getters (get_pos_x and get_pos_y)") {
    
    SUBCASE("Default constructor sets position to origin (0, 0)") {
        boids::Boid b;
        CHECK(b.get_pos_x() == doctest::Approx(0.0));
        CHECK(b.get_pos_y() == doctest::Approx(0.0));
    }

    SUBCASE("Parameterized constructor with positive coordinates") {
        double expected_x = 450.5;
        double expected_y = 300.25;
        boids::Boid b({expected_x, expected_y}, {1.0, 1.0});
        
        CHECK(b.get_pos_x() == doctest::Approx(expected_x));
        CHECK(b.get_pos_y() == doctest::Approx(expected_y));
    }

    SUBCASE("Parameterized constructor with negative coordinates (before border wrapping)") {
        double expected_x = -50.0;
        double expected_y = -120.4;
        boids::Boid b({expected_x, expected_y}, {0.0, 0.0});
        
        CHECK(b.get_pos_x() == doctest::Approx(expected_x));
        CHECK(b.get_pos_y() == doctest::Approx(expected_y));
    }

    SUBCASE("Checking X and Y independently") {
        boids::Boid b({750.12, 12.34}, {0.0, 0.0});
        
        CHECK(b.get_pos_x() == doctest::Approx(750.12));

        CHECK(b.get_pos_x() != doctest::Approx(b.get_pos_y()));
        
        CHECK(b.get_pos_y() == doctest::Approx(12.34));
    }
}

TEST_CASE("Testing Flock size method") {
    
    SUBCASE("Default constructor flock has size zero") {
        boids::Flock flock;
        CHECK(flock.size() == 0);
    }

    SUBCASE("Parameterized constructor flock has correct size") {
        long unsigned int expected_size = 10;
        boids::Flock flock(expected_size);
        CHECK(flock.size() == expected_size);
    }

    SUBCASE("Size increases correctly when pushing back a new boid") {
        boids::Flock flock;
        CHECK(flock.size() == 0);

        flock.push_back(boids::Boid());
        CHECK(flock.size() == 1);

        flock.push_back(boids::Boid({10.0, 20.0}, {1.0, 1.0}));
        CHECK(flock.size() == 2);
    }

    SUBCASE("Size matches the requested count after init()") {
        boids::Flock flock;
        long unsigned int count = 25;
        
        flock.init(count);
        CHECK(flock.size() == count);
    }
}

TEST_CASE("Testing Flock push_back method") {
    
    SUBCASE("Pushing back a default boid increases size and stores it correctly") {
        boids::Flock flock;
        CHECK(flock.size() == 0);

        boids::Boid default_boid;
        flock.push_back(default_boid);

        CHECK(flock.size() == 1);
        CHECK(flock[0].get_pos_x() == doctest::Approx(0.0));
        CHECK(flock[0].get_pos_y() == doctest::Approx(0.0));
    }

    SUBCASE("Pushing back a custom boid preserves its position and velocity properties") {
        boids::Flock flock;
        
        std::array<double, 2> custom_pos = {150.5, 250.25};
        std::array<double, 2> custom_vel = {3.5, -2.1};
        boids::Boid custom_boid(custom_pos, custom_vel);

        flock.push_back(custom_boid);

        CHECK(flock.size() == 1);
        CHECK(flock[0].get_pos_x() == doctest::Approx(custom_pos[0]));
        CHECK(flock[0].get_pos_y() == doctest::Approx(custom_pos[1]));
        
        double expected_orientation = std::atan2(custom_vel[1], custom_vel[0]);
        CHECK(flock[0].orientation() == doctest::Approx(expected_orientation));
    }

    SUBCASE("Pushing back multiple boids maintains correct insertion order") {
        boids::Flock flock;

        boids::Boid b1({10.0, 10.0}, {1.0, 1.0});
        boids::Boid b2({20.0, 20.0}, {2.0, 2.0});
        boids::Boid b3({30.0, 30.0}, {3.0, 3.0});

        flock.push_back(b1);
        flock.push_back(b2);
        flock.push_back(b3);

        CHECK(flock.size() == 3);

        CHECK(flock[0].get_pos_x() == doctest::Approx(10.0));
        CHECK(flock[1].get_pos_x() == doctest::Approx(20.0));
        CHECK(flock[2].get_pos_x() == doctest::Approx(30.0));
    }
}

TEST_CASE("Testing Flock begin and end iterators") {
    
    SUBCASE("Iterators on an empty flock") {
        boids::Flock flock;
        
        CHECK(flock.begin() == flock.end());
    }

    SUBCASE("Iterators distance matches flock size") {
        boids::Flock flock;
        long unsigned int count = 10;
        flock.init(count);

        auto distance = std::distance(flock.begin(), flock.end());
        CHECK(distance == static_cast<long>(count));
    }

    SUBCASE("Range-based for loop works correctly with Flock iterators") {
        boids::Flock flock;
        flock.init(5);

        long unsigned int counted_boids = 0;
        
        for (auto const& boid : flock) {
            CHECK(boid.get_pos_x() >= 0.0);
            CHECK(boid.get_pos_x() <= 800.0);
            CHECK(boid.get_pos_y()>=0);
            CHECK(boid.get_pos_y()<=600.0);
            counted_boids++;
        }

        CHECK(counted_boids == flock.size());
    }

    SUBCASE("Iterators are constant (read-only access)") {
        boids::Flock flock;
        flock.init(3);

        static_assert(std::is_same_v<
            std::vector<boids::Boid>::const_iterator, 
            decltype(flock.begin())
        >, "begin() should return a const_iterator");
    }
}

TEST_CASE("Flock::update_flock core functionality") {
  
  SUBCASE("Updates a single isolated boid linearly") {
    boids::Flock flock;
    flock.push_back(boids::Boid({100.0, 100.0}, {50.0, 0.0}));

    flock.update_flock();

    CHECK(flock[0].get_pos_x() == doctest::Approx(102.5));
    CHECK(flock[0].get_pos_y() == doctest::Approx(100.0));
  }

  SUBCASE("Handles zero-velocity boids by assigning valid random speeds") {
    boids::Flock flock;
    flock.push_back(boids::Boid({300.0, 300.0}, {0.0, 0.0}));

    flock.update_flock();

    double speed = flock[0].velocity();
    CHECK(speed >= 39.9);
    CHECK(speed <= 100.1);
  }

  SUBCASE("Applies toroidal border wrapping correctly") {
    boids::Flock flock;
    flock.push_back(boids::Boid({798.0, 200.0}, {100.0, 0.0}));

    flock.update_flock();

    CHECK(flock[0].get_pos_x() == doctest::Approx(3.0));
    CHECK(flock[0].get_pos_y() == doctest::Approx(200.0));
  }

  SUBCASE("Computes neighbor interactions for multiple boids") {
    boids::Flock flock;
    flock.push_back(boids::Boid({200.0, 200.0}, {50.0, 10.0}));
    flock.push_back(boids::Boid({205.0, 200.0}, {45.0, 12.0}));

    double prev_x0 = flock[0].get_pos_x();
    double prev_x1 = flock[1].get_pos_x();

    flock.update_flock();

    CHECK(flock[0].get_pos_x() != doctest::Approx(prev_x0 + 50.0 * 0.05));
    CHECK(flock[1].get_pos_x() != doctest::Approx(prev_x1 + 45.0 * 0.05));
  }
}

TEST_CASE("Flock statistics calculations") {
    
    SUBCASE("Flock with single boid returns zeroed or NaN-safe stats") {
        Flock flock;
        
        std::array<double, 4> stats = flock.flock_statistics();
        
        CHECK(doctest::Approx(stats[0]) == 0.0);
        CHECK(doctest::Approx(stats[1]) == 0.0);
        CHECK(doctest::Approx(stats[2]) == 0.0);
        CHECK(doctest::Approx(stats[3]) == 0.0);
    }

    SUBCASE("Flock with multiple boids computes expected means and standard deviations") {
        Flock flock;
        flock.push_back({{10.0,0.0},{50.0,50.0}});
        flock.push_back({{10.0,30.0},{80.0,40.0}});
        flock.push_back({{300.0,600.0},{60.0,65.0}});
        
        std::array<double, 4> stats = flock.flock_statistics();
        
        double expected_mean_distance = 203.849133;
        double expected_dist_stdev = 75.279876;
        double expected_mean_velocity = 82.87077;
        double expected_vel_stdev = 10.5425;

        CHECK(stats[0] == doctest::Approx(expected_mean_distance));
        CHECK(stats[1] == doctest::Approx(expected_dist_stdev));
        CHECK(stats[2] == doctest::Approx(expected_mean_velocity));
        CHECK(stats[3] == doctest::Approx(expected_vel_stdev));
    }
}