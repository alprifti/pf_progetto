#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/WindowBase.hpp>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
#include <cmath>

#include "boids.hpp"

int main() {
  try {
    long unsigned int n{};
    std::cout << "Insert number of boids [1,1000]" << std::endl;
    std::cin >> n;
    if (n < 1 || n > 1000) {
      std::cout << "Wrong number of boids, terminating program" << std::endl;
      return EXIT_FAILURE;
    }
    boids::Flock flock{};

    std::ifstream is{"config"};
    if (is.is_open()) {
    } else {
      std::cout << "Failure to open configuration file" << std::endl;
    }
    flock.flight_parameters(is);
    is.close();
    flock.init(n);

    // fill vector with triangles
    std::vector<sf::CircleShape> triangles;
    for (long unsigned int i{}; i < flock.size(); i++) {
      triangles.push_back(sf::CircleShape(8.f, 3));
      triangles[i].setFillColor(sf::Color::Green);
      triangles[i].setOrigin(8.f, 8.f);
      triangles[i].setScale(0.5, 1.f);
    }

    // initialize window
    sf::RenderWindow window = sf::RenderWindow(
        sf::VideoMode(800, 600), "Boid simulation", sf::Style::Close);
    window.setFramerateLimit(60);

    sf::Event event;

    std::cout << "Window created and circle initialized" << std::endl;

    // drawing loop
    while (window.isOpen()) {
      // check events
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
        }
      }

      window.clear(sf::Color::Black);

      // draw triangles
      for (long unsigned int i = 0; i != flock.size(); i++) {
        triangles[i].setRotation(
            static_cast<float>((flock[i].orientation()) * 360 /
            (2 * M_PI)));
        triangles[i].setPosition(static_cast<float>(flock[i].getPosX()),
                                 static_cast<float>(flock[i].getPosY()));
        window.draw(triangles[i]);
      }
      flock.update_flock();

      window.display();
    }
  } catch (std::runtime_error const& e) {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cout << "Unknown error";
    return EXIT_FAILURE;
  }
}
