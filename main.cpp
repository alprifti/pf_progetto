#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/WindowBase.hpp>
#include <iostream>
#include <optional>
#include <vector>

#include "boids.hpp"

int main() {
  long unsigned int n{100};
  boids::Flock flock;
  flock.init(n);
  flock.update_flock();

  // fill vector with triangles
  std::vector<sf::CircleShape> triangles;
  for (long unsigned int i{}; i < flock.size(); i++) {
    triangles.push_back(sf::CircleShape(5.f,3));
    triangles[i].setFillColor(sf::Color::Green);
  }

  // initialize window
  sf::RenderWindow* window = new sf::RenderWindow(
      sf::VideoMode({800, 600}), "My window", sf::Style::Close);
  window->setFramerateLimit(60);

  sf::Event event;

  while (window->isOpen()) {
    // check events
    while (window->pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window->close();
      }
    }
    window->clear(sf::Color::Black);

    // draw triangles
    for (long unsigned int i = 0; i != flock.size(); i++) {
      triangles[i].setRotation((float)flock[i].orientation());
      triangles[i].setPosition((float)flock[i].getPosX(), flock[i].getPosY());
      window->draw(triangles[i]);
    }
    flock.update_flock();

    window->display();
  }
}
