/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

// Defines the particle system and composing particles for the Disco code
// tutorial, a first algorithm for new developers to AmoebotSim. Disco
// demonstrates the basics of algorithm architecture, instantiating a particle
// system, moving particles, and changing particles' states. The pseudocode is
// available in the docs:
// [https://amoebotsim.rtfd.io/en/latest/tutorials/tutorials.html#discodemo-your-first-algorithm].

#ifndef AMOEBOTSIM_ALG_DEMO_PORTALGRAPH_H_
#define AMOEBOTSIM_ALG_DEMO_PORTALGRAPH_H_

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"
#include <vector>

class PortalGraphParticle : public AmoebotParticle {
 public:
  enum class State {
    Red,
    Nothing
  };

  enum Direction {
      EAST=0, NORTHEAST=1, NORTHWEST=2, WEST=3, SOUTHWEST=4, SOUTHEAST=5
  };

  PortalGraphParticle& nbrAtLabel(int label) const;

  bool leader;
  int xDistanceFromRoot;
  int yDistanceFromRoot;
  int zDistanceFromRoot;

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, a system that it belongs to, and a maximum value for
  // its counter.
  PortalGraphParticle(const Node& head, const int orientation, const bool leader, AmoebotSystem& system);
  // Executes one particle activation.
  void activate() override;

  // Functions for altering the particle's color. headMarkColor() (resp.,
  // tailMarkColor()) returns the color to be used for the ring drawn around the
  // particle's head (resp., tail) node. In this demo, the tail color simply
  // matches the head color.
  int headMarkColor() const override;
  int tailMarkColor() const override;

  // Returns the string to be displayed when this particle is inspected; used to
  // snapshot the current values of this particle's memory at runtime.
  QString inspectionText() const override;

 protected:
  // Returns a random State.
  State getRandColor(const bool leader) const;

  // Member variables.
  State _state;
  bool xPascDone;
  bool yPascDone;
  bool zPascDone;
  //They can contain the parallel connections as well
  std::vector<Direction> portalXDirections;
  std::vector<Direction> portalYDirections;
  std::vector<Direction> portalZDirections;
  bool portalSet;

 private:
  friend class PortalGraphSystem;
  void createPortalGraph(std::vector<Direction>& portalDirections, Direction* axis, Direction* sideA, Direction* sideB, Direction boundaryDirection);
};

class PortalGraphSystem : public AmoebotSystem {
 public:
  // Constructs a system of the specified number of PortalGraphDemoParticles.
  PortalGraphSystem(int numParticles = 30);
};

#endif  // AMOEBOTSIM_ALG_DEMO_PORTALGRAPH_H_
