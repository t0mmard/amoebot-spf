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
#include <iostream>
#include <vector>
#include <map>
#include <limits>

class PortalGraphParticle : public AmoebotParticle {
 public:
  enum class Axis {
      X,
      Y,
      Z
  };
  enum Direction {
      EAST=0, NORTHEAST=1, NORTHWEST=2, WEST=3, SOUTHWEST=4, SOUTHEAST=5, NONE=-1
  };

  PortalGraphParticle& nbrAtLabel(int label) const;

  bool leader;
  bool allPascDone() const {
      bool result = true;
      for (const auto& kv : _pascDone) {
          result = result && kv.second;
      }
      return result;
  }

  void setPascDone(Axis axis, bool value) {
      _pascDone[axis] = value;
  }

  bool isPascDone(Axis axis) const {
      return _pascDone.at(axis);
  }

  void setDistanceFromRoot(Axis axis, int value) {
      _distanceFromRoot[axis] = value;
  }

  int getDistanceFromRoot(Axis axis) const {
      return _distanceFromRoot.at(axis);
  }

  Axis getPortalClosestToRoot() {
      Axis result;
      int minimum = std::numeric_limits<int>::max();
      for (const auto& kv : _distanceFromRoot) {
          if (minimum > kv.second) {
              result = kv.first;
              minimum = kv.second;
          }
      }

      return result;
  }

  Direction getDirCloserToRoot(Axis axis) {
      Direction result;
      int minimum = std::numeric_limits<int>::max();
      for (auto dir : getPortalDirections(axis)) {
          if (minimum > nbrAtLabel(dir).getDistanceFromRoot(axis)) {
              minimum = nbrAtLabel(dir).getDistanceFromRoot(axis);
              result = dir;
          }
      }

      return result;
  }

  std::vector<Direction> getPortalDirections(Axis axis) const {
      return _portalDirections.at(axis);
  }

  void pushPortalDirections(Axis axis, Direction dir) {
      _portalDirections[axis].push_back(dir);
  }

  bool neighbourExists(Axis axis, Direction dir) {
      return std::find(_portalDirections[axis].begin(), _portalDirections[axis].end(), dir) != _portalDirections[axis].end();
  }

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, a system that it belongs to, and a maximum value for
  // its counter.
  PortalGraphParticle(const Node& head, const int orientation, const bool leader, std::string portalGraph, AmoebotSystem& system);
  // Executes one particle activation.
  void activate() override;

  // Functions for altering the particle's color. headMarkColor() (resp.,
  // tailMarkColor()) returns the color to be used for the ring drawn around the
  // particle's head (resp., tail) node. In this demo, the tail color simply
  // matches the head color.
  int headMarkColor() const override;
  int headMarkDir() const override;
  int tailMarkColor() const override;

  // Returns the string to be displayed when this particle is inspected; used to
  // snapshot the current values of this particle's memory at runtime.
  QString inspectionText() const override;

 protected:
  // Member variables.
  //They can contain the parallel connections as well
  bool portalSet = false;

 private:
  friend class PortalGraphSystem;
  void createPortalGraph(Axis axis, Direction* portalAxis, Direction* sideA, Direction* sideB, Direction boundaryDirection);
  void initializePortalGraph();
  void calculatePasc();
  void propagateDistance(Axis axis);
  void chooseParent();
  Direction chooseClosestToLeader(std::vector<Direction>);
  int _headMarkDir = -1;
  Direction parent = NONE;
  std::string _portalGraph = "";
  std::map<Axis, bool> _pascDone;
  std::map<Axis, int> _distanceFromRoot;
  std::map<Axis, std::vector<Direction>> _portalDirections;
};

class PortalGraphSystem : public AmoebotSystem {
 public:
  // Constructs a system of the specified number of PortalGraphDemoParticles.
  PortalGraphSystem(int numParticles = 30, std::string portalGraph = "");
};

#endif  // AMOEBOTSIM_ALG_DEMO_PORTALGRAPH_H_
