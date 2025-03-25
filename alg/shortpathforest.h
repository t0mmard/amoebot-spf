#ifndef SHORTPATHFOREST_H
#define SHORTPATHFOREST_H

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class ShortestPathForestParticle : public AmoebotParticle {
public:
    enum class State {
      Seed,      // The unique particle centering the hexagon.
      Idle,      // All other particles' initial state.
      Follower,  // Member of the spanning forest but not on the forming hexagon.
      Root,      // On the surface of the forming hexagon.
      Retired    // In the forming hexagon.
    };

      // Constructs a new contracted particle with a node position for its head, a
      // particle system it belongs to, and an initial state (either State::Seed or
      // State::Idle).
      ShortestPathForestParticle(const Node head, AmoebotSystem& system,const State state);

      // Executes one particle activation.
      void activate() override;

      // Functions for altering a particle's cosmetic appearance; headMarkColor
      // (respectively, tailMarkColor) returns the color to be used for the ring
      // drawn around the head (respectively, tail) node. Tail color is not shown
      // when the particle is contracted. headMarkDir returns the label of the port
      // to draw the head marker on.
      int headMarkColor() const override;
      int tailMarkColor() const override;
      int headMarkDir() const override;

      // Returns the string to be displayed when this particle is inspected; used
      // to snapshot the current values of this particle's memory at runtime.
      QString inspectionText() const override;

      // Gets a reference to the neighboring particle incident to the specified port
      // label. Crashes if no such particle exists at this label; consider using
      // hasNbrAtLabel() first if unsure.
      ShortestPathForestParticle& nbrAtLabel(int label) const;

      // Returns the label of the first port incident to a neighboring particle in
      // any of the specified states, starting at the (optionally) specified label
      // and continuing counterclockwise.
      int labelOfFirstNbrInState(std::initializer_list<State> states,
                                 int startLabel = 0) const;

      // Checks whether this particle has a neighbor in any of the given states.
      bool hasNbrInState(std::initializer_list<State> states) const;

      // Returns the direction from this particle's head to the next position in an
      // oriented traversal (+1 for clockwise, -1 for counter-clockwise) along the
      // forming hexagon's surface. See the .cpp implementation for details on why
      // the usual +1 for counter-clockwise, -1 for clockwise is reversed here.
      int nextHexagonDir(int orientation) const;

      // Returns true if and only if a seed or retired neighbor is pointing at this
      // particle with its _dir variable.
      bool canRetire() const;

      // Returns true if and only if there is a neighbor whose _sParent variable
      // points at this particle's tail.
      bool hasTailChild() const;

      // Returns a list of labels that uniquely address any contracted neighbors
      // whose _sParent variable points at this particle's tail.
      const std::vector<int> conTailChildLabels() const;

     protected:
      // Particle memory.
      State _state;
      int _parentDir;   // Corresponds to "parent" in paper.
      int _hexagonDir;  // Corresponds to "dir" in paper.

     private:
      friend class ShortestPathForestSystem;
    };


class ShortestPathForestSystem : public AmoebotSystem {

public:
 // Constructs a system of HexagonFormationParticles with an optionally
 // specified size (#particles) and hole probability in [0,1) controlling how
 // sparse the initial configuration is.
 ShortestPathForestSystem(int numParticles = 200);

 // Checks whether the system has formed a hexagon (i.e., all particles are in
 // State::Seed or State::Retired).
 bool hasTerminated() const override;

};

bool dfsPathExists(const std::set<Node>& graph, const Node& start, const Node& target,std::set<Node>& visited);
#endif // SHORTPATHFOREST_H
