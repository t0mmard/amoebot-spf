#ifndef SHORTPATHFOREST_H
#define SHORTPATHFOREST_H

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class ShortestPathForestParticle : public AmoebotParticle {
public:
    enum class State{};

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, and a system which it belongs to.
    ShortestPathForestParticle(const Node head, const int globalTailDir,
                           const int orientation, AmoebotSystem& system,
                           State state);

    // Executes one particle activation.
    virtual void activate();

    // Functions for altering a particle's cosmetic appearance; headMarkColor
    // (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int headMarkColor() const;
    virtual int tailMarkColor() const;

    // Returns the string to be displayed when this particle is inspected; used
    // to snapshot the current values of this particle's memory at runtime.
    virtual QString inspectionText() const;

    // Returns the borderColors and borderPointColors arrays associated with the
    // particle to draw the boundaries for leader election.
    virtual std::array<int, 18> borderColors() const;
    virtual std::array<int, 6> borderPointColors() const;

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    ShortestPathForestParticle& nbrAtLabel(int label) const;

    // Returns the label associated with the direction which the next (resp.
    // previous) agent is according to the cycle that the agent is on (which is
    // determined by the provided agentDir parameter).
    int getNextAgentDir(const int agentDir) const;
    int getPrevAgentDir(const int agentDir) const;

    // Returns a count of the number of particle neighbors surrounding the calling
    // particle.
    int getNumberOfNbrs() const;

};

class ShortestPathForestSystem : public AmoebotSystem {

public:
 // Constructs a system of EDFHexagonFormationParticles with an optionally
 // specified size (#particles), number of energy source particles, hole
 // probability in [0,1) controlling how sparse the initial configuration is,
 // and the energy distribution framework parameters.
 ShortestPathForestSystem(int numParticles = 200);

 // Checks whether all particles belong to the energy distribution spanning
 // forest, all particles have fully recharged, and the system has formed a
 // hexagon (i.e., all particles are in ShapeState::Retired).
 bool hasTerminated() const override;
};


#endif // SHORTPATHFOREST_H
