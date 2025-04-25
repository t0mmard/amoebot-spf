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
#include <random>
#include <sstream>
#include <iomanip>

enum Axis {
    X=0,
    Y=1,
    Z=2
};
enum Direction {
    EAST=0, NORTHEAST=1, NORTHWEST=2, WEST=3, SOUTHWEST=4, SOUTHEAST=5, NONE=-1
};
// Struct to hold axis data
struct AxisData {
    std::array<Direction, 2> axis;
    std::array<Direction, 2> sideA;
    std::array<Direction, 2> sideB;
    Direction boundaryDirection;
};

// Map that stores axis information
const std::map<Axis, AxisData> axisMap = {
    {X, {{WEST, EAST},
         {NORTHWEST, NORTHEAST},
         {SOUTHWEST, SOUTHEAST},
         WEST
     }},

    {Y, {{SOUTHWEST, NORTHEAST},
         {NORTHWEST, WEST},
         {EAST, SOUTHEAST},
         NORTHEAST
     }},

    {Z, {{SOUTHEAST, NORTHWEST},
         {SOUTHWEST, WEST},
         {EAST, NORTHEAST},
         SOUTHEAST
     }}
};

class PortalGraphParticle : public AmoebotParticle {
public:
    std::string groupId[2];

    std::string generate_uuid() {
        static thread_local std::mt19937_64 rng(std::random_device{}());
        static thread_local std::uniform_int_distribution<uint64_t> dist(0, 0xFFFFFFFFFFFFFFFF);

        auto rand64 = []() { return dist(rng); };

        uint64_t part1 = rand64();
        uint64_t part2 = rand64();

        std::stringstream ss;
        ss << std::hex << std::setfill('0')
           << std::setw(8) << static_cast<uint32_t>(part1 >> 32) << "-"
           << std::setw(4) << static_cast<uint16_t>(part1 >> 16) << "-"
           << std::setw(4) << ((static_cast<uint16_t>(part1) & 0x0FFF) | 0x4000) << "-"
           << std::setw(4) << ((static_cast<uint16_t>(part2 >> 48) & 0x3FFF) | 0x8000) << "-"
           << std::setw(12) << (part2 & 0xFFFFFFFFFFFF);

        return ss.str();
    }


    PortalGraphParticle& nbrAtLabel(int label) const;

    void setPortalDistanceFromRoot(Axis axis, int value) {
        _portalDistanceFromRoot[axis] = value;
    }

    int getPortalDistanceFromRoot(Axis axis) const {
        return _portalDistanceFromRoot.at(axis);
    }

    int getPortalDistanceFromGiven(Axis axis,PortalGraphParticle given){
        return abs(_portalDistanceFromRoot.at(axis) - given.getPortalDistanceFromRoot(axis));
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

    bool neighboursDoneConstructingPortal(Axis axis) {
        bool done = true;
        for (int i = 0; i < system.size() - 1; i++) {
            const Particle& p = system.at(i);
            auto pgp = dynamic_cast<PortalGraphParticle&>(const_cast<Particle&> (p));
            done = done && pgp.getPortalDirections(axis).size() > 0;
        }
        return done;
    }

    bool neighboursDoneParentChoice() {
        bool done = true;
        for (int i = 0; i < system.size() - 1; i++) {
            const Particle& p = system.at(i);
            auto pgp = dynamic_cast<PortalGraphParticle&>(const_cast<Particle&> (p));
            done = done && (pgp.parent != -1 || pgp._leader);
        }
        return done;
    }

    bool connectedAmoebot() const {
        bool connected = false;
        for (int i = 0; i < 6; ++i) {
            connected = connected || getInedge(i) != -1;
        }
        return connected;
    }

    void setDistanceSet(Axis axis, bool val) {
        _distanceSet[axis] = val;
    }

    bool getDistanceSet(Axis axis) {
        return _distanceSet.at(axis);
    }

    bool distancesSet() const {
        bool result = true;
        for (const auto& kv : _distanceSet) {
            result = result && kv.second;
        }
        return result;
    }

    bool neighboursFinished() const {
        bool result = true;
        for (int dir = EAST; dir <= SOUTHEAST; dir += 1) {
            if (hasNbrAtLabel(dir)) {
                result = result && nbrAtLabel(dir).distancesSet();
            }
        }
        return result;
    }

    bool operator<(const PortalGraphParticle& other) const {
        // Compare based on x-coordinate first
        if (head.x != other.head.x) {
            return head.x < other.head.x;

        }
        // If x's are equal, compare based on y-coordinate
        return head.y < other.head.y;
    }


    QString directionToString2(Direction dir) {
        switch (dir) {
        case WEST: return "WEST";
        case EAST: return "EAST";
        case SOUTHWEST: return "SOUTHWEST";
        case NORTHEAST: return "NORTHEAST";
        case NORTHWEST: return "NORTHWEST";
        case SOUTHEAST: return "SOUTHEAST";
        default: return "Unknown";
        }
    }

    QString stringifyDirectionVector2(const std::vector<Direction>& vec) {
        QString result = "";
        for (size_t i = 0; i < vec.size(); ++i) {
            result += directionToString2(vec[i]);
            if (i != vec.size() - 1) {
                result += ", ";
            }
        }
        return result;
    }

    void startEulerTour(Axis axis){
        if(eulerDone || !neighboursDoneParentChoice()){
            return;
        }
        eulerDone = true;
        Direction direction;
        Direction nbrDirection;
        int potentialdirection[6]={0,1,2,3,4,5};
        for(int pot : potentialdirection){
            if (hasNbrAtLabel(pot) && nbrAtLabel(pot).parent == (pot + 3) % 6){
                direction =static_cast<Direction>((pot));
                break;
            }
        }
        setOutedge(direction,0);
        nbrDirection = static_cast<Direction>((static_cast<int>(direction)+3)%6);
        nbrAtLabel(direction).eulerTour(0, nbrDirection,axis);
        //rootPruning(); //Ezt vissza kell állítani demo után
    }

    void eulerTour(int value, Direction movedirection, Axis axis){
        setInedge(movedirection, value);
        eulerDone = true; //Ez most csak vizhez kell, majd törölni
        // megkeressük a helyes irányt = direction
        Direction direction;
        int potentialdirection[6]={(movedirection+1) % 6,(movedirection+2) % 6,(movedirection+3) % 6,
                                   (movedirection+4) % 6,(movedirection+5) % 6,movedirection};
        bool directionFound = false;
        for(int pot : potentialdirection){
            if (hasNbrAtLabel(pot) && (nbrAtLabel(pot).parent == (pot + 3) % 6 || pot == parent) && getOutedge(pot) == -1){
                direction =static_cast<Direction>(pot);
                directionFound = true;
                break;
            }
        }
        if (!directionFound) {
            return;
        }
        if(isTarget && !isTargetused){
            value += 1;
            isTargetused = true;
        }
        setOutedge(direction, value);
        Direction nbrDirection;
        nbrDirection = static_cast<Direction>((static_cast<int>(direction)+3)%6);
        nbrAtLabel(direction).eulerTour(value, nbrDirection, axis);
    }

    void rootPruning() {
        noTargetinPath();
        visited = true;
        int potentialdirection[6]={0,1,2,3,4,5};
        for(int pot : potentialdirection){
            if (hasNbrAtLabel(pot) && !nbrAtLabel(pot).visited){
                nbrAtLabel(pot).rootPruning();
            }
        }
    }

    void noTargetinPath(){
        visited = true; //Törölni csak vizualáizáció
        for(int i=0;i<6;i++){
            if(inedge[i] == outedge[i]) {
                inedge[i] = -1;
                outedge[i] = -1; // this means the we cut the edges between in this
            }
        }
    }

    int getInedge(int index) const {
        return inedge[index];
    }

    void setInedge(int index,int value){
        inedge[index]= value;
    }

    int getOutedge(int index) const {
        return outedge[index];
    }

    void setOutedge(int index,int value){
        outedge[index]= value;
    }

    std::map<PortalGraphParticle,std::pair<int, int>> visibility(std::set<PortalGraphParticle>& P,std::set<PortalGraphParticle>& B){
          std::map<PortalGraphParticle, std::pair<int, int>> visiable = {};

            for(PortalGraphParticle b : B ){
                visiable[b] = {-1,-1};
                for(PortalGraphParticle p : P){
                    if(p.getPortalDistanceFromGiven(Y,b) == 0){
                       visiable[b].second = getPortalDistanceFromGiven(Z,b);
                    }
                    else if(p.getPortalDistanceFromGiven(Z,b) == 0){
                        visiable[b].first = getPortalDistanceFromGiven(Y,b);
                    }
                }
            }
          return visiable;
          //-1,-1 azt jelenti hogy nem látható sem y-ból sem z-ből
          // a szülője az lesz amelyik kisebb ha egyik sem 0
          // Lehet úgy kellene megcsinálni hogy nincs a külső for ciklus és majd amikor használjuk akkor hívjuk meg mindegyiknek ezt a funkcióját
          //és akkor csak két int-et kellene visszadnia
    }

    PortalGraphParticle(const Node& head, const int orientation, const bool _leader, std::string portalGraph, AmoebotSystem& system);
    void activate() override;

    int headMarkColor() const override;
    int headMarkDir() const override;
    int tailMarkColor() const override;

    bool isTarget = false;
    bool visited = false;
    bool isTargetused = false;

    // Returns the string to be displayed when this particle is inspected; used to
    // snapshot the current values of this particle's memory at runtime.
    QString inspectionText() const override;

protected:
    // Member variables.
    //They can contain the parallel connections as well
    bool portalSet = false;
    bool eulerDone = false;

private:
    friend class PortalGraphSystem;

    bool _leader; //the leader / root amoebot
    bool _neighboursSet = false; //has gone through distance propagation

    int inedge[6] = {-1,-1,-1,-1,-1,-1};
    int outedge[6] = {-1,-1,-1,-1,-1,-1};

    void calculatePortalDistance();
    void chooseParent();
    void prune();
    void createPortalGraph(Axis axis);
    void initializePortalGraph();
    Direction chooseClosestToLeader(std::vector<Direction>);

    int _headMarkDir = -1;
    Direction parent = NONE;
    std::string _portalGraph = "";
    std::map<Axis, bool> _pascDone;
    std::map<Axis, int> _distanceFromRoot;
    std::map<Axis, std::vector<Direction>> _portalDirections;
    std::map<Axis, int> _portalDistanceFromRoot;
    std::map<Axis,bool> _distanceSet; //distance from root set by neighbour
};

class PortalGraphSystem : public AmoebotSystem {
public:
    // Constructs a system of the specified number of PortalGraphDemoParticles.
    PortalGraphSystem(int numParticles = 30,
                      int targetCount = 1,
                      std::string portalGraph = "",int grid_size = 40);
};

#endif  // AMOEBOTSIM_ALG_DEMO_PORTALGRAPH_H_
