/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/demo/portalgraph.h"
#include "alg/shortpathforest.h"
#include <iostream>
#include <string>

//Portal axis and the amoebot order to check on each side for parallel connections
//x portal
PortalGraphParticle::Direction xAxis[2] = {PortalGraphParticle::WEST, PortalGraphParticle::EAST};
PortalGraphParticle::Direction xSideA[2] = {PortalGraphParticle::NORTHWEST, PortalGraphParticle::NORTHEAST};
PortalGraphParticle::Direction xSideB[2] = {PortalGraphParticle::SOUTHWEST, PortalGraphParticle::SOUTHEAST};
//y portal
PortalGraphParticle::Direction yAxis[2] = {PortalGraphParticle::SOUTHWEST, PortalGraphParticle::NORTHEAST};
PortalGraphParticle::Direction ySideA[2] = {PortalGraphParticle::NORTHWEST, PortalGraphParticle::WEST};
PortalGraphParticle::Direction ySideB[2] = {PortalGraphParticle::EAST, PortalGraphParticle::SOUTHEAST};
//z portal
PortalGraphParticle::Direction zAxis[2] = {PortalGraphParticle::SOUTHEAST, PortalGraphParticle::NORTHWEST};
PortalGraphParticle::Direction zSideA[2] = {PortalGraphParticle::SOUTHWEST, PortalGraphParticle::WEST};
PortalGraphParticle::Direction zSideB[2] = {PortalGraphParticle::EAST, PortalGraphParticle::NORTHEAST};


//helper functions
bool contains(const std::vector<int>& vec, int num) {
    return std::find(vec.begin(), vec.end(), num) != vec.end();
}

QString directionToString(PortalGraphParticle::Direction dir) {
    switch (dir) {
        case PortalGraphParticle::WEST: return "WEST";
        case PortalGraphParticle::EAST: return "EAST";
        case PortalGraphParticle::SOUTHWEST: return "SOUTHWEST";
        case PortalGraphParticle::NORTHEAST: return "NORTHEAST";
        case PortalGraphParticle::NORTHWEST: return "NORTHWEST";
        case PortalGraphParticle::SOUTHEAST: return "SOUTHEAST";
        default: return "Unknown";
    }
}

QString stringifyDirectionVector(const std::vector<PortalGraphParticle::Direction>& vec) {
    QString result = "";
    for (size_t i = 0; i < vec.size(); ++i) {
        result += directionToString(vec[i]);
        if (i != vec.size() - 1) {
            result += ", ";
        }
    }
    return result;
}

unsigned int getColor(int value, int limit) {
    value = std::min(std::max(value, 0), limit);

    int red = static_cast<int>((value / static_cast<double>(limit)) * 255);
    int green = static_cast<int>((1.0 - (value / static_cast<double>(limit))) * 255);

    return (red << 16) | (green << 8);
}


PortalGraphParticle::PortalGraphParticle(const Node &head,
                                         const int orientation,
                                         const bool isLeader,
                                         const std::string portalGraph,
                                         AmoebotSystem &system)
    : AmoebotParticle(head, -1, orientation, system)
{
    _portalGraph = portalGraph;
    leader = isLeader;
    setPascDone(Axis::X, false);
    setPascDone(Axis::Y, false);
    setPascDone(Axis::Z, false);

    setDistanceFromRoot(Axis::X, 0);
    setDistanceFromRoot(Axis::Y, 0);
    setDistanceFromRoot(Axis::Z, 0);

    _portalDirections[Axis::X] = std::vector<Direction>();
    _portalDirections[Axis::Y] = std::vector<Direction>();
    _portalDirections[Axis::Z] = std::vector<Direction>();

    getPortalDirections(Axis::X).push_back(Direction::EAST);
}


void PortalGraphParticle::activate()
{
    if (!portalSet) { // select neighbours for all portal graphs
        initializePortalGraph();
    } else if (!allPascDone()){ //propagate pasc distance on each portal graph
        calculatePasc();
    } else {
        chooseParent();
    }
}

void PortalGraphParticle::initializePortalGraph() {
    createPortalGraph(Axis::X, xAxis, xSideA, xSideB, WEST);
    createPortalGraph(Axis::Y, yAxis, ySideA, ySideB, NORTHEAST);
    createPortalGraph(Axis::Z, zAxis, zSideA, zSideB, SOUTHEAST);
    portalSet = true;
}

void PortalGraphParticle::createPortalGraph(Axis axis, Direction portalAxis[2], Direction sideA[2], Direction sideB[2], Direction boundaryDirection) {
    //add main axis
    for(int i = 0; i< 2; ++i) {
        Direction dir = portalAxis[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
        }
    }

    //return if there is an amoebot in the boundaryDirection (no parallel connection needed)
    if (neighbourExists(axis, boundaryDirection)) {
        //we check if the parallel amoebots are on the boundary, if so we connect them
        if (!hasNbrAtLabel(sideA[0]) && hasNbrAtLabel(sideA[1])) {
            pushPortalDirections(axis, sideA[1]);
        }
        if (!hasNbrAtLabel(sideB[0]) && hasNbrAtLabel(sideB[1])) {
            pushPortalDirections(axis, sideB[1]);
        }
        return;
    }

    //add the western, northeastern, southeastern most parallel amoebots on both side of the axis
    //sideA
    for(int i = 0; i< 2; ++i) {
        Direction dir = sideA[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
            break;
        }
    }
    //sideB
    for(int i = 0; i< 2; ++i) {
        Direction dir = sideB[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
            break;
        }
    }
}


void PortalGraphParticle::calculatePasc()
{
    if (leader && !allPascDone()) {
        propagateDistance(Axis::X);
        propagateDistance(Axis::Y);
        propagateDistance(Axis::Z);
    }
    if (getDistanceFromRoot(Axis::X) != 0 && !isPascDone(Axis::X)) {
        propagateDistance(Axis::X);
    }

    if (getDistanceFromRoot(Axis::Y) != 0 && !isPascDone(Axis::Y)) {
        propagateDistance(Axis::Y);
    }

    if (getDistanceFromRoot(Axis::Z) != 0 && !isPascDone(Axis::Z)) {
        propagateDistance(Axis::Z);
    }
}


void PortalGraphParticle::propagateDistance(Axis axis)
{
    int i = 0;
    for (auto dir : getPortalDirections(axis)) {
        if (nbrAtLabel(dir).getDistanceFromRoot(axis) == 0 && !nbrAtLabel(dir).leader) {
            int distanceFromRoot = getDistanceFromRoot(axis) + 1;
            nbrAtLabel(dir).setDistanceFromRoot(axis, distanceFromRoot);
        }
    }
    setPascDone(axis, true);
}

void PortalGraphParticle::chooseParent() {
    Axis closestPortal = getPortalClosestToRoot();
    _headMarkDir = getDirCloserToRoot(closestPortal);
}

PortalGraphParticle& PortalGraphParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<PortalGraphParticle>(label);
}


int PortalGraphParticle::headMarkColor() const
{
    if (leader) {
      return 0x0000FF;
    } else if (allPascDone()) {
        if (_portalGraph == "x") {
            return getColor(getDistanceFromRoot(Axis::X), 30);
        } else if (_portalGraph == "y") {
            return getColor(getDistanceFromRoot(Axis::Y), 30);
        } else if (_portalGraph == "z") {
            return getColor(getDistanceFromRoot(Axis::Z), 30);
        }
    }

    return -1;
}

int PortalGraphParticle::headMarkDir() const {
    return _headMarkDir;
}

int PortalGraphParticle::tailMarkColor() const
{
    return headMarkColor();
}



QString PortalGraphParticle::inspectionText() const
{
    QString text;
    text += "X portal graph neighbours: ";
    text += stringifyDirectionVector(getPortalDirections(Axis::X));
    text += "\n";
    text += "Y portal graph neighbours: ";
    text += stringifyDirectionVector(getPortalDirections(Axis::Y));
    text += "\n";
    text += "Z portal graph neighbours: ";
    text += stringifyDirectionVector(getPortalDirections(Axis::Z));
    text += "\n";
    text += "X distance from root: ";
    text += QString::number(getDistanceFromRoot(Axis::X));
    text += "\n";
    text += "Y distance from root: ";
    text += QString::number(getDistanceFromRoot(Axis::Y));
    text += "\n";
    text += "Z distance from root: ";
    text += QString::number(getDistanceFromRoot(Axis::Z));
    text += "\n";

    return text;
}

PortalGraphSystem::PortalGraphSystem(int numParticles, std::string portalGraph)
{
    std::set<Node> occupied;
    occupied.insert(Node(16, 16));
    int grid_size = 32;
    std::set<Node> graph;

    for (int i = 0; i < grid_size; i++)
    {
        for (int j = 0; j < grid_size; j++)
        {
            if (!(i == 16 && j == 16))
                graph.insert(Node(i, j));
        }
    }

    // Initialize the candidate positions set.
    std::set<Node> candidates;
    for (int i = 0; i < 6; ++i)
    {
        candidates.insert(Node(16, 16).nodeInDir(i));
    }

    // Add all other particles using the random tree algorithm.
    int particlesAdded = 1;
    while (particlesAdded < numParticles && !candidates.empty())
    {
        // Pick a random candidate node.
        int randIndex = randInt(0, candidates.size());
        Node randCand;
        for (auto cand = candidates.begin(); cand != candidates.end(); ++cand)
        {
            if (randIndex == 0)
            {
                randCand = *cand;
                candidates.erase(cand);
                break;
            }
            else
            {
                randIndex--;
            }
        }

        // Is there any hole if we put the node down?
        graph.erase(randCand);
        bool noHole = true;

        for (int i = 0; i < 6; ++i)
        {
            std::set<Node> visited;
            noHole = (noHole && dfsPathExists(graph, randCand.nodeInDir(i), Node(0, 0), visited));
        }

        if (noHole)
        {
            occupied.insert(randCand);
            particlesAdded++;

            // Add new candidates.
            for (int i = 0; i < 6; ++i)
            {
                if ((occupied.find(randCand.nodeInDir(i)) == occupied.end() && randCand.nodeInDir(i).x > 0 && randCand.nodeInDir(i).y > 0 &&
                     randCand.nodeInDir(i).x < grid_size - 1 && randCand.nodeInDir(i).y < grid_size - 1))
                {
                    candidates.insert(randCand.nodeInDir(i));
                }
            }
        }
        else
        {
            graph.insert(randCand);
        }
    }

    int leader = randInt(0, numParticles);
    int i = 0;

    for (const auto &node : occupied)
    {

        insert(new PortalGraphParticle(node, 0, i == leader, portalGraph, *this));
        ++i;
    }
}
