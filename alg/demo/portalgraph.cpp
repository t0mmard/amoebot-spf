/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/demo/portalgraph.h"
#include "alg/shortpathforest.h"
#include <iostream>
#include <string>
#include <cstdlib>

//for visualization only
int maxDistance = 0;
int numberOfParticles = 0;
int numberOfTargets = 0;

//helper functions
bool contains(const std::vector<int>& vec, int num) {
    return std::find(vec.begin(), vec.end(), num) != vec.end();
}

QString directionToString(Direction dir) {
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

QString stringifyDirectionVector(const std::vector<Direction>& vec) {
    QString result = "";
    for (size_t i = 0; i < vec.size(); ++i) {
        result += "\n" + directionToString(vec[i]);
        if (i != vec.size() - 1) {
            result += ", ";
        }
    }
    return result;
}

unsigned int getColor(int value, int limit) {
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
    _leader = isLeader;

    _distanceSet[X] = isLeader;
    _distanceSet[Y] = isLeader;
    _distanceSet[Z] = isLeader;

    setPortalDistanceFromRoot(X, 0);
    setPortalDistanceFromRoot(Y, 0);
    setPortalDistanceFromRoot(Z, 0);

    _portalDirections[X] = std::vector<Direction>();
    _portalDirections[Y] = std::vector<Direction>();
    _portalDirections[Z] = std::vector<Direction>();
}


void PortalGraphParticle::activate()
{
    initializePortalGraph();
    calculatePortalDistance();
    chooseParent();

    prune();
}

void PortalGraphParticle::prune() {
    /*if (!_leader) { Ez az eredeti, demo után visszaállítani
         return;
    }
    startEulerTour(X);
    */
    if (!_leader && !eulerDone) {
         return;
    } else if (!_leader && eulerDone && !visited) {
        noTargetinPath();
    } else if (_leader) {
        startEulerTour(X);
        noTargetinPath();
    }
}


void PortalGraphParticle::initializePortalGraph() {
    if (getPortalDirections(Axis::X).size() != 0) {
        return;
    }
    for (int axis = X; axis <= Z; axis += 1) {
        createPortalGraph(static_cast<Axis>(axis));
    }
    portalSet = true;
}

void PortalGraphParticle::createPortalGraph(Axis axis) {
    AxisData axisData = axisMap.at(axis);
    //add main axis
    for(int i = 0; i< 2; ++i) {
        Direction dir = axisData.axis[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
        }
    }

    //return if there is an amoebot in the boundaryDirection (no parallel connection needed)
    if (neighbourExists(axis, axisData.boundaryDirection)) {
        //we check if the parallel amoebots are on the boundary, if so we connect them
        if (!hasNbrAtLabel(axisData.sideA[0]) && hasNbrAtLabel(axisData.sideA[1])) {
            pushPortalDirections(axis, axisData.sideA[1]);
        }
        if (!hasNbrAtLabel(axisData.sideB[0]) && hasNbrAtLabel(axisData.sideB[1])) {
            pushPortalDirections(axis, axisData.sideB[1]);
        }
        return;
    }

    //add the western, northeastern, southeastern most parallel amoebots on both side of the axis
    //sideA
    for(int i = 0; i< 2; ++i) {
        Direction dir = axisData.sideA[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
            break;
        }
    }
    //sideB
    for(int i = 0; i< 2; ++i) {
        Direction dir = axisData.sideB[i];
        if(hasNbrAtLabel(dir)) {
            pushPortalDirections(axis, dir);
            break;
        }
    }
}



void PortalGraphParticle::calculatePortalDistance() {
    if (_neighboursSet) {
        return;
    }
    bool neighboursSet = true;
    for (int axis = X; axis <= Z; axis += 1) {
        if (!getDistanceSet(static_cast<Axis>(axis))) {
            neighboursSet = false;
            continue;
        }
        AxisData axisData = axisMap.at(static_cast<Axis>(axis));
        for (auto dir : getPortalDirections(static_cast<Axis>(axis))) {
            if (nbrAtLabel(dir).getDistanceSet(static_cast<Axis>(axis))) continue;
            if (std::find(axisData.axis.begin(), axisData.axis.end(), dir) != axisData.axis.end()) {
                nbrAtLabel(dir).setPortalDistanceFromRoot(static_cast<Axis>(axis), getPortalDistanceFromRoot(static_cast<Axis>(axis)));
            } else {
                nbrAtLabel(dir).setPortalDistanceFromRoot(static_cast<Axis>(axis), getPortalDistanceFromRoot(static_cast<Axis>(axis)) + 1);
            }
            nbrAtLabel(dir).setDistanceSet(static_cast<Axis>(axis), true);
        }
    }
    _neighboursSet = neighboursSet;
}

void PortalGraphParticle::chooseParent() {
    if (_leader || !_neighboursSet || !neighboursFinished() || parent != NONE) {
        return;
    }
    // For visualization only
    int distance = (getPortalDistanceFromRoot(X) + getPortalDistanceFromRoot(Y) + getPortalDistanceFromRoot(Z)) / 2;
    if (distance > maxDistance) maxDistance = distance;
    //For visualization only
    for (int dir = EAST; dir <= SOUTHEAST; dir += 1) {
        if (hasNbrAtLabel(dir)) {
            int candidate = (getPortalDistanceFromRoot(X) - nbrAtLabel(dir).getPortalDistanceFromRoot(X))
                    + (getPortalDistanceFromRoot(Y) - nbrAtLabel(dir).getPortalDistanceFromRoot(Y))
                    + (getPortalDistanceFromRoot(Z) - nbrAtLabel(dir).getPortalDistanceFromRoot(Z));
            if (candidate == 2) {
                parent = static_cast<Direction>(dir);
                _headMarkDir = dir;
            }
        }
    }
}


PortalGraphParticle& PortalGraphParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<PortalGraphParticle>(label);
}


int PortalGraphParticle::headMarkColor() const
{
    if (_leader) {
        return 0x0000FF;
    } else if (isTarget && numberOfTargets < numberOfParticles - 1) {
        return 0xFF10F0;
    } else if(visited && !connectedAmoebot() && parent != NONE){
        return 0xA9A9A9;
    } else if (distancesSet()) {
        return getColor(((getPortalDistanceFromRoot(X) + getPortalDistanceFromRoot(Y) + getPortalDistanceFromRoot(Z))/2), maxDistance);
    } else {
        return -1;
    }
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
    text += "\n\n";
    text += "Y portal graph neighbours: ";
    text += stringifyDirectionVector(getPortalDirections(Axis::Y));
    text += "\n\n";
    text += "Z portal graph neighbours: ";
    text += stringifyDirectionVector(getPortalDirections(Axis::Z));
    text += "\n\n\n";

    text += "X portal distance: ";
    text += QString::number(getPortalDistanceFromRoot(Axis::X));
    text += "\n";

    text += "Y portal distance: ";
    text += QString::number(getPortalDistanceFromRoot(Axis::Y));
    text += "\n";

    text += "Z portal distance: ";
    text += QString::number(getPortalDistanceFromRoot(Axis::Z));
    text += "\n";
    text += "\n";

    text += "Shortest path length: ";
    text += QString::number((getPortalDistanceFromRoot(Axis::X)+getPortalDistanceFromRoot(Axis::Y)+getPortalDistanceFromRoot(Axis::Z))/2);
    text += "\n";
    text += "\n";
    text += "Outedge: ";

    QString strList2;
    for (int i = 0; i < 6; ++i) {
        strList2 += QString::number(getOutedge(i));
        strList2 += ", ";
    }
    text += strList2;
    text += "\n";
    text += "\n";

    text += "Inedge: ";

    QString strList;
    for (int i = 0; i < 6; ++i) {
        strList += QString::number(getInedge(i));
        strList += ", ";
    }
    text += strList;
    text += "\n";
    text += "\n";


    text += "Parent amoebot: ";
    text += directionToString(parent);
    text += "\n";

    return text;
}


PortalGraphSystem::PortalGraphSystem(int numParticles, int targetCount, std::string portalGraph,int grid_size)
{
    //For visualization only
    maxDistance = 0;
    numberOfTargets = targetCount;
    numberOfParticles = numParticles;
    //For visualization only
    std::set<Node> occupied;
    occupied.insert(Node(grid_size/2,grid_size/2 ));

    std::set<Node> graph;

    for (int i = 0; i < grid_size; i++)
    {
        for (int j = 0; j < grid_size; j++)
        {
            if (!(i == grid_size/2 && j == grid_size/2))
                graph.insert(Node(i, j));
        }
    }

    // Initialize the candidate positions set.
    std::set<Node> candidates;
    for (int i = 0; i < 6; ++i)
    {
        candidates.insert(Node(grid_size/2, grid_size/2).nodeInDir(i));
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

    // Generate indices
    std::vector<int> indices(numParticles);
    std::iota(indices.begin(), indices.end(), 0); // [0, 1, ..., numParticles-1]

    // Shuffle
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    // Pick targets
    std::vector<int> targetIndices(indices.begin() + 1, indices.begin() + 1 + targetCount);

    int i = 0;

    for (const auto &node : occupied)
    {
        auto newParticle = new PortalGraphParticle(node, 0, i == indices[0], portalGraph, *this);
        newParticle->isTarget = std::find(targetIndices.begin(), targetIndices.end(), i) != targetIndices.end();
        insert(newParticle);
        ++i;
    }
}
