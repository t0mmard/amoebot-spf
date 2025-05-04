/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/demo/spf.h"
#include <iostream>
#include <string>
#include <cstdlib>

//for visualization only
int maxDistance = 0;
int numberOfParticles = 0;
int numberOfTargets = 0;
int numberOfSources = 0;
int numberOfCuts = 0;
int currentId = 0;


//helper functions
bool contains(const std::vector<int>& vec, int num) {
    return std::find(vec.begin(), vec.end(), num) != vec.end();
}

uint32_t hashID(int id) {
    uint32_t x = static_cast<uint32_t>(id);
    x ^= x >> 16;
    x *= 0x85ebca6b;
    x ^= x >> 13;
    x *= 0xc2b2ae35;
    x ^= x >> 16;
    return x;
}

// Return a color in 0xRRGGBB format
uint32_t getColorFromID(int id) {
    uint32_t hash = hashID(id);

    int r = (hash >> 16) & 0xFF;
    int g = (hash >> 8)  & 0xFF;
    int b = (hash)       & 0xFF;

    return (r << 16) | (g << 8) | b;
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


ShortestPathForestParticle::ShortestPathForestParticle(const Node &head,
                                         const int orientation,
                                         const bool isSource,
                                         AmoebotSystem &system)
    : AmoebotParticle(head, -1, orientation, system)
{
    _source = isSource;

    _distanceSet[X] = isSource;
    _distanceSet[Y] = isSource;
    _distanceSet[Z] = isSource;

    setPortalDistanceFromRoot(X, 0);
    setPortalDistanceFromRoot(Y, 0);
    setPortalDistanceFromRoot(Z, 0);

    _portalDirections[X] = std::vector<Direction>();
    _portalDirections[Y] = std::vector<Direction>();
    _portalDirections[Z] = std::vector<Direction>();
}


void ShortestPathForestParticle::activate()
{
    if (numberOfSources == 1) {
        initializePortalGraph();
        calculatePortalDistance();
        chooseParent();

        prune();
    } else {
        initializePortalGraph();
       if(_source){
            sendSignal();
       }
       //ütemezés
       if(hasSourceOnPortal && !hasNbrAtLabel(3) && !cutDone){
            numberOfCuts += cutPortal(true);
       }

       if(numberOfCuts == numberOfSources && !regionSet && hasSourceOnPortal && !hasNbrAtLabel(3)){
          setRegion(++currentId,true,head.y);
          setRegion(++currentId,false,head.y);
       }

    }
}

void ShortestPathForestParticle::prune() {
    if (!_source && !eulerDone) {
        return;
    } else if (!_source && eulerDone && !visited) {
        noTargetinPath();
    } else if (_source) {
        startEulerTour(X);
        noTargetinPath();
    }
}


void ShortestPathForestParticle::initializePortalGraph() {
    if (getPortalDirections(Axis::X).size() != 0) {
        return;
    }
    for (int axis = X; axis <= Z; axis += 1) {
        createPortalGraph(static_cast<Axis>(axis));
    }
    portalSet = true;
}

void ShortestPathForestParticle::createPortalGraph(Axis axis) {
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



void ShortestPathForestParticle::calculatePortalDistance() {
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

void ShortestPathForestParticle::chooseParent() {
    if (_source || !_neighboursSet || !neighboursFinished() || parent != NONE) {
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


ShortestPathForestParticle& ShortestPathForestParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<ShortestPathForestParticle>(label);
}


int ShortestPathForestParticle::headMarkColor() const
{
    if (_source) {
        return 0x0000FF;
    }

    if (id[0] != -1) {
        return getColorFromID(id[0]);
    }
    return -1;
    /*else if (isTarget && numberOfTargets < numberOfParticles - 1) {
        return 0xFF10F0;
    } else if(visited && !connectedAmoebot() && parent != NONE){
        return 0xA9A9A9;
    } else if (distancesSet()) {
        return getColor(((getPortalDistanceFromRoot(X) + getPortalDistanceFromRoot(Y) + getPortalDistanceFromRoot(Z))/2), maxDistance);
    } else {
        return -1;
    }*/
}

int ShortestPathForestParticle::headMarkDir() const {
    return _headMarkDir;
}

int ShortestPathForestParticle::tailMarkColor() const
{
    return headMarkColor();
}



QString ShortestPathForestParticle::inspectionText() const
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

    text += "Has Source in portal: ";
    text += QString::number(hasSourceOnPortal);
    text += "\n";
    text += "\n";

    text += "North cut portal: ";
    text += QString::number(northCut);
    text += "\n";
    text += "\n";

    text += "South cut portal: ";
    text += QString::number(southCut);
    text += "\n";
    text += "\n";


    text += "Region ids: ";

    QString strList3;
    for (int i = 0; i < 3; ++i) {
        strList3 += QString::number(id[i]);
        strList3 += ", ";
    }
    text += strList3;
    text += "\n";
    text += "\n";

    return text;
}

bool dfsPathExists(const std::set<Node>& graph, const Node& start, const Node& target,std::set<Node>& visited){
    if (start.x == target.x && start.y == target.y)
            return true; // Found the target

    visited.insert(start);

    // Possible moves: left, right, up, down (assuming a grid-based movement)
    std::vector<Node> neighbors = {
        {start.x + 1, start.y}, {start.x - 1, start.y},
        {start.x, start.y + 1}, {start.x, start.y - 1},
        {start.x+1,start.y-1}, {start.x-1, start.y+1}
    };

    for (const Node& neighbor : neighbors) {
        if (graph.find(neighbor) != graph.end() && visited.find(neighbor) == visited.end()) {
            if (dfsPathExists(graph, neighbor, target, visited))
                return true;
        }
    }
    return false;
}



ShortestPathForestSystem::ShortestPathForestSystem(int numParticles, int sourceCount, int targetCount)
{
    //For visualization only
    int grid_size = 40;
    maxDistance = 0;
    numberOfTargets = targetCount;
    numberOfParticles = numParticles;
    numberOfSources = sourceCount;
    numberOfCuts = 0;
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
    std::vector<int> sourceIndices(indices.begin(), indices.begin() + sourceCount);
    std::vector<int> targetIndices(indices.begin() + sourceCount, indices.begin() + sourceCount + targetCount);

    int i = 0;

    for (const auto &node : occupied)
    {
        auto newParticle = new ShortestPathForestParticle(node, 0,
                                                   std::find(sourceIndices.begin(), sourceIndices.end(), i) != sourceIndices.end(),
                                                   *this);
        newParticle->isTarget = std::find(targetIndices.begin(), targetIndices.end(), i) != targetIndices.end();
        insert(newParticle);
        ++i;
    }
}
