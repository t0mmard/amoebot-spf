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
PortalGraphParticle::Direction ySideA[2] = {PortalGraphParticle::WEST, PortalGraphParticle::NORTHWEST};
PortalGraphParticle::Direction ySideB[2] = {PortalGraphParticle::SOUTHEAST, PortalGraphParticle::EAST};
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



PortalGraphParticle::PortalGraphParticle(const Node &head,
                                         const int orientation, const bool isLeader,
                                         AmoebotSystem &system)
    : AmoebotParticle(head, -1, orientation, system)
{
    _state = getRandColor(isLeader);
    leader = isLeader;
    xPascDone = false;
    yPascDone = false;
    zPascDone = false;
    portalSet = false;
    xDistanceFromRoot = 0;
    yDistanceFromRoot = 0;
    zDistanceFromRoot = 0;
}

void PortalGraphParticle::createPortalGraph(std::vector<Direction>& portalDirections, Direction axis[2], Direction sideA[2], Direction sideB[2], Direction boundaryDirection) {
    //add main axis
    for(int i = 0; i< 2; ++i) {
        Direction dir = axis[i];
        if(hasNbrAtLabel(dir)) {
            portalDirections.push_back(dir);
        }
    }

    //return if there is an amoebot in the boundaryDirection (no parallel connection needed)
    if (std::find(portalDirections.begin(), portalDirections.end(), boundaryDirection) != portalDirections.end()) {
        //we check if the parallel amoebots are on the boundary, if so we connect them
        if (!hasNbrAtLabel(sideA[0]) && hasNbrAtLabel(sideA[1])) {
            portalDirections.push_back(sideA[1]);
        }
        if (!hasNbrAtLabel(sideB[0]) && hasNbrAtLabel(sideB[1])) {
            portalDirections.push_back(sideB[1]);
        }
        return;
    }

    //add the western, northeastern, southeastern most parallel amoebots on both side of the axis
    //sideA
    for(int i = 0; i< 2; ++i) {
        Direction dir = sideA[i];
        if(hasNbrAtLabel(dir)) {
            portalDirections.push_back(dir);
            break;
        }
    }
    //sideB
    for(int i = 0; i< 2; ++i) {
        Direction dir = sideB[i];
        if(hasNbrAtLabel(dir)) {
            portalDirections.push_back(dir);
            break;
        }
    }
}

void PortalGraphParticle::activate()
{
    if (!portalSet) {
        createPortalGraph(portalXDirections, xAxis, xSideA, xSideB, WEST);
        createPortalGraph(portalYDirections, yAxis, ySideA, ySideB, SOUTHWEST);
        createPortalGraph(portalZDirections, xAxis, xSideA, xSideB, SOUTHEAST);
        portalSet = true;
    } else { //set amoebot distance on different axis'
        if (leader && !xPascDone && !yPascDone && !zPascDone) {
            for (auto dir : portalXDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.xDistanceFromRoot == 0) {
                    nbrAtLabel(dir).xDistanceFromRoot = xDistanceFromRoot + 1;
                }
            }
            for (auto dir : portalYDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.yDistanceFromRoot == 0) {
                    nbrAtLabel(dir).yDistanceFromRoot = yDistanceFromRoot + 1;
                }
            }
            for (auto dir : portalZDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.zDistanceFromRoot == 0) {
                    nbrAtLabel(dir).zDistanceFromRoot = zDistanceFromRoot + 1;
                }
            }
            xPascDone = true;
            yPascDone = true;
            zPascDone = true;
        }

        if (xDistanceFromRoot != 0 && !xPascDone) {
            for (auto dir : portalXDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.xDistanceFromRoot == 0 && !nbr.leader) {
                    nbrAtLabel(dir).xDistanceFromRoot = xDistanceFromRoot + 1;
                }
            }
            xPascDone = true;
        }

        if (yDistanceFromRoot != 0 && !yPascDone) {
            for (auto dir : portalYDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.yDistanceFromRoot == 0 && !nbr.leader) {
                    nbrAtLabel(dir).yDistanceFromRoot = yDistanceFromRoot + 1;
                }
            }
            yPascDone = true;
        }

        if (zDistanceFromRoot != 0 && !zPascDone) {
            for (auto dir : portalZDirections) {
                auto nbr = nbrAtLabel(dir);
                if (nbr.zDistanceFromRoot == 0 && !nbr.leader) {
                    nbrAtLabel(dir).zDistanceFromRoot = zDistanceFromRoot + 1;
                }
            }
            zPascDone = true;
        }
    }
}

PortalGraphParticle& PortalGraphParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<PortalGraphParticle>(label);
}


int PortalGraphParticle::headMarkColor() const
{
    switch (_state)
    {
    case State::Red:
        return 0xff0000;
    case State::Nothing:
        return -1;
    }

    return -1;
}

int PortalGraphParticle::tailMarkColor() const
{
    return headMarkColor();
}



QString PortalGraphParticle::inspectionText() const
{
    QString text;
    text += "X portal graph neighbours: ";
    text += stringifyDirectionVector(portalXDirections);
    text += "\n";
    text += "Y portal graph neighbours: ";
    text += stringifyDirectionVector(portalYDirections);
    text += "\n";
    text += "Z portal graph neighbours: ";
    text += stringifyDirectionVector(portalZDirections);
    text += "\n";
    text += "X distance from root: ";
    text += QString::number(xDistanceFromRoot);
    text += "\n";
    text += "Y distance from root: ";
    text += QString::number(yDistanceFromRoot);
    text += "\n";
    text += "Z distance from root: ";
    text += QString::number(zDistanceFromRoot);
    text += "\n";

    return text;
}

PortalGraphParticle::State PortalGraphParticle::getRandColor(const bool leader) const
{
    // Select colours that are in the portal graph
    if (leader)
    {
        return static_cast<State>(0);
    }
    return static_cast<State>(1);
}

PortalGraphSystem::PortalGraphSystem(int numParticles)
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

        insert(new PortalGraphParticle(node, 0, i == leader, *this));
        ++i;
    }
}
