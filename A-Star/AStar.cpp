#include "AStar.hpp"
#include <algorithm>

using namespace std::placeholders;
NodeBuffer Pathfinder::Nodes;
Vec2i Pathfinder::worldSize;
CoordinateList Pathfinder::walls;

bool Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}
bool Vec2i::operator != (const Vec2i& coordinates_)
{
	return (x != coordinates_.x || y != coordinates_.y);
}

Vec2i operator + (const Vec2i& left_, const Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

Node::Node(Vec2i & coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

Node::Node(){}


uint Node::getScore()
{
    return G + H;
}

Pathfinder::Pathfinder()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = 
	{ 
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void Pathfinder::setWorldSize(const Vec2i worldSize_)
{
    worldSize = worldSize_;
	Pathfinder::Nodes.resize(worldSize.x * worldSize.y);
	for (int i = 0; i < worldSize.x; ++i)
	{
		for (int j = 0; j < worldSize.y; ++j)
		{
			Pathfinder::Nodes[worldSize.x * j + i].coordinates = { i, j };
			Pathfinder::Nodes[worldSize.x * j + i].G = 0;
			Pathfinder::Nodes[worldSize.x * j + i].H = 0;
			Pathfinder::Nodes[worldSize.x * j + i].parent = nullptr;
		}
	}
}

void Pathfinder::setDiagonalMovement(const bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void Pathfinder::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void Pathfinder::addCollision(const Vec2i &coordinates_)
{
    walls.push_back(coordinates_);
}

void Pathfinder::removeCollision(const Vec2i & coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void Pathfinder::clearCollisions()
{
    walls.clear();
}

CoordinateList Pathfinder::findPath(const Vec2i & source_, const Vec2i & target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
	openSet.push_back(&Pathfinder::Nodes[Pathfinder::worldSize.x * target_.y + target_.x]);
	openSet[0]->parent = nullptr;
	openSet[0]->G = 0;
	openSet[0]->H = heuristic(openSet[0]->coordinates, source_);

    while (!openSet.empty()) 
	{
        current = *openSet.begin();
        for (const auto& node : openSet) 
		{
            if (node->getScore() <= current->getScore()) 
			{
                current = node;
            }
        }

        if (current->coordinates == source_) 
		{
            break;
        }

		closedSet.push_back(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

        for (uint i = 0; i < directions; ++i) 
		{
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) 
			{
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) 
			{
				successor = &Pathfinder::Nodes[Pathfinder::worldSize.x * newCoordinates.y + newCoordinates.x];
				successor->parent = current;
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, source_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) 
			{
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
	unsigned node_count = 0;
	auto temp = current;
	while (temp != nullptr) 
	{
		node_count++;
		temp = temp->parent;
    }

	path.reserve(node_count);

	while (current != nullptr) 
	{
		path.push_back(current->coordinates);
		current = current->parent;
	}

	return path;
}

Node* Pathfinder::findNodeOnList(const NodeSet& nodes_, const Vec2i & coordinates_)
{
    for (const auto &node : nodes_) 
	{
        if (node->coordinates == coordinates_) 
		{
            return node;
        }
    }
    return nullptr;
}

void Pathfinder::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) 
	{
        it = nodes_.erase(it);
    }
}

bool Pathfinder::detectCollision(const Vec2i & coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) 
	{
        return true;
    }
    return false;
}

Vec2i Heuristic::getDelta(const Vec2i & source_, const Vec2i & target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

uint Heuristic::manhattan(Vec2i & source_, Vec2i & target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

uint Heuristic::euclidean(Vec2i & source_, Vec2i & target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint Heuristic::octagonal(Vec2i & source_, Vec2i & target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}