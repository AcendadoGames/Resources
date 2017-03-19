
#pragma once

#include <vector>
#include <functional>


struct Vec2i
{
    int x, y;

    bool operator == (const Vec2i& coordinates_);
	bool operator != (const Vec2i& coordinates_);
};

using uint = unsigned int;
using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
using CoordinateList = std::vector<Vec2i>;

struct Node
{
    uint G, H;
    Vec2i coordinates;
    Node *parent;

    Node(Vec2i & coord_, Node *parent_ = nullptr);
	Node();
    uint getScore();
};

using NodeSet = std::vector<Node*>;
using NodeBuffer = std::vector < Node > ;

class Pathfinder
{
    bool detectCollision(const Vec2i & coordinates_);
    Node* findNodeOnList(const NodeSet& nodes_, const Vec2i & coordinates_);
    void releaseNodes(NodeSet& nodes_);

public:
    Pathfinder();
    void static setWorldSize(Vec2i worldSize_);
    void setDiagonalMovement(const bool enable_);
    void setHeuristic(const HeuristicFunction heuristic_);
	CoordinateList findPath(const Vec2i & source_, const Vec2i & target_);
    void static addCollision(const Vec2i& coordinates_);
    void static removeCollision(const Vec2i & coordinates_);
    void static clearCollisions();
	static CoordinateList walls;
private:
    HeuristicFunction heuristic;
	CoordinateList direction;
    static Vec2i worldSize;
    uint directions;
	static NodeBuffer Nodes;
};

class Heuristic
{
    static Vec2i getDelta(const Vec2i & source_, const Vec2i & target_);

public:
    static uint manhattan(Vec2i & source_, Vec2i & target_);
    static uint euclidean(Vec2i & source_, Vec2i & target_);
    static uint octagonal(Vec2i & source_, Vec2i & target_);
};

