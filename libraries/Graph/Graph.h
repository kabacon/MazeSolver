/*
	Graph.h - Defines a graph data structure and some methods for
	for an Arduino maze solving robot. Methods are defined to add 
	nodes, find shortest paths, and follow that path. Shortest 
	path finding uses Dijkstra's algorithm.
*/

#ifndef GRAPH
#define GRAPH

#include "Node.h"
#include <vector>
#include <list>
#include <set>
#include <map>

using std::vector;
using std::list;
using std::set;
// Can't use 

class Graph {
private: 
	// Nodes and edges of the graph
	vector<Node*> nodes;
	vector<Edge*> edges;

	Node* startNode;
	Node* finishNode;
	Node* currentNode;
	Node* prevNode;

	// The path that should be followed
	list<Node*> path;

	// Structures for Dijkstra's shortest path algorithm
	set<Node*> unvisited;
	set<Node*> visited;
	std::map<Node*, Node*> previous;
	std::map<Node*, long> distance;


public:
	// Inlined
	void foundFinish() { finishNode = currentNode; }
	vector<Node*>& getNodes()	{ return nodes; }
	vector<Edge*>& getEdges()	{ return edges; }
	list<Node*>& getPath()		{ return path;	}

	// Defined in Graph.cpp
	Graph(bool north, bool south, bool east, bool west);
	~Graph();

	void addNode(bool forward, bool left, bool right,  
		int direction, long length);
	Node* getStart()	{ return startNode; }
	Node* getFinish()	{ return finishNode; }
	Node* getCurrent()	{ return currentNode; }
	int advancePath();
	void calculateShortestPath(Node* start, Node* finish);

private:
	// Helper methods
	Node* getClosest();
	void calculateDistances(Node* n);
	float getDistance(Node* n);
};

#endif