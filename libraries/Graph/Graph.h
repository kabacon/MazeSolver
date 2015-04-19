#ifndef GRAPH
#define GRAPH

#include "Node.h"
#include <vector>
#include <set>
#include <map>

class Graph {
private:
	// Nodes and edges of the graph
	std::vector<Node*> nodes;
	std::vector<Edge*> edges;

	Node* startNode;
	Node* finishNode;
	Node* currentNode;
	Node* prevNode;

	// Structures for Dijkstra's shortest path algorithm
	std::set<Edge*> unvisited;
	std::set<Edge*> visited;
	std::map<Edge*, Edge*> previous;
	std::map<Edge*, int> distance;

public:
	Graph(bool north, bool south, bool east, bool west);
	~Graph();

	void addNode(bool north, bool south, bool east, bool west, 
		int direction, int length);
	void foundFinish() { finishNode = currentNode; }
	std::vector<Node*>& getNodes()	{ return nodes; }
	std::vector<Edge*>&	getEdges()	{ return edges; }
};

#endif