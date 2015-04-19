#include "Graph.h"

Graph::Graph(bool north, bool south, bool east, bool west) {
	startNode = new Node(north, south, east, west);
	currentNode = startNode;
	nodes.push_back(startNode);
}
Graph::~Graph() {
	
}

/*
	Add a node, based on the current traveling direction. N = 0, E = 1, S = 2, W = 3.
*/
void Graph::addNode(bool north, bool south, bool east, bool west, int direction, int length) {
	prevNode = currentNode;

	// Create a new node, defining whether it has edges in each direction
	currentNode = new Node(north, south, east, west);
	nodes.push_back(currentNode);

	// Create a new edge with the measured length
	Edge* newEdge = new Edge(prevNode, currentNode, length);
	edges.push_back(newEdge);

	// Set the edges of the current and previous nodes, and set the x, y coordinate
	if (direction == 0) {		 // Currently traveling north
		prevNode->setNorthEdge(newEdge);
		currentNode->setSouthEdge(newEdge);
		currentNode->setX(prevNode->getX());
		currentNode->setY(prevNode->getY() + length);

	} else if (direction == 1) { // East
		prevNode->setEastEdge(newEdge);
		currentNode->setWestEdge(newEdge);
		currentNode->setX(prevNode->getX() + length);
		currentNode->setY(prevNode->getY());

	} else if (direction == 2) { // South
		prevNode->setSouthEdge(newEdge);
		currentNode->setNorthEdge(newEdge);
		currentNode->setX(prevNode->getX());
		currentNode->setY(prevNode->getY() - length);

	} else if (direction == 3) { // West
		prevNode->setWestEdge(newEdge);
		currentNode->setEastEdge(newEdge);
		currentNode->setX(prevNode->getX() - length);
		currentNode->setY(prevNode->getY());
	}
}
