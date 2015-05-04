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
void Graph::addNode(bool forward, bool left, bool right, int direction, long length) {
	// Get the x and y location of this node
	long x = 0;
	long y = 0;
	
	if (direction == 0) {		 // Currently traveling north
		y = currentNode->getY() + length;
		x = currentNode->getX();
	} else if (direction == 1) { // East
		x = currentNode->getX() + length;
		y = currentNode->getY();
	} else if (direction == 2) { // South
		y = currentNode->getY() - length;
		x = currentNode->getX();
	} else if (direction == 3) { // West
		x = currentNode->getX() - length;
		y = currentNode->getY();
	}
	prevNode = currentNode;
	
	// Check to see if this node exists in the map already with a tolerance
	bool found = false;
	vector<Node*>::iterator it = nodes.begin();
	while (it != nodes.end() && !found) {
		if (abs((*it)->getX() - x) < 800 && abs((*it)->getY() - y) < 800) {
			found = true;
			currentNode = *it;
		}
		it++;
	}
	
	// If it wasn't found, create a new node
	if (!found) {
		bool north, south, east, west;
		if (direction == 0) {		// North
			north = forward;
			south = true;
			east = right;
			west = left;
		}
		else if (direction == 1) {	// East
			north = left;
			south = right;
			east = forward;
			west = true;
		}
		else if (direction == 2) {	// South
			north = true;
			south = forward;
			east = left;
			west = right;
		}
		else if (direction == 3) {	// West
			north = right;
			south = left;
			east = true;
			west = forward;
		}
		currentNode = new Node(north, south, east, west);
		nodes.push_back(currentNode);
		currentNode->setX(x);
		currentNode->setY(y);
	}
	
	// Create a new edge with the measured length
	Edge* newEdge = new Edge(prevNode, currentNode, length);
	edges.push_back(newEdge);
	
	// Connect the edges of the nodes
	if (direction == 0) {		 // Currently traveling north
		prevNode->setNorthEdge(newEdge);
		currentNode->setSouthEdge(newEdge);

	} else if (direction == 1) { // East
		prevNode->setEastEdge(newEdge);
		currentNode->setWestEdge(newEdge);

	} else if (direction == 2) { // South
		prevNode->setSouthEdge(newEdge);
		currentNode->setNorthEdge(newEdge);

	} else if (direction == 3) { // West
		prevNode->setWestEdge(newEdge);
		currentNode->setEastEdge(newEdge);
	}
}

/*
	Advance the path by one node, and return the next direction
*/
int Graph::advancePath() {
	// Assuming front to back
	if (path.empty()) return -1;

	// Get the direction of the next node
	path.pop_front();
	if (path.empty()) return -1;
	prevNode = currentNode;
	currentNode = path.front();

	if (prevNode->getNorth() && prevNode->getNorth() == currentNode->getSouth()) {
		return 0;
	} else if (prevNode->getEast() && prevNode->getEast() == currentNode->getWest()) {
		return 1;
	} else if (prevNode->getSouth() && prevNode->getSouth() == currentNode->getNorth()) {
		return 2;
	} else if (prevNode->getWest() && prevNode->getWest() == currentNode->getEast()) {
		return 3;
	} else {
		return -1;
	}
}

/*
	Calculate the shortest path between two nodes in the graph using 
	Dijkstra's algorithm. After the path is calculated, it is assumed 
	that the robot is at the starting node in order to follow the path.
*/
void Graph::calculateShortestPath(Node* start, Node* finish) {
	visited.clear();
	unvisited.clear();
	distance.clear();
	previous.clear();

	distance[start] = 0;
	unvisited.insert(start);

	// Iterate until all connected nodes have been visited
	while (!unvisited.empty()) {
		// Add the closest node to visited 
		Node* n = getClosest();
		visited.insert(n);

		// Remove it from unvisited
		set<Node*>::iterator it = unvisited.find(n);
		if (it != unvisited.end()) unvisited.erase(n);

		// Calculate the distance to each neighbor
		calculateDistances(n);
	}

	// Step backwards through the path from finish to start
	path.clear();

	// If the search never found the finish, return (should be impossible)
	if (!previous.count(finish)) return;

	// Add the finish node
	path.push_back(finish);
	Node* step = finish;

	// Add each node along the path
	while (previous.count(step)) {
		step = previous[step];
		path.push_back(step);
	}

	// Now reverse the path and return
	path.reverse();
	currentNode = path.front();
	return;
}

/*
	Returns the closest unvisited node to the last visited node
*/
Node* Graph::getClosest() {
	Node* closest = 0;
	for (Node* n : unvisited) {
		if (!closest) {
			closest = n;
		} else{
			if (getDistance(n) < getDistance(closest)) {
				closest = n;
			}
		}
	}

	return closest;
}

/*
	Calculate the distance to each neighbor and add it to the 
	unvisited list
*/
void Graph::calculateDistances(Node* n) {
	// Get all neighbor nodes
	vector<Node*> neighbors;
	if (n->hasNorth()) {
		if (n->getNorth()) {
			if (n->getNorth()->getNode1() == n) neighbors.push_back(n->getNorth()->getNode2());
			else neighbors.push_back(n->getNorth()->getNode1());
		}
	}
	if (n->hasSouth()) {
		if (n->getSouth()) {
			if (n->getSouth()->getNode1() == n) neighbors.push_back(n->getSouth()->getNode2());
			else neighbors.push_back(n->getSouth()->getNode1());
		}
	}
	if (n->hasEast()) {
		if (n->getEast()) {
			if (n->getEast()->getNode1() == n) neighbors.push_back(n->getEast()->getNode2());
			else neighbors.push_back(n->getEast()->getNode1());
		}
	}
	if (n->hasWest()) {
		if (n->getWest()) {
			if (n->getWest()->getNode1() == n) neighbors.push_back(n->getWest()->getNode2());
			else neighbors.push_back(n->getWest()->getNode1());
		}
	}

	// Calculate the shortest distance to each neighbor
	for (Node* neighbor : neighbors) {
		if (getDistance(neighbor) > getDistance(n) + n->getDistance(neighbor)) {
			distance[neighbor] = getDistance(n) + n->getDistance(neighbor);
			previous[neighbor] = n;
			unvisited.insert(neighbor);
		}
	}
}

float Graph::getDistance(Node* n) {
	if (!distance.count(n)) return 32767;
	return distance[n];
}
