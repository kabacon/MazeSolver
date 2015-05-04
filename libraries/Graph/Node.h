#ifndef GRAPH_NODE
#define GRAPH_NODE

class Edge;

class Node {
private:
	int directions;	// Store whether there are edges in all 4 directions in one byte
	Edge* northEdge;
	Edge* southEdge;
	Edge* eastEdge;
	Edge* westEdge;
	long x, y;

public:
	Node() : directions(0), x(0), y(0) {
		
	}
	Node(bool north, bool south, bool east, bool west) : x(0), y(0) {
		directions = 0;
		if (north) directions += 1;
		if (south) directions += 2;
		if (east)  directions += 4;
		if (west)  directions += 8;
		northEdge = 0;
		southEdge = 0;
		eastEdge = 0;
		westEdge = 0;
	}

	~Node() {
		delete northEdge;
		delete southEdge;
		delete eastEdge;
		delete westEdge;
	}

	bool hasNorth() { return directions % 2; 	 }
	bool hasSouth() { return directions / 2 % 2;  }
	bool hasEast()	{ return directions / 4 % 2;  }
	bool hasWest()	{ return directions / 8 % 2;  }

	void setNorthEdge(Edge* e) { northEdge = e; }
	void setSouthEdge(Edge* e) { southEdge = e; }
	void setEastEdge(Edge* e)  { eastEdge = e;  }
	void setWestEdge(Edge* e)  { westEdge = e;  }

	Edge* getNorth() { return northEdge; }
	Edge* getSouth() { return southEdge; }
	Edge* getEast()  { return eastEdge;	 }
	Edge* getWest()  { return westEdge;	 }

	void setX(long x)	{ this->x = x; }
	void setY(long y)	{ this->y = y; }
	long getX() { return x; }
	long getY() { return y; }

	// Get the distance to another node, if it's connected
	long getDistance(Node* n);
};

#endif

#ifndef GRAPH_EDGE
#define GRAPH_EDGE

class Edge {
private:
	Node* n1;
	Node* n2;

	long length;

public:
	Edge(Node* n1, Node* n2, long length) {
		this->n1 = n1;
		this->n2 = n2;
		this->length = length;
	}

	~Edge() { 
		delete n1;
		delete n2;
	}

	Node* getNode1() { return n1; }
	Node* getNode2() { return n2; }
	long  getLength() { return length; }
};

#endif