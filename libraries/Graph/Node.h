#ifndef GRAPH_NODE
#define GRAPH_NODE

class Edge;

class Node {
private:
	bool north, south, east, west;
	Edge* northEdge;
	Edge* southEdge;
	Edge* eastEdge;
	Edge* westEdge;
	int x, y;

public:
	Node() : north(false), south(false), east(false), west(false), x(0), y(0) {
		northEdge = 0;
		southEdge = 0;
		eastEdge = 0;
		westEdge = 0;
	}
	Node(bool north, bool south, bool east, bool west) : north(north),
		south(south), east(east), west(west), x(0), y(0) {
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

	bool hasNorth() { return north; }
	bool hasSouth() { return south; }
	bool hasEast()	{ return east;  }
	bool hasWest()	{ return west;  }

	void setNorthEdge(Edge* e) { northEdge = e; }
	void setSouthEdge(Edge* e) { southEdge = e; }
	void setEastEdge(Edge* e)  { eastEdge = e;  }
	void setWestEdge(Edge* e)  { westEdge = e;  }

	Edge* getNorth() { return northEdge; }
	Edge* getSouth() { return southEdge; }
	Edge* getEast()  { return eastEdge;	 }
	Edge* getWest()  { return westEdge;	 }

	void setX(int x);
	void setY(int y);
	int getX() { return x; }
	int getY() { return y; }
};

#endif

#ifndef GRAPH_EDGE
#define GRAPH_EDGE

class Edge {
private:
	Node* n1;
	Node* n2;

	int length;

public:
	Edge(Node* n1, Node* n2, int length) {
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
	int  getLength() { return length; }
};

#endif