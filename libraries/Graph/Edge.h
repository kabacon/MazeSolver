#ifndef GRAPH_EDGE
#define GRAPH_EDGE
#include "Node.h"

class Edge {
private:
	Node* n1;
	Node* n2;

	int length;

public:
	Edge(Node* n1, Node* n2) {
		this->n1 = n1;
		this->n2 = n2;
	}

	Node* getNode1() { return n1;	  }
	Node* getNode2() { return n2;	  }	
	int  getLength() { return length; }
};

#endif