#ifndef DTPROCESSOR_H_
#define DTPROCESSOR_H_

#include <map>
#include <set>
#include <vector>
#include <cstring>
#include <fstream>
#include <cctype>
using namespace std;

class Edge {
public:
	int source, target;
	double weight = 0 ;
public:
	bool operator < ( const Edge &eg ) const;
};

class DTProcessor {
public:
	set< Edge > edge_set, r_edge_set;
	set< int > node_set;
	vector< pair<int, int> > edge;

public:
	void data_processing ( string path );

};


#endif /* DTPROCESSOR_H_ */
