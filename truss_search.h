#ifndef TRUSSSEARCH_H_
#define TRUSSSEARCH_H_

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <list>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cstring>
#include <stdlib.h>
using namespace std;

#include "truss_detection.h"

struct nbr {
	int v; //common neighbor
	int w; //triangle uvw   // w means weight
	int pos1; // the position of u w
	int pos2; // the position of v w
};

struct qnode {
	int u;
	int v;
	int pos1; // the position of edge (u, v)
};

class ScoreEdge {
public:
    int u;
    int v;
    int w;
    int truss;
	int core_supt;
public:
	ScoreEdge();
    bool operator < ( const ScoreEdge &e ) const;
};

class edge_t {
public:
    int u;
    int v;
    int truss;
    int sup;
    //int timestamp;
    int w;
public:
    edge_t();
    //bool operator < ( const edge_t &e ) const;
};

class tree_vertex {
public:
    int influence;
    int timestamp;
    edge first_one;
    list< edge > vertex_list;
public:
    tree_vertex();
};

class kTree {
public:
    int vertex_num;
    list< tree_vertex > tree_list;
public:
    kTree();
};

class k_truss_search {
private:
    k_truss_detection* truss_cal;

    vector< vector< edge_t >* > LinkedList; //edge index starting from 1
    vector< vector< edge_t >* > DupList;
    //vector< vector< edge >* > k_class; //edges in different k trusses
    ScoreEdge* arr; //edge array
	vector< vector <nbr>* > common_nb; //common neighbors
    map<edge, int> in_queue; //1 means in queue, 2 means treated
    map<edge, int> supt;
    map<edge, int> map_fun; //edge to edge id
    kTree* ICPTree;
    int edge_sz; //total edge number
    int node_sz;
    int arr_sz;
    int sub;
    int kmax; //truss
public:
    void init();
    void truss_compute( string path, string mode, int w );
    void inc_row_vec( int st, int ed );
    void load_graph( string path );
	int load_tree_vertex_edge(string path, int k, int r); //local search with weight

    void maxk_truss_cpt( int k );
    int binarysearch( int w, int to, int L, int R );
    void sort_edges();
    int find_start_pos( int st, int ed );
    //void tree_vertex_cpt(); //for vertex computation
	void tree_vertex_cpt(int kst, int ked); //for BFS
	void BFS( edge_t e );

    void new_sort_edges();
    void kernal_support_cpt();
	void update_graph( int w );
    void new_tree_vertex_cpt();

    void construct_tree();
    void truss_search( string path );

    void save_tree( string path );
	void save_key_edges( string path );

    void Clear();
    void test( int k );
};

//===find the top-r k-truss
class top_r_k_truss {
private:
    vector< vector< int >* > edge_list; //the original graph
    map< edge,int > Pos;
	vector< vector <nbr>* > com_nb; //common neighbors
    kTree ktree;
	int *Rank;
	edge *arr;
	int edge_num;
public:
    void initialization();
    void inc_edge_vec( int st, int ed );
    void load_ori_graph( string path, int k );
    void load_tree_index( string path, int k );
	double compute_com_nbr();
    double local_search ( edge e, int r );
    void find_top_r_k_truss ( string path, int k, int r ); // local search with timestamp;
};
#endif // TRUSSSEARCH_H_
