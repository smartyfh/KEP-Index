#ifndef TRUSSDETECTION_H_
#define TRUSSDETECTION_H_

#include <map>
#include <vector>
#include <string>
using namespace std;

class edge {
public:
    int u, v;
public:
    bool operator < ( const edge &eg ) const;
};

class edge_s {
public:
    int u, v, w, truss, sup, ssup;
public:
    edge_s();
};

class k_truss_detection {
private:
    map< edge, int > location;
    vector< vector< pair< int, int> >* > ad_list;
    vector< vector< edge >* > answer;
	vector< vector <int>* > com_nb;
    edge_s *arr;
	string res;
    int *bin;
	int *pos;
	int *rep;
    int node_num, edge_num, kmax, k; //the final value of k is the largest K-truss
private:
    void add_row_vector( int st, int ed);
    void init_ans();
public:
    k_truss_detection();
    void load_data( string path, string model, int w );
    int find_com_neighbor(int w, int to, int L, int R);
    void pre_sup_cal( string model );
    void binsort( int start );
	void com_nb_cpt();
	void dec(int x);
    void truss_decomposition( string path, string model, int ww ); //model = multigraph, mean, mode, simple "simple means we don't consider edge weight"
    void save_results( string path );
    void out_results();
};

#endif // TRUSSDETECTION_H_
