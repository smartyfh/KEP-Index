#include "DataProcessor.h"
#include <iostream>


//=======Edge
bool Edge::operator < ( const Edge &eg ) const {
	if ( source == eg.source )
		return target < eg.target;
	else
		return source < eg.source;
}

//=======DataProcessor
void DTProcessor::data_processing ( string path ) {
	Edge e, re;
	int u, v, cnt = 0;
	map< pair<int, int>, double > w_map;
	map< int, pair<int, int> > uw_map;
	string output_dir = "E://DataSet/";
	ifstream fin;
	ofstream fout;
	fin.open( path.c_str(), ifstream::in );
	if ( fin )
	{
		/*while ( !fin.eof() )
		{
			fin >> e.source >> e.target >> e.weight;
			re.source = e.target;
			re.target = e.source;
			re.weight = e.weight;
			edge_set.insert( e );
			edge_set.insert( re );
			node_set.insert( e.source );
			node_set.insert( e.target );
			if ( e.source < e.target )
				edge.push_back( make_pair(e.source, e.target) );
		}*/
		pair<int, int> uv;
		while ( !fin.eof() )
        {
            fin >> u >> v;
            if ( u == -1 ) break;
            if ( u > v ) swap( u, v);
            uv = make_pair(u, v);
            if ( w_map[uv] == 0 )
                uw_map[ ++cnt ] = uv;
            w_map[uv] ++;
            u = -1;
        }

        for ( int i = 1; i <= cnt; i ++)
        {
            uv = uw_map[i];
            e.source = uv.first;
            e.target = uv.second;
            e.weight = w_map[uv];
//          re.source = e.target;
//			re.target = e.source;
//			re.weight = e.weight;
			edge_set.insert( e );
//			r_edge_set.insert( re );
//			node_set.insert( e.source );
//			node_set.insert( e.target );
        }
	} else
		cout << "Error occurred when opening: " + path << endl;
	fin.close();
	w_map.clear();
	uw_map.clear();

//======output the graph to directory
/*	cnt = 1;
// node's index and its name
	fout.open( (output_dir + "1-node.txt").c_str() );
	for ( set<int>::iterator it = node_set.begin(); it != node_set.end(); it ++)
		fout << cnt++ << " " << *it << endl;
	fout.close();
	node_set.clear();
//e(u, v) with u < v
	fout.open( (output_dir + "2-edge.txt").c_str() );
	cnt = 1;
//	for ( vector< pair<int, int> >::iterator it = edge.begin(); it != edge.end(); it ++)
//		fout << cnt++ << " " << (*it).first << " " << (*it).second << endl;
    for ( set<Edge>::iterator it = edge_set.begin(); it != edge_set.end(); it ++)
        if ( it->source < it->target )
            fout << cnt++ << " " << it->source << " " << it->target << endl;
	fout.close();*/
//	edge.clear();
//the adjacency list
	fout.open( (output_dir + "3-ad_list.txt").c_str() );
//	for ( set<Edge>::iterator itt = r_edge_set.begin(); itt != r_edge_set.end(); itt ++)
//        fout << itt->source << " " << itt->target << " " << itt->weight << endl;
//    r_edge_set.clear();
	for ( set<Edge>::iterator it = edge_set.begin(); it != edge_set.end(); it ++)
		fout << it->source << " " << it->target << " " << it->weight << endl;
	fout.close();
	edge_set.clear();
}
