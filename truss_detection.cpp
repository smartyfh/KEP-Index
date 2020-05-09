#include "truss_detection.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <cstring>
#include <stdlib.h>
#include <sstream>
using namespace std;

//======edge
bool edge::operator < ( const edge &eg ) const {
    if ( u == eg.u ) return v < eg.v;
    else return u < eg.u;
}

//======edge_s
edge_s::edge_s (){
    sup = 0;
    ssup = 0;
    truss = 2;
}

//======k_truss_detection
k_truss_detection::k_truss_detection() {
    location.clear();
    ad_list.clear();
    answer.clear();
	com_nb.clear();
    arr = NULL;
    bin = NULL;
	pos = NULL;
	rep = NULL;
    node_num = 0;
    edge_num = 0;
    kmax = 0;
    k = 2;
}

void k_truss_detection::add_row_vector( int st, int ed ) {
    while ( st < ed )
    {
        st ++;
        ad_list.push_back( new vector< pair < int, int > > );
    }
}

void k_truss_detection::init_ans() {
    for ( int i = 0; i <= (kmax + 4); i ++ )
        answer.push_back( new vector< edge > );
}

void k_truss_detection::load_data ( string path, string model, int w ) {
    ifstream fin;
    int source;
    int target;
    int weight;
    int prenode = 0;
    path = path + "3-ad_list.txt";
    fin.open ( path.c_str() );
    if ( fin )
    {
        ad_list.push_back( new vector< pair< int, int > > );
        while ( !fin.eof() )
        {
            fin >> source >> target >> weight;
            if ( source == -1 ) break;
            if ( source > prenode )
            {
                add_row_vector( prenode, source );
                prenode = source;
				node_num = source;
            }
			if ( weight >= w )
				ad_list[source] -> push_back( make_pair( target, weight ) );
            edge_num ++;
            source = -1;
        }
    } else {
        cout << "Error occurred when opening: " + path << endl;
        exit ( 1 );
    }
	edge_num = edge_num / 2;
    fin.close();
    return;
}

int k_truss_detection::find_com_neighbor( int w, int to, int L, int R ) {
	if ( L > R )
		return -1;
	else
	{
		int mid = ( L + R ) / 2;
		if ( (*ad_list[to])[mid].first == w )
			return mid;
		else if ( (*ad_list[to])[mid].first > w )
			return find_com_neighbor( w, to, L, mid-1 );
		else if ( (*ad_list[to])[mid].first < w )
			return find_com_neighbor( w, to, mid+1, R );
	}
}

void k_truss_detection::pre_sup_cal ( string model ) {
    //======support calculation
    arr = new edge_s[ edge_num + 2 ];
    //memset( arr, 0, sizeof( edge_s )*( edge_num + 2) );
    int u;
    int v;
    int len;
    int len1;
    int len2;
    int id = 0;
    int s;
    int t;
    int spt;
    for ( int i = 0; i <= node_num; i ++ )
    {
        len = ad_list[i] -> size();
        for ( int j = 0; j < len; j ++ )
        {
            v = (*ad_list[ i ])[ j ].first;
            u = i;
            if ( u <= v ) break; // only consider u > v

            len1 = len;
            len2 = ad_list[v] -> size();
            if ( len1 > len2 )
            {
                swap(u, v);
                swap(len1, len2);
            }

			spt = 0;
			s = 0;
            t = 0;
			
            if ( (len1 + len2) <= (int)(len1 * (log10(1.0 * len2)/log10(2.0))) )
            { 
                while ( (s < len1) && (t < len2) )
                {
                    if ( (*ad_list[u])[s].first == (*ad_list[v])[t].first )
                    {
                        spt ++;
                        s ++;
                        t ++;
                    }
                    else if ( (*ad_list[u])[s].first > (*ad_list[v])[t].first )
                        t ++;
                    else
                        s ++;
                }
            }
            else
            {
                while ( s < len1 )
                {
                    t = find_com_neighbor( (*ad_list[u])[s].first, v, 0, len2 - 1 );
                    if ( t != -1 )
						spt ++;
                    s ++;
                }
            }
            arr[id].u = i;
            arr[id].v = (*ad_list[i])[j].first;
            arr[id].w = (*ad_list[i])[j].second;
            arr[id].ssup = spt;
            arr[id++].sup = spt;
            if ( spt > kmax )
                kmax = spt;
        }
    }
    //================
    //=======bin sort asc
    bin = new int[ kmax + 2 ];
	pos = new int[edge_num + 2];
	rep = new int[edge_num + 2];
}

void k_truss_detection::com_nb_cpt() {
	int u;
	int v;
	int len1;
	int len2;
	int s;
	int t;
	com_nb.push_back( new vector< int > );
	for ( int i = 0;i < edge_num; i ++)
	{
		u = arr[i].u;
		v = arr[i].v;
		com_nb.push_back( new vector< int > );

		len1 = ad_list[u] -> size();
		len2 = ad_list[v] -> size();

		if ( len1 > len2 )
		{
			swap(u, v);
			swap(len1, len2);
		}

		s = 0;
		t = 0;
		if ( (len1 + len2) <= (int)(len1 * (log10(1.0 * len2)/log10(2.0))) )
		{
			while ( (s < len1) && (t < len2) )
            {
				if ( (*ad_list[u])[s].first == (*ad_list[v])[t].first )
				{
					com_nb[i] -> push_back( (*ad_list[u])[s].first );
					s ++;
                    t ++;
				}
				else if ( (*ad_list[u])[s].first > (*ad_list[v])[t].first )
					t ++;
				else
					s ++;
			}
		}
        else{
			while ( s < len1 )
            {
				t = find_com_neighbor( (*ad_list[u])[s].first, v, 0, len2 - 1 );
				if ( t != -1 )
					com_nb[i] -> push_back( (*ad_list[u])[s].first );
                s ++;
			}
		}
	}
}

void k_truss_detection::binsort( int start ) {
    edge_s* dup = new edge_s[ edge_num + 2 - start ];
    for ( int i = 0; i < edge_num - start; i ++)
        dup[i] = arr[ i + start ];
    for ( int i = 0; i <= kmax; i ++ )
        bin[i] = 0;
    int spt;
    for ( int i = 0; i < edge_num - start; i ++ )
    {
        spt = dup[i].sup;
        bin[ spt ] ++;
    }
    int st = start, num = 0;
    for ( int i = 0; i <= kmax; i ++ )
    {
        num = bin[i];
        bin[i] = st;
        st += num;
    }
    edge e;
    for ( int i = start; i < edge_num; i ++ )
    {
        e.u = dup[i - start].u;
        e.v = dup[i - start].v;
        spt = dup[i - start].sup;
        location[ e ] = bin[ spt ];
        arr[ bin[spt] ] = dup[ i - start ];

		pos[ bin[spt] ] = bin[spt];
		rep[ bin[spt] ] = bin[spt];

        bin[ spt ] ++;
    }

	for ( int i = kmax; i > 0; i -- )
		bin[i] = bin[i-1]; 
	bin[0] = start;

    delete[] dup;

	com_nb_cpt();
 }

void k_truss_detection::dec(int x) { //swap
	int oo;
	int ppos;
	int op;
	int y;
	oo = arr[x].sup;
	ppos = bin[oo];
	bin[oo] ++;
	arr[x].sup --;
	op = rep[x];
	y = pos[ppos];

	pos[ppos] = x;
	rep[x] = ppos;
	pos[op] = y;
	rep[y] = op;
}

 void k_truss_detection::truss_decomposition( string path, string model, int ww ) {

     cout << "Beginning loading graph..." << endl;
     clock_t stat, finish;
     stat = clock();
     load_data( path, model, ww );
     finish = clock();
     cout << "time: " << (double)(finish-stat)/CLOCKS_PER_SEC << endl;
     cout << "Finished loading graph!" << endl;

     cout<< "Beginning computing support..." << endl;
     stat = clock();
     pre_sup_cal( model );
     finish = clock();
     cout << "time: " << (double)(finish-stat)/CLOCKS_PER_SEC << endl;
     cout << "Finished computing support!" << endl;

     cout << "The max number of support: kmax = " << kmax << endl;

	 cout << "Beginning truss decomposition!" << endl;
     stat = clock();
     init_ans();

     int start = 0;
	 int spt;
     int u;
     int v;
	 int w;
     int s;
	 int len;
     int ppos1;
     int ppos2;
	 int now;
     edge_s ttemp;
     edge e, re, te;

	 binsort( start ); //sort

     while( start < edge_num )
     {
		 now = pos[start];
         spt = arr[now].sup;
         if ( spt <= (k - 2) )
         {
             u = arr[now].u;
             v = arr[now].v;
			 
             e.u = u;
             e.v = v;
			 answer[k] -> push_back( e );
             
			 len = com_nb[now] -> size();
			 s = 0;
			 while ( s < len )
			 {
				 w = (*com_nb[now])[s];
				 re.u = max(u, w);
				 re.v = min(u, w);
				 te.u = max(v, w);
				 te.v = min(v, w);
				 ppos1 = location[re];
				 ppos2 = location[te];
				 if ( (arr[ppos1].sup == -1) || (arr[ppos2].sup == -1) )
				 {
					 s ++;
					 continue;
				 }

				 if ( arr[ppos1].sup > (k - 2) )
					dec(ppos1);
				 if ( arr[ppos2].sup > (k - 2) )
					dec(ppos2);
				 s ++;
			 }

			 arr[now].truss = k;
			 arr[now].sup = -1;
			 start ++;

         } else
            k ++;
     }

     finish = clock();
     cout << "time: " << (double)(finish-stat)/CLOCKS_PER_SEC << endl;
	 cout << "Finished truss decomposition!" << endl;

	 cout << "Maximal truss number: " << k << endl;
	 cout << answer[k] -> size() << endl;

     cout << "Beginning saving data!" << endl;

     save_results( path );
	
     cout << "Finished saving data!" << endl;

     delete[] arr;
     delete[] bin;
	 delete[] pos;
	 delete[] rep;
     location.clear();
     ad_list.clear();
	 com_nb.clear();

     //out_results();
 }

void k_truss_detection::save_results( string path ) {
	 for ( int i = 0; i <= node_num; i ++ )
    {
        int len = (*ad_list[i]).size();
        edge e;
        int pos;
        int target;
        for ( int j = 0; j < len; j ++ )
        {
            target = (*ad_list[i])[j].first;
            if ( i < target )
            {
                e.u = target;
                e.v = i;
            }
            else
            {
                e.u = i;
                e.v = target;
            }
            pos = location[e];

			stringstream ss1, ss2, ss3, ss4, ss5;
			ss1 << i;
			res += ss1.str();
			res += " ";

			ss2 << target;
			res += ss2.str();
			res += " ";

			ss3 << arr[pos].w;
			res += ss3.str();
			res += " ";

			ss4 << arr[pos].ssup;
			res += ss4.str();
			res += " ";

			ss5 << arr[pos].truss;
			res += ss5.str();
			res += "\n";

            //fout << i << " " << target << " " << arr[pos].w << " " << arr[pos].ssup << " " << arr[pos].truss << endl;
        }
    }

    path = path + "truss.txt";
    ofstream fout( path.c_str() );
    fout << res ;
    fout.close();
}

 void k_truss_detection::out_results() {
     bool flag = true;
     string path = "E://k-truss/trussness.txt";
     ofstream fout( path.c_str() );
     for ( int o = 2; o <= k; o ++ )
     {
         flag = true;
         for ( int j = 0; j < (int)answer[o] -> size(); j ++ )
         {
             if ( flag )
             {
                 fout << -2 << " " << o << endl;
                 flag = false;
             }
             fout << (*answer[o])[j].u << " " << (*answer[o])[j].v << endl;
         }
     }
     fout.close();
     answer.clear();
 }
