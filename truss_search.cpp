#include "truss_search.h"
#include <iostream>
using namespace std;

bool sortdes( const nbr &t1, const nbr &t2)
{
	return t1.w > t2.w; //des
}

//======ScoreEdge
ScoreEdge::ScoreEdge() {
	truss = 2;
	core_supt = 0;
}
bool ScoreEdge::operator < ( const ScoreEdge &e ) const {
	if ( (w == e.w) && (truss == e.truss) )
	{
		if ( u == e.u )
			return v < e.v;
		else
			return u < e.u;
	}
	else if ( w == e.w )
		return truss < e.truss;
    else
		return w < e.w;
}

//======edge_t
edge_t::edge_t(){
    truss = 2;
    sup = 0;
    //timestamp = 0;
}

/*bool edge_t::operator < ( const edge_t &e ) const {
/    if ( u == e.u )
/        return v < e.v;
/    else
/        return u < e.u;
	if ( (w == e.w) && (truss == e.truss) )
	{
		if ( u == e.u )
			return e.v < v;
		else
			return e.u < u;
	}
	else if ( e.w == w )
		return e.truss < truss;
	else
		return e.w < w;
}*/

//======tree_vertex
tree_vertex::tree_vertex() {
    vertex_list.clear();
}

//======kTree
kTree::kTree() {
    vertex_num = 0;
    tree_list.clear();
}

//======k_truss_search
void k_truss_search::truss_compute( string path, string model, int w) {
    truss_cal = new k_truss_detection();
    truss_cal -> truss_decomposition( path, model, w );
    delete truss_cal;
}

void k_truss_search::init() {
    truss_cal = NULL;
    LinkedList.clear();
    DupList.clear();
	common_nb.clear();
    in_queue.clear();
    supt.clear();
    map_fun.clear();
    arr = NULL;
    ICPTree = NULL;
    kmax = 2;
    sub = 0;
    edge_sz = 0;
    node_sz = 0;
    arr_sz = 0;
}

void k_truss_search::inc_row_vec ( int st, int ed ) {
    while ( st < ed )
    {
        st ++;
        LinkedList.push_back( new vector< edge_t > );
        DupList.push_back( new vector< edge_t > );
    }
}

void k_truss_search::load_graph( string path ) {
    cout << "Beginning loading graph..." << endl;
    clock_t stt, edd;
    stt = clock();
    edge_t e;
    int prenode = 0;
    path = path + "truss.txt";
    ifstream fin;
    fin.open ( path.c_str() );
    if ( fin )
    {
        LinkedList.push_back( new vector< edge_t > );
        DupList.push_back( new vector< edge_t > );
        while ( !fin.eof() )
        {
            fin >> e.u >> e.v >> e.w >> e.sup >> e.truss;
            if ( e.u == -1 ) break;
            if ( e.u > prenode )
            {
                inc_row_vec( prenode, e.u );
                prenode = e.u;
                node_sz = e.u;
            }
            LinkedList[e.u] -> push_back( e );
            kmax = max( kmax, e.truss );
            edge_sz ++;
            e.u = -1;
        }
        edge_sz >>= 1;
    } else {
        cout << "Error occurred when opening: " + path << endl;
        exit ( 1 );
    }
    fin.close();

    edd = clock();
    cout << "Time for loading graph: " << (double)(edd - stt)/CLOCKS_PER_SEC << endl;
    cout << "Finished loading graph!" << endl;
}

void k_truss_search::maxk_truss_cpt( int k ) {
    int len;
    for ( int i = 0; i <= node_sz; i ++ )
    {
        len = LinkedList[i] -> size();
        for ( int j = 0; j < len; j ++ )
        {
            if ( (*LinkedList[i])[j].truss >= k )
            {
                DupList[i] -> push_back( (*LinkedList[i])[j] );
            }
            else
                sub ++;
        }
        LinkedList[i] -> clear();
    }

    for ( int i = 0; i <= node_sz; i ++ )
    {
        len = DupList[i] -> size();
        for ( int j = 0; j < len; j ++ )
        {
            LinkedList[i] -> push_back( (*DupList[i])[j] );
        }
        DupList[i] -> clear();
    }
}

int k_truss_search::binarysearch( int w, int to, int L, int R ) {
	 if ( L > R )
        return -1;
    int mid = ( L + R ) / 2;
    if ( (*LinkedList[to])[mid].v == w )
        return mid;
    else if ( (*LinkedList[to])[mid].v > w )
        return binarysearch( w, to, L, mid-1 );
    else if ( (*LinkedList[to])[mid].v < w )
        return binarysearch( w, to, mid+1, R );
}

void k_truss_search::sort_edges() {
    int id = 0;
    int len;
    int len1;
    int len2;
    int to;
    int u;
    int v;
    int s;
    int t;
    int spt;
    ScoreEdge se;
    edge e;
    for ( int i = 0; i <= node_sz; i ++ )
    {
        len = LinkedList[i] -> size();
        se.u = i;
        e.u = i;
        for ( int j = 0; j < len; j ++ )
        {
            e.v = (*LinkedList[i])[j].v;
            if ( e.u < e.v ) break;
            se.v = e.v;
            se.w = (*LinkedList[i])[j].w;
			se.truss = (*LinkedList[i])[j].truss;
            arr[id++] = se;
            in_queue[e] = 0;

            //recalculate support
            spt = 0;
            u = e.u;
            v = e.v;
            len1 = len;
            len2 = LinkedList[v] -> size();

            if ( len1 > len2 )
            {
                swap(u, v);
                swap(len1, len2);
            }

            if ( (len1 + len2) > (int)(len1 * (log10(1.0 * len2)/log10(2))) )
            {
                for ( int o = 0; o < len1; o ++ )
                {
                    to = binarysearch( (*LinkedList[u])[o].v, v, 0, len2 -1 );
                    if ( to != -1 )
                        spt ++;
                }
            }
            else
            {
                s = 0;
                t = 0;
                while ( (s < len1) && (t < len2) )
                {
                    if ( (*LinkedList[u])[s].v == (*LinkedList[v])[t].v )
                    {
                        spt ++;
                        s ++;
                        t ++;
                    }
                    else if ( (*LinkedList[u])[s].v > (*LinkedList[v])[t].v )
                        t ++;
                    else
                        s ++;
                }
            }
            supt[e] = spt;
        }
    }
    sort(arr, arr + id);
    //cout << "Sort edges by influence value successfully!" << endl;
}

int k_truss_search::find_start_pos( int st, int ed ) {
    edge e;
    while ( st < ed )
    {
        e.u = arr[st].u;
        e.v = arr[st].v;
        if ( in_queue[e] != 2 )
            return st;
        else
            st ++;
    }
    return -1;
}

//======basic index construction algorithm
//==for vertex computation
void k_truss_search::tree_vertex_cpt(int kst, int ked) {

    cout << "Beginning constructing index by basic algorithm..." << endl;
    clock_t stt, edd;
    stt = clock();

    ICPTree = new kTree[kmax + 2]; //totally kmax - 1 trees

    for ( int i = kst; i <= ked; i ++ ) //only changes here
    {
        maxk_truss_cpt( i );

        arr_sz = edge_sz - (sub >> 1);
        arr = new ScoreEdge[ arr_sz + 2 ];
        sort_edges();
        //cout << "tree: " << i << endl;

        int sub_num = 0;
        int ptr = 0;
        int timestamp = 0;
        int u;
        int v;
        int uu;
        int vv;
        int s;
        int t;
        int ulen;
        int vlen;
        queue < edge > que;

        while( sub_num < arr_sz )
        {
            while( !que.empty() ) que.pop();

            ptr = find_start_pos ( ptr, arr_sz ); //find the next smallest influence value
            if ( ptr == -1 ) break;

            edge qe, oe, re;
            qe.u = arr[ptr].u;
            qe.v = arr[ptr].v;
            que.push( qe );
            in_queue[qe] = 1;
            timestamp ++;
            //cout << "vertex: " << timestamp << endl;

            tree_vertex tv;
            tv.influence = arr[ptr].w;
            tv.timestamp = timestamp;
            tv.first_one = qe;
            tv.vertex_list.push_back ( qe );

            while ( !que.empty() )
            {
                oe = que.front();
                que.pop();
                sub_num ++;
                u = oe.u;
                v = oe.v;
                //cout << u << " " << v << endl;
                s = 0;
                t = 0;
                if ( (LinkedList[u] -> size()) > (LinkedList[v] -> size()) )
                    swap( u, v );
                ulen = LinkedList[u] -> size();
                vlen = LinkedList[v] -> size();

                if ( (ulen + vlen) > (int)(ulen * (log10(1.0 * vlen)/log10(2))) )
                {
                    while ( s < ulen )
                    {
                        uu = (*LinkedList[u])[s].v;
                        t = binarysearch( uu, v, 0, vlen-1 );
                        if ( t == -1 )
                        {
                            s ++;
                            continue;
                        }

                        qe.u = max( u, uu );
                        qe.v = min( u, uu );
                        re.u = max( v, uu );
                        re.v = min( v, uu );
                        if ( (in_queue[qe] != 2) && (in_queue[re] != 2) )
                        {
                            if ( in_queue[qe] == 0 )
                            {
								supt[qe] --;
								if ( supt[qe] < (i - 2) )
								{
									que.push(qe);
                                    in_queue[qe] = 1;
                                    tv.vertex_list.push_back( qe );
								}
							}

                            if ( in_queue[re] == 0 )
                            {
								supt[re] --;
								if ( supt[re] < (i - 2) )
								{
									que.push(re);
									in_queue[re] = 1;
									tv.vertex_list.push_back( re );
								}
							}
                        }
                        s ++;
                    }
                }
                else
                {
                    while ( (s < ulen) && (t < vlen) )
                    {
                        uu = (*LinkedList[u])[s].v;
                        vv = (*LinkedList[v])[t].v;

                        if ( uu == vv )
                        {
                            qe.u = max( u, uu );
                            qe.v = min( u, uu );
                            re.u = max( v, uu );
                            re.v = min( v, uu );

							if ( (in_queue[qe] != 2) && (in_queue[re] != 2) )
							{
								if ( in_queue[qe] == 0 )
								{
									supt[qe] --;
									if ( supt[qe] < (i - 2) )
									{
										que.push(qe);
										in_queue[qe] = 1;
										tv.vertex_list.push_back( qe );
									}
								}

								if ( in_queue[re] == 0 )
								{
									supt[re] --;
									if ( supt[re] < (i - 2) )
									{
										que.push(re);
										in_queue[re] = 1;
										tv.vertex_list.push_back( re );
									}
								}
							}

                            s ++;
                            t ++;
                        }
                        else if ( uu > vv )
                            t ++;
                        else
                            s ++;
                    }
                }
                in_queue[oe] = 2;
            }
            ICPTree[i].vertex_num ++;
            ICPTree[i].tree_list.push_front( tv );
            tv.vertex_list.clear();
        }
        delete[] arr;
        in_queue.clear();
        supt.clear();
    }

    edd = clock();
    cout << "Time for basic algorithm: " << (double)( edd - stt )/CLOCKS_PER_SEC << endl;
    cout << "Finished running basic basic algorithm!" << endl;
}

//======Improved index construction algorithm
void k_truss_search::new_sort_edges() {
    int id = 0;
    int len;
    ScoreEdge se;
    for ( int i = 0; i <= node_sz; i ++ )
    {
        len = LinkedList[i] -> size();
        se.u = i;
        for ( int j = 0; j < len; j ++ )
        {
            se.v = (*LinkedList[i])[j].v;
            if ( se.u < se.v ) break; //only for u > v
            se.w = (*LinkedList[i])[j].w;
            se.truss = (*LinkedList[i])[j].truss;
			se.core_supt = 0;
            arr[id++] = se;
        }
    }
    sort(arr, arr + id);
	edge e;
	for ( int i = 0; i < id; i ++)
	{
		e.u = arr[i].u;
		e.v = arr[i].v;
		map_fun[e] = i;//the position of edge in arr
	}
}

void k_truss_search::kernal_support_cpt() {
    cout << "Beginning computing kernel support..." << endl;
    clock_t t1, t2;
    t1 = clock();

    int len1;
    int len2;
    int s;
    int t;
    int u;
    int v;
	int uu;
	int vv;
	int spt;
    edge e;
	nbr elm;
	common_nb.push_back( new vector< nbr > );
    for ( int i = 0; i < edge_sz; i ++ )
    {
        u = arr[i].u;
        v = arr[i].v;
		uu = u;
		vv = v;

        len1 = LinkedList[u] -> size();
        len2 = LinkedList[v] -> size();

        if ( len1 > len2 )
        {
            swap(u, v);
            swap(len1, len2);
        }

        //calculate kernel support and common neighbors
		common_nb.push_back( new vector< nbr > );
		spt = 0;
        if ( (len1 + len2) > (int)(len1 * (log10(1.0 * len2)/log10(2.0))) )
        {
            s = 0;
            while ( s < len1 )
            {
                t = binarysearch( (*LinkedList[u])[s].v, v, 0, len2 - 1 );
                if ( t == -1 )
                {
                    s ++;
                    continue;
                }
				elm.v = (*LinkedList[u])[s].v;
				elm.w = min( (*LinkedList[u])[s].w, (*LinkedList[v])[t].w );
				e.u = max(uu, elm.v);
				e.v = min(uu, elm.v);
				elm.pos1 = map_fun[e];
				e.u = max(vv, elm.v);
				e.v = min(vv, elm.v);
				elm.pos2 = map_fun[e];
				common_nb[i] -> push_back ( elm );
                if ( ((*LinkedList[u])[s].truss >= arr[i].truss) && ((*LinkedList[v])[t].truss >= arr[i].truss) )
                    spt ++;
                s ++;
            }
        }
        else
        {
            s = 0;
            t = 0;
            while ( (s < len1) && (t < len2) )
            {
                if ( ((*LinkedList[u])[s].v == (*LinkedList[v])[t].v) )
                {
					elm.v = (*LinkedList[u])[s].v;
					elm.w = min( (*LinkedList[u])[s].w, (*LinkedList[v])[t].w );
					e.u = max(uu, elm.v);
					e.v = min(uu, elm.v);
					elm.pos1 = map_fun[e];
					e.u = max(vv, elm.v);
					e.v = min(vv, elm.v);
					elm.pos2 = map_fun[e];
					common_nb[i] -> push_back ( elm );
					if ( ((*LinkedList[u])[s].truss >= arr[i].truss) && ((*LinkedList[v])[t].truss >= arr[i].truss) )
						spt ++;
                    s ++;
                    t ++;
                }
                else if ( (*LinkedList[u])[s].v > (*LinkedList[v])[t].v )
                    t ++;
                else
                    s ++;
            }
        }
		std::sort((*common_nb[i]).begin(), (*common_nb[i]).end(), sortdes);
		arr[i].core_supt = spt;
    }

    t2 = clock();
    cout << "Time: " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
    cout << "Finished computing support!" << endl;
}

/*void k_truss_search::update_graph( int w ) {
	int len;
    for ( int o = 1; o <= node_sz; o ++ )
    {
        len = LinkedList[o] -> size();
        for ( int j = 0; j < len; j ++ )
        {
			if ( (*LinkedList[o])[j].w >= w )
            {
                DupList[o] -> push_back( (*LinkedList[o])[j] );
            }
        }
        LinkedList[o] -> clear();
    }

    for ( int o = 1; o <= node_sz; o ++ )
    {
        len = DupList[o] -> size();
        for ( int j = 0; j < len; j ++ )
        {
            LinkedList[o] -> push_back( (*DupList[o])[j] );
        }
        DupList[o] -> clear();
    }
}*/

void k_truss_search::new_tree_vertex_cpt() {
    cout << "Beginning constructing KEP-Index by new algorithm..." << endl;
    clock_t stt, edd;
    stt = clock();

    arr = new ScoreEdge[edge_sz + 2];
    new_sort_edges(); //sort by weight and truss asc

//	for ( int i = 0; i < edge_sz; i ++ )
//		cout << arr[i].u << " " << arr[i].v << " " << arr[i].w << " " << arr[i].truss << endl;

    kernal_support_cpt(); //compute kernel support and common neighbors

    ICPTree = new kTree[kmax + 2]; //totally kmax - 1 trees

    int u;
    int v;
    nbr elm;
    int vertex_num;
    int len;
    int s;
    int pos1;
    int pos2;
    int pos3;
	int iflns;
    //int timestamp = 0;
    int *visit = new int[edge_sz + 2]; //visit =1 means in queue and = 2 been treated
    qnode e, oe, re;
	queue< qnode > que;
	vector< int > U;
	edge ie;

	memset(visit, 0, sizeof(int)*(edge_sz + 2) );
    for ( int i = 0; i < edge_sz; i ++ )
    {
		//if ( i % 1000000 == 0 ) cout << i << endl;
        //timestamp ++;
        e.u = arr[i].u;
        e.v = arr[i].v;
		e.pos1 = i;
		vertex_num = arr[i].truss;
		iflns = arr[i].w;
		ie.u = e.u;
		ie.v = e.v;
        tree_vertex *tv = new tree_vertex[ vertex_num + 2 ];

//cout << u << " " << v << " " << vertex_num << endl;
        for ( int x = 2; x <= vertex_num; x ++ )
        {
            tv[x].first_one = ie;
            tv[x].influence = iflns;
            //tv[x].timestamp = timestamp;
            //tv[x].vertex_list.push_back( e );
        }

        //===update truss or expand each tree vertex

        while ( !que.empty() ) que.pop();

        que.push( e );
		arr[i].truss = 0; //trusses = 0 means deleted. In other cases, it means truss
        visit[i] = 1;

        while( !que.empty() )
        {
            oe = que.front();
            que.pop();
			pos1 = oe.pos1;
            //if( arr[pos1].truss != 0 )
            //    tv[arr[pos1].truss + 1].vertex_list.push_back( oe );
            U.push_back( pos1 );

            u = oe.u;
            v = oe.v;
            len = common_nb[pos1] -> size();

            s = 0;
            while ( s < len )
            {
				elm = (*common_nb[pos1])[s];
				if (elm.w < iflns )
					break;
                e.u = max(u, elm.v);
				e.v = min(u, elm.v);
                re.u = max(v, elm.v);
                re.v = min(v, elm.v);
				pos2 = elm.pos1; //e
				pos3 = elm.pos2; //re
				e.pos1 = pos2;
				re.pos1 = pos3;

				if ( (arr[pos2].truss == 0 ) || (arr[pos3].truss == 0) )
				{
					s ++;
					continue;
				}

                if ( visit[pos2] == 0 )
                {
					if ( (arr[pos1].truss == 0) && (vertex_num >= arr[pos2].truss) )
                    {
						if ( (visit[pos3] == 0) && (arr[pos3].truss >= arr[pos2].truss) )
							arr[pos2].core_supt --;
						else if ( (visit[pos3] == 1) && (arr[pos3].truss + 1 >= arr[pos2].truss) )
							arr[pos2].core_supt --;
					}
					else if ( (arr[pos1].truss > 0) && ( arr[pos2].truss == (arr[pos1].truss + 1) ) )
                    {
						if ( (visit[pos3] == 0) && (arr[pos3].truss >= arr[pos2].truss) )
							arr[pos2].core_supt --;
						else if ( (visit[pos3] != 0) && (arr[pos3].truss + 1 > arr[pos2].truss) )
							arr[pos2].core_supt --;
                        else if ( (visit[pos3] == 1) && (arr[pos3].truss + 1 == arr[pos2].truss) )
							arr[pos2].core_supt --;
					}

					if ( (arr[pos2].core_supt + 2 < arr[pos2].truss) ) //(arr[pos2].truss <= vertex_num) &&
                    {
						arr[pos2].truss --;
                        que.push(e);
                        visit[pos2] = 1;
					}
				}

				if ( visit[pos3] == 0 )
                {
					if ( (arr[pos1].truss == 0) && (vertex_num >= arr[pos3].truss) )
                    {
						if ( (visit[pos2] == 0) && (arr[pos2].truss >= arr[pos3].truss) )
							arr[pos3].core_supt --;
                        else if ( (visit[pos2] == 1) && (arr[pos2].truss + 1 >= arr[pos3].truss) )
							arr[pos3].core_supt --;
                     }
					 else if ( (arr[pos1].truss > 0) && ( arr[pos3].truss == (arr[pos1].truss + 1) ) )
                     {
                         if ( (visit[pos2] == 0) && (arr[pos2].truss >= arr[pos3].truss) )
							arr[pos3].core_supt --;
                         else if ( (visit[pos2] != 0) && (arr[pos2].truss + 1 > arr[pos3].truss) )
							arr[pos3].core_supt --;
                         else if ( (visit[pos2] == 1) && (arr[pos2].truss + 1 == arr[pos3].truss) )
							arr[pos3].core_supt --;
                     }

					 if ( (arr[pos3].core_supt + 2 < arr[pos3].truss) ) //(arr[pos3].truss <= vertex_num) &&
                     {
						 arr[pos3].truss --;
                         que.push(re);
                         visit[pos3] = 1;
                     }
				}
                s ++;
			}
            visit[pos1] = 2;
        }

        //===update kernel support
        int len1 = U.size();
        for ( int o = 0; o < len1; o ++ )
        {
			pos1 = U[o];
            u = arr[pos1].u;
            v = arr[pos1].v;

			arr[pos1].core_supt = 0;
			if ( arr[pos1].truss == 0 ) continue;
			visit[pos1] = 0;

			len = common_nb[pos1] -> size();
			s = 0;
            while ( s < len )
			{
				elm = (*common_nb[pos1])[s];
				if (elm.w < iflns )
					break;
                //oe.u = max(u, elm.v);
                //oe.v = min(u, elm.v);
                //re.u = max(v, elm.v);
                //re.v = min(v, elm.v);
				pos2 = elm.pos1;
                pos3 = elm.pos2;

                if ( (arr[pos2].truss >= arr[pos1].truss) && (arr[pos3].truss >= arr[pos1].truss) )
					arr[pos1].core_supt ++;
                s ++;
			}
        }

        //===add vertex to tree
        for ( int o = 2; o <= vertex_num; o ++ )
        {
            ICPTree[o].vertex_num ++;
            ICPTree[o].tree_list.push_front( tv[o] );
        }

        U.clear();
        delete[] tv;
    }

    edd = clock();
    cout << "Time: " << (double)(edd - stt)/CLOCKS_PER_SEC << endl;
    cout << "Finished constructing index!" << endl;

    delete[] visit;
    delete[] arr;
}

//======the entrance of index construction
int k_truss_search::load_tree_vertex_edge(string path, int k, int r) {
	 ifstream fin;
    int source;
    int target;
    int influence;
	int id = 0;
	int ans = 0;

    stringstream ss;
    ss << k;
    path = path +"2/"+ ss.str() + "-tree.txt";
    fin.open ( path.c_str() );

    if ( fin )
    {
        while ( !fin.eof() )
        {
            fin >> source >> target >> influence;
			if (source == -1)
				break;
			ans = influence;
			id ++;
			if ( id == r )
				break;
			source = -1;

        }
    }
    fin.close();
	cout << "Load tree successfully!" << endl;
	return ans;
}

void k_truss_search::truss_search( string path ) {
    //string model = "simple";
	//int k = 16;
	//int r = 40;
	//int w = load_tree_vertex_edge(path, k, r);

   // truss_compute( path, model, w );

	string s1 , s2;
	s1 = "1/";
	s2 = "2/";

    init();
    load_graph( path );

    tree_vertex_cpt(2, kmax);
	save_tree( path + s1 );

    //new_tree_vertex_cpt();
    //save_key_edges( path + s2 );

    //test( kmax - 1 );
    Clear();
}

void k_truss_search::save_tree( string path ) {
	cout << "Beginning saving tree..." << endl;
    for ( int i = 2; i <= kmax; i ++ )
    {
        stringstream ss;
        ss << i;
		string str1;

        list< tree_vertex >::iterator it;
        for ( it = ICPTree[i].tree_list.begin(); it != ICPTree[i].tree_list.end(); it ++ )
        {
            list< edge >::iterator itt;
            int timestamp;
            //int influence;
            timestamp = (*it).timestamp;
            //influence = (*it).influence;

			stringstream ss3;
			ss3 << timestamp;
            for ( itt = (*it).vertex_list.begin(); itt != (*it).vertex_list.end(); itt ++)
			{
				stringstream ss1, ss2;
				ss1 << (*itt).u;
				ss2 << (*itt).v;

				str1 += ss1.str();
				str1 += " ";

				str1 += ss2.str();
				str1 += " ";

				str1 += ss3.str();
				str1 += "\n";

			}
                //fout << (*itt).u << " " << (*itt).v << " " << timestamp << " " << influence << endl;
        }

		string path_dir;
        path_dir = path + ss.str() + "-tree.txt";
        ofstream fout( path_dir.c_str() );
		fout << str1 ;
        fout.close();
    }
	cout << "Finished saving tree!" << endl;
}

void k_truss_search::save_key_edges( string path ) {
	cout << "Beginning saving tree..." << endl;
    for ( int i = 2; i <= kmax; i ++ )
    {
        stringstream ss;
        ss << i;
		string str1;

        list< tree_vertex >::iterator it;
        for ( it = ICPTree[i].tree_list.begin(); it != ICPTree[i].tree_list.end(); it ++ )
        {
			stringstream ss1, ss2, ss3;
			ss1 << (*it).first_one.u;
			ss2 << (*it).first_one.v;
			ss3 << (*it).influence;

			str1 += ss1.str();
			str1 += " ";

			str1 += ss2.str();
			str1 += " ";

			str1 += ss3.str();
			str1 += "\n";

        }

		string path_dir;
        path_dir = path + ss.str() + "-tree.txt";
        ofstream fout( path_dir.c_str() );
		fout << str1 ;
        fout.close();
    }
	cout << "Finished saving tree!" << endl;
}

void k_truss_search::Clear() {

    LinkedList.clear();

    DupList.clear();

	common_nb.clear();

    supt.clear();

    map_fun.clear();

    in_queue.clear();

    delete[] ICPTree;
}

void k_truss_search::test( int k ) {
    cout << ICPTree[k].vertex_num << endl;
    list< tree_vertex >::iterator it;
    for ( it = ICPTree[k].tree_list.begin(); it != ICPTree[k].tree_list.end(); it ++)
    {
        cout << "vertex: " << (*it).timestamp << endl;
        list< edge >::iterator itt;
        for ( itt = (*it).vertex_list.begin(); itt != (*it).vertex_list.end(); itt ++)
            cout << (*itt).u << " " << (*itt).v << endl;
    }
}

//======top-r-k-truss
void top_r_k_truss::initialization() {
    edge_list.clear();
    ktree.tree_list.clear();
    Pos.clear();
	com_nb.clear();
    ktree.vertex_num = 0;
	Rank = NULL;
	arr = NULL;
}

void top_r_k_truss::inc_edge_vec( int st, int ed ) {
    while ( st < ed )
    {
        st ++;
        edge_list.push_back( new vector< int > );
    }
}

void top_r_k_truss::load_ori_graph( string path, int k ) { // load k-truss
    ifstream fin;
    int source;
    int target;
    int weight;
    int support;
    int truss;
    int prenode = 0;
	int num = 0;
    path = path + "truss.txt";

    fin.open ( path.c_str() );
    if ( fin )
    {
        edge_list.push_back( new vector< int > );
        while ( !fin.eof() )
        {
            fin >> source >> target >> weight >> support >> truss;
            if ( source == -1 ) break;
            if ( source > prenode )
            {
                inc_edge_vec( prenode, source );
                prenode = source;
            }
			if (truss >= k)
			{
				edge_list[source] -> push_back( target );
				num ++;
			}
            source = -1;
        }
    } else {
        cout << "Error occurred when opening: " + path << endl;
        exit ( 1 );
    }
    fin.close();
    cout << "Load original graph successfully!" << endl;
	num = num / 2;
	Rank = new int[ num + 2 ];
	arr = new edge[ num + 2 ];
}

void top_r_k_truss::load_tree_index( string path, int k ) {
    ifstream fin;
    int source;
    int target;
    int timestamp;
    int last = -1;
	int id = 0;
    tree_vertex tv;
    edge e;

    stringstream ss;
    ss << k;
    path = path + ss.str() + "-tree.txt";
    fin.open ( path.c_str() );

    if ( fin )
    {
        while ( !fin.eof() )
        {
            fin >> source >> target >> timestamp; // >> influence;

            e.u = source;
            e.v = target;
			if ( timestamp == -1 )
                break;

            if ( timestamp != last )
            {
				tv.first_one = e;
                ktree.tree_list.push_back( tv );
                last = timestamp;
                //tv.vertex_list.clear();
                //tv.influence = influence;
                //tv.timestamp = timestamp;
            }

            //tv.vertex_list.push_back( e );
			Pos[e] = id; //pos
			Rank[id] = timestamp; //edge time stamp
			arr[id].u = source;
			arr[id].v = target;
			id ++;
            timestamp = -1;
        }
		edge_num = id;
    }
    fin.close();
	cout << "Load tree successfully!" << endl;
}

double top_r_k_truss::compute_com_nbr() {
	clock_t st, ed;
	st = clock();
	int u;
	int v;
	int len1;
	int len2;
	int s;
	int t;
	edge e;
	nbr elm;
	com_nb.push_back ( new vector<nbr> );
	for ( int i = 0; i < edge_num; i ++ )
	{
		com_nb.push_back ( new vector<nbr> );
		u = arr[i].u;
		v = arr[i].v;

		len1 = edge_list[u] -> size();
		len2 = edge_list[v] -> size();

		s = 0;
		t = 0;
		while ( (s < len1) && (t < len2) )
		{
			if ( (*edge_list[u])[s] == (*edge_list[v])[t] )
			{
				elm.v = (*edge_list[u])[s];
				e.u = max(u, elm.v);
				e.v = min(u, elm.v);
				elm.pos1 = Pos[e];

				e.u = max(v, elm.v);
				e.v = min(v, elm.v);
				elm.pos2 = Pos[e];

				com_nb[i] -> push_back(elm);
				s ++;
				t ++;
			}
			else if ( (*edge_list[u])[s] > (*edge_list[v])[t] )
				t ++;
			else
				s ++;
		}
	}
	ed = clock();
	cout << "Finished computing common neighbors!" << endl;
	return (double)(ed - st);
}

double top_r_k_truss::local_search( edge e, int r ) {
    int u;
    int v;
    int s;
    int len;
	int pos;
	int pos1;
	int pos2;
	int pos3;
    qnode oe, re, se;
	nbr elm;

    queue<qnode> que;
    while ( !que.empty() ) que.pop();

	pos1 = Pos[e];
	oe.u = e.u;
	oe.v = e.v;
	oe.pos1 = pos1;
    que.push( oe );
	int *inqueue = new int[edge_num + 2];
	memset (inqueue, 0, sizeof(int)*(edge_num +2));
    inqueue[pos1] = 1;

    cout << "top-" << r << ": vertex(vertices)" << endl;
	clock_t stt, edd;
	stt = clock();
    while( !que.empty() )
    {
        oe = que.front();
        que.pop();
        u = oe.u;
        v = oe.v;
		pos = oe.pos1;

        cout << u << " " << v << endl;

        s = 0;
		len = com_nb[pos] -> size();
        while ( s < len )
        {
			elm = (*com_nb[pos])[s];
			re.u = max(u, elm.v);
			re.v = min(u, elm.v);
			se.u = max(v, elm.v);
			se.v = min(v, elm.v);
			pos2 = elm.pos1;
			pos3 = elm.pos2;

			re.pos1 = pos2;
			se.pos1 = pos3;

            if ( Rank[pos2] >= Rank[pos1] && Rank[pos3] >= Rank[pos1] && (inqueue[pos2] == 0) )
            {
				que.push( re );
                inqueue[pos2] = 1;
            }
            if ( Rank[pos3] >= Rank[pos1] && Rank[pos2] >= Rank[pos1] && (inqueue[pos3] == 0) )
            {
				que.push( se );
                inqueue[pos3] = 1;
            }
			s ++;
        }
    }
	edd = clock();
	delete[] inqueue;
	return (double)(edd - stt);
}

void top_r_k_truss::find_top_r_k_truss ( string path, int k, int r ) {
    initialization();
    load_ori_graph( path, k );
    load_tree_index( path, k );
	double tt = compute_com_nbr();

	cout << tt/CLOCKS_PER_SEC << endl;
	tt = 0;

    list< tree_vertex >::iterator it;
    int id = 1;
	//int stp = 5;
    for ( it = ktree.tree_list.begin(); it != ktree.tree_list.end(); it ++ )
    {
        tt += local_search( (*it).first_one, id );
        r --;
		//if ( id == stp)
		//{
			//printf("%.5lf \n", tt/CLOCKS_PER_SEC);
			//stp = stp*2;
		//}
        id ++;
        if ( r == 0 )
            break;
    }
	cout << "Time for local search based on timestamp: ";
	printf("%.5lf \n", tt/CLOCKS_PER_SEC);

    edge_list.clear();
    ktree.tree_list.clear();
    Pos.clear();
	com_nb.clear();
	delete[] arr;
	delete[] Rank;
}
