#include <iostream>
#include <fstream>
#include <cstdio>
#include <stdlib.h>
#include "truss_detection.h"
#include "truss_search.h"
using namespace std;

void truss_detection ( string path, string model, int w )
{
    k_truss_detection *app = new k_truss_detection();
    app -> truss_decomposition( path, model, w );
    delete app;
}

void truss_search ( string path )
{
    k_truss_search* app = new k_truss_search();
    app -> truss_search( path );
    delete app;
}

void topr_cpt ( string path, int k, int r )
{
    top_r_k_truss * app = new top_r_k_truss();
    app -> find_top_r_k_truss ( path, k, r );
    delete app;
}

int main()
{
    int k = 4;
    int r = 6;
	int w = 0;
    string model = "simple";
    string path = "datapath";

    //truss_detection( path, model, w );

    //truss_search( path ); //in k_truss_search function, we can choose different models

    topr_cpt( path, k, r );

    return 0;
}
