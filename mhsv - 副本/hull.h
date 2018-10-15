#ifndef Hull_Head_H
#define Hull_Head_H

inline float
isLeft( Point P0, Point P1, Point P2 )
{
    return (P1.x - P0.x)*(P2.y - P0.y) - (P2.x - P0.x)*(P1.y - P0.y);
}
 


// simpleHull_2D(): Melkman's 2D simple polyline O(n) convex hull algorithm
//    Input:  P[] = array of 2D vertex points for a simple polyline
//            n   = the number of points in V[]
//    Output: H[] = output convex hull array of vertices (max is n)
//    Return: h   = the number of points in H[]
int
simpleHull_2D( Point* P, int n, Point* H )
{
    // initialize a deque D[] from bottom to top so that the
    // 1st three vertices of P[] are a ccw triangle
    Point* D = new Point[2*n+1];
    int bot = n-2, top = bot+3;    // initial bottom and top deque indices
    D[bot] = D[top] = P[2];        // 3rd vertex is at both bot and top
    if (isLeft(P[0], P[1], P[2]) > 0) {
        D[bot+1] = P[0];
        D[bot+2] = P[1];           // ccw vertices are: 2,0,1,2
    }
    else {
        D[bot+1] = P[1];
        D[bot+2] = P[0];           // ccw vertices are: 2,1,0,2
    }

    // compute the hull on the deque D[]
    for (int i=3; i < n; i++) {   // process the rest of vertices
        // test if next vertex is inside the deque hull
        if ((isLeft(D[bot], D[bot+1], P[i]) > 0) &&
            (isLeft(D[top-1], D[top], P[i]) > 0) )
                 continue;         // skip an interior vertex

        // incrementally add an exterior vertex to the deque hull
        // get the rightmost tangent at the deque bot
        while (isLeft(D[bot], D[bot+1], P[i]) <= 0)
            ++bot;                 // remove bot of deque
        D[--bot] = P[i];           // insert P[i] at bot of deque

        // get the leftmost tangent at the deque top
        while (isLeft(D[top-1], D[top], P[i]) <= 0)
            --top;                 // pop top of deque
        D[++top] = P[i];           // push P[i] onto top of deque
    }

    // transcribe deque D[] to the output hull array H[]
    int h;        // hull vertex counter
    for (h=0; h <= (top-bot); h++)
        H[h] = D[bot + h];

    delete D;
    return h-1;
}


#endif