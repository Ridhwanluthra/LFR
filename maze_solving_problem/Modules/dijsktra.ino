#include <StackArray.h>
#define nVertices 6
#define INF 10000
#define destination 5
#define source 0              //Mapping source at 0

StackArray <int> shortest_path;
int path[nVertices];

void dijkstra (int [nVertices][nVertices], int, int[]);
void shortestPath (int, int);

void setup() {
  Serial.begin(9600);
  int graph [nVertices][nVertices] {{0, 1, 0, 0, 0, 0},
    {1, 0, 1, 0, 1, 0},
    {0, 1, 0, 0, 0, 1},
    {0, 0, 0, 0, 1, 0},
    {0, 1, 0, 1, 0, 0},
    {0, 0, 1, 0, 0, 0}};    //declaring an adjacency matrix
  dijkstra (graph, source, path);
  shortestPath (source, destination);
  while (!shortest_path.isEmpty()){
    Serial.println (shortest_path.pop());   
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}

//Finds the vertex with minimum distace value
int minDistance (int dist [], bool sptSet []) {
  // Initializing minimum values
  int min = INF, mIndex;
  for (int i = 0; i < nVertices; i++){
    if (sptSet[i] == false && dist[i] <= min)
      min = dist[i], mIndex = i;
  }
  return mIndex;
}

void dijkstra (int graph[nVertices][nVertices], int src, int path []) {
  int start_time = millis ();
  int dist[nVertices];        //Stores shortest distance of vertex i from source
  bool sptSet [nVertices];    //Tells if the shortest path is known yet
  //Initializing shortest distance to infinity and inclusion in shortest path to false
  for (int i = 0; i < nVertices; i++)
    dist [i] = INF, sptSet [i] = false;
  dist [src] = 0;               //Distance of source is 0 from itself
  //Finding shortest path of all matrix
  for (int i = 0; i < nVertices - 1; i++){
    //Takes the minimum distance vertex from the set of vertices that is yet to be processed
    int u = minDistance (dist, sptSet);
    //Marks the just explored vertex as processed
    sptSet[u] = true;
    //Updating the value of adjacent vertices of the picked vertex.
    for (int v = 0; v < nVertices; v++){
      /*
       * Update dist[v] only if it is not in sptSet. There is an edge from u to v,
       * and total weight of path from source to v through u is
       * smaller than the current value of dist[v]
       */
       if (!sptSet[v] && graph[u][v] && dist[u] != INF && 
            dist[u] + graph [u][v] < dist[v])
              dist[v] = dist[u] +  graph [u][v], path[v] = u;
    }
    path [src] = -1;          //Trace back to source succesful, marking source with -1.
  }
}

void shortestPath (int src, int dest){
  //Updates stack with the shortest path from source to destination.
  //Pop operation on the stack will retain the next vertex that needs to be visited.
  int current_vertex = dest;
  //path[src] = -1;
  shortest_path.push(dest);                 //To make sure that 
  while (path[current_vertex] != -1){      //-1 indicates end of path tracing
    shortest_path.push(path[current_vertex]);     //Pushing the shortest path to the stack
    //Popping from the stack will return the vertex that needs to be visited from the source in the correct order
    current_vertex = path[current_vertex];
  }
}

