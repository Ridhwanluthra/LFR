#include <QTRSensors.h>
#include <StackArray.h>

#define rightMotorF 7
#define rightMotorB 6
#define rightMotorPWM 5
#define leftMotorF 10
#define leftMotorB 9
#define leftMotorPWM 11
#define stby 8
//Add another matrix with all turns

//Incorporating Dijkstra
#define nVertices 6
#define INF 10000


QTRSensorsRC qtr((unsigned char[]) {A0,4, A1, A2, A3, A4, 2, A5}, 8, 2500);
//Declaring functions required for finding shortest path
//if required, convert parameters to pointers.
void dijkstra (int [nVertices][nVertices], int, int[]);
void shortestPath (int, int);

//Declaring variables required for Dijkstra and shortest path.
//If possible efficiently, convert to local variables
StackArray <int> shortest_path;
int path[nVertices];

int lastError = 0;
float kp = 0.1;  // 0.08 // for small = 0.1
float kd = 1.7; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;

/*          DEAFAULT CONFIGS
 *        
 *        * when encounter junction 3 and 6 go left first
 * 
 * 
 */



//direction :- N - 1, NE - 2, E - 3 , SE - 4, S - 5, SW - 6, W - 7, NW - 8;
int orientation;

/*      JUNCTION TYPES                                                                                  
 *       
 *   0 - no junction found    
 *       
 * 1 - ____ ____      2 -                 3 -      |                                                 
 *         |                  |                ____|                                   
 *         |              ____| ____               |                                      
 *                                                 |                             
 *                                                 
 * 4 - 
 *         |
 *      ___|        5 -                 6 -                        
 *         \                   /                 /   
 *          \             ___/               ___/            
 *                           |                  \                                 
 *                           |                   \                                    
 *                           
 *                           
 *                           
 */
int junction_type;

StackArray<int> x_coords;
StackArray<int> y_coords;
StackArray<int> next_direction;
StackArray<bool> need_for_exploration;
StackArray<int> next_explore;
StackArray<int> type_of_junc;

int adj_matrix[nVertices][nVertices];           //changing from 20 to arbitrary constant

int junction_count = 0;
 
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  Serial.begin(9600);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

 
void loop()
{
  unsigned int sensors[8];
  
  
  int position = qtr.readLine(sensors);
    
  int error = int(position) - 3500;
  integral += error;
  derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  //Serial.println(power_difference);
  lastError = error;
  
  const int maximum = 80;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;
    
  if (power_difference < 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum + power_difference);
    digitalWrite(stby,HIGH);
  } 
  else { 
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum);
    digitalWrite(stby,HIGH);
  }
}

void maze_solve() {
  // variable declarations
  int xy[] = [0, 0];
  int buf = 7;
  int x, y;
  int next_vertex;
  int previous_junc;
  int cost;
  int vertex_index;       //stores current vertex
  
  junction_type = check_junction();
  if (junction_type != 0) {
    //correct for pointers!!!1
    xy = get_coords()

    //loop to check which vertices have the same junction type
    for (int junc_finding_cnt = 0; junc_finding_cnt < type_of_junc.count(); junc_finding_cnt++) {
      // only if the junction is same do we check if the coords match
      //JUNCTION TYPE 1=2=3, 4=5=6 AS APPROACH WILL BE DIFF
      if (type_of_junc.peekindex(junc_finding_cnt) == junction_type) {
        x = x_coords.peekindex(junc_finding_cnt);
        y = y_coords.peekindex(junc_finding_cnt);
        if (xy[0] < x + buf && xy[0] > x - buf && xy[1] < y + buf && xy[1] > y - buf) {
          update_adj_matrix(junc_finding_cnt, previous_junc, cost);       //Check how to calculate cost
          need_for_exploration.change_value_at(junc_finding_cnt, 0);
          next_vertex = next_explore.pop();
          previous_junc = junc_finding_cnt;
          //now go to next vertex!!!!
          //    find  shortest path             --> Possible done
          //    then actually move on that
          //          angle btwen each set of vertices
          //     rotate by that angle and move
          //      Store type of each juntion as well to ensure movement in that direction
          //    Handle what to do if it finds two vertices in the same coord area.
          //  
          //Send in next dirn and then break the loop
          dijkstra(adj_matrix, current_vertex, path);       //In final run, we don't really need to use it
          // over and over again, values stored in path after the first run should suffince
          shortestPath (current_vertex, next_vertex);
          //Now use the shortest_path stack to decide the path to follow till the next vertex
          int current_junction = current_vertex, next_junction;
          while (!shortest_path.isEmpty()){
            next_junction = shortest_path.pop();
            //Basic of move funtion to be handled by kartheek
            //He'll return a stack with degrees and millis() of every movement between two adjacent junction
            move (current_junction, next_junction);         //Make sure that the function handles the case where source and destination is same
            //Handle the case when there are two direct paths to go from junction 'A' to adjacent junction 'B'
            current_junction = next_junction;
          }
        }
      }
    }
    //NEW VERTEX FOUND COMPLETELY handled         -->  How're we calculating cost?
    //use flag to handle new vertices
    new_junction_handler(previous_junc);
  }
}

void new_junction_handler(int &previous_junc) {         //passing previous_junc with reference as we need to update the value.
  type_of_junc.push(junction_type);
  junction_setting();
  need_for_exploration.push(1);
  next_explore.push(junction_count);
  // find cost from time taken in travel
  update_adj_matrix(junction_count, previous_junc, cost);
  previous_junc = junction_count;
  //finally increase junction count as new junction detected
  junction_count++;
}

void update_adj_matrix(int junc1, int junc2, int cost) {
  // if new vertex
  adj_matrix[junc1][junc2] = cost;
  adj_matrix[junc2][junc1] = cost;

}

void get_coords() {
  float xy[2];
  //write code for getting coords
  return xy;
}

void junction_setting() {
  //SET NEXT DIRECTION AFTER SOME TIME
  if (junction_type == 1) {
     next_direction.push(get_direction(theta + 90));
  }
  else if (junction_type == 2) {
    next_direction.push(get_direction(theta - 90));
  }
  else if (junction_type == 3) {
    next_direction.push(get_direction(theta + 180));
  }
  else if (junction_type == 4) {
    next_direction.push(get_direction(theta - 135));
  }
  else if (junction_type == 5) {
    next_direction.push(get_direction(theta + 135));
  }
  else if (junction_type == 6) {
    next_direction.push(get_direction(theta + 90));
  }
}

int get_direction() {
  if (theta > 345 && theta > 15){
    return 1;
  }
  else if (theta > 30 && theta < 60) {
    return 2;
  }
  else if (theta > 75 && theta < 105) {
    return 3;
  }
  else if (theta > 120 && theta < 150) {
    return 4;
  }
  else if (theta > 165 && theta < 195) {
    return 5;
  }
  else if (theta > 210 && theta < 240) {
    return 6;
  }
  else if (theta > 255 && theta < 285) {
    return 7;
  }
  else if (theta > 300 && theta < 330) {
    return 8;
  }
}


int check_junction() {
  private int junction_type = 0;
  //conditions to detect which junction
  if (junction_type != 0 ) {
    //FIX DELAY - USE SOMETHING ELSE;
    delay(3000);
  }
  // return junction_type
}





void stop1() {
  digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,0 );
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,LOW);
}

void right()
  {    
   digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
   }
void forward()
  {
  digitalWrite(rightMotorF,HIGH);
  digitalWrite(rightMotorB,LOW);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF,HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
  }

  void move (int curret, int next){
    //Integrate code to move from current vertex to the next vertex
  }

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

//Finds the shortest path from source to a particular destination
//Uses the shortest distance and neighbour marked by Dijkstra
//Returns it as a stack
void shortestPath (int src, int dest){
  //Updates stack with the shortest path from source to destination.
  //Pop operation on the stack will retain the next vertex that needs to be visited.
  int current_vertex = dest;
  //path[src] = -1;

  //Ensuring that the stack used for tracing back the path is empty
  //Find better method to handle this exception
  if (!shortest_path.isEmpty())
    while (!shortest_path.isEmpty())
      shortest_path.pop();
  shortest_path.push(dest);                 //To tell the bot to move to the destination after reaching the closest neighbour
  while (path[current_vertex] != -1){      //-1 indicates end of path tracing
    shortest_path.push(path[current_vertex]);     //Pushing the shortest path to the stack
    //Popping from the stack will return the vertex that needs to be visited from the source in the correct order
    current_vertex = path[current_vertex];
  }
}
