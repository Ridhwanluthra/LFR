// declare a global variable with name initial_time and initialize it with zero before using this function
// include math.h
#define pi 3.14 // declare this variable as a global variable	
void exploration(int index){ 
	int final_time; 
 	int delta_time, teeta; 
 	float x, y; 
 	theeta=fx_imu();  // function to calculate theta 
 	final_time =millis(); 
 	delta_time=final_time-initial_time; // finding delta time 
 	xy[2]=delta_time;
 	initial_time=final_time; 
 	x=x_coordinate.peekindex(i)+delta_time1*sin(theta*pi/180); // calculating the x and y coordinate of the point and pushing them into stack
 	y=y_coordinate.peekindex(i)+delta_time1*cos(theta*pi/180);
 	xy[0]=x;
 	xy[1]=y; 
 }	 		      


//Fixed syntax error in coordinates.cpp
//Added Dijkstra's algorithm module to maze_solve/modules
