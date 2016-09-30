// declare a global variable with name initial_time and initialize it with zero before using this function
// include math.h
int[] exploration(int index){ 
	# define pi 3.14; // declare this variable as a global variable	
 	float A[3]; // Array to store the x and y and delta_time of the points 
 	int final_time; 
 	int delta_time, theta; 
 	float x, y; 
 	theta=fx_imu();  // function to calculate theta 
 	final_time =millis(); 
 	delta_time=final_time-initial_time; // finding delta time 
 	A[2]=delta_time;
 	initial_time=final_time; 
 	x=x_coordinate.peekindex(i)+delta_time1*sin(theta*pi/180); // calculating the x and y coordinate of the point and pushing them into stack
 	y=y_coordinate.peekindex(i)+delta_time1*cos(theta*pi/180);
 	A[0]=x;
 	A[1]=y; 
 	return A;
 }	 		      

