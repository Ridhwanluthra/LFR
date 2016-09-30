void exploration(){ 
	# define pi 3.14;
 	StackArray<float> x_coordinate; // stack to store x coordintes 
	StackArray<float> y_coordinate; // stack to store y coordintes 
 	x_coordinate.push(0); //setting the coordintes of the origin
 	y_coordinate.push(0);
 	StackArray<int> delta_time; // to store the time between two points
 	//delta_time.push(0); 
 	int a=1, i=0; 
 	int initial_time=0, final_time; 
 	while (a!=-1){ 
 		int delta_time1, theta; 
 		float x, y; 
 		a=has_junction();  // a function which returns the value between 1 to 6  if a point occurs
 		forward();// function to run the bot  
 		if(a>0) {   // FINDING THE COORDINATES  
 			theta=fx_imu(); 
 			final_time =millis(); 
 			delta_time1=final_time-initial_time; // finding delta time 
 			delta_time.push(delta_time1);  
 			initial_time=final_time; 
 			x_coordinate.push(x_coordinate.peek(i)+delta_time1*sin(theta*pi/180)); // calculating the x and y coordinate of the point and pushing them into stack
 			y_coordinate.push(y_coordinate.peek(i)+delta_time1*cos(theta*pi/180));
 			i++; 
 		}	 		      
 	} 
 }