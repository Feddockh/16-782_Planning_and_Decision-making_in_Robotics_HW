/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <vector>
#include <queue>
//#include <fstream>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

// Create a struct to hold the properties of each cell
struct cell {
    int x;
    int y;
    int h;
    bool EXPLORED = false;
};

// Create a struct to hold the properties of each node
struct node {
    int g;
    int t;
    int f;
    cell* c;
    node* parent;
    bool CLOSED = false;
    bool GOAL = false;
};

// Comparator function used by the priority queue to organize nodes by f values
struct compare_nodes {
    bool operator()(const node* node1, const node* node2) const {
        return node1->f > node2->f;
    }
};

/*
// Open the text file and save the coordinate data
void save_to_file(const std::string& text) {
    
    // Open the file in append mode
    std::ofstream file("coordinates.txt", std::ios::app);
    
    // Check that the file has successfully openned
    if (file.is_open()) {
        
        // Write the text to the file
        file << text << std::endl;
        
        // Close the file
        file.close();
    }
}

// Delete the text file
void delete_file() {
    std::remove("coordinates.txt");
}
*/

// Trace back to the starting node while sending data to a file
void traceback(node* s, int initial_x, int initial_y) {
    
    // Check if we've reached the starting node
    while (s->c->x != initial_x || s->c->y != initial_y) {
        
        // Send the coordinates of the current node to the text file
        //std::string coordinates = s->c->x + ", " + s->c->y;
        
        // Save the string to the file
        //save_to_file(coordinates);
        printf("traceback - x: %d, y: %d, t: %d\n", s->c->x, s->c->y, s->t);
        
        // Set the address of node s to the parent of itself to go backwards
        s = s->parent;
    }
    
    // Copy the coordinates of the starting position to the file
    //std::string coordinates = s->c->x + ", " + s->c->y;
    
   // Save the string to the file
   //save_to_file(coordinates);
}



static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // FOR TESTING PURPOSES - OUT-OF-MAP POSITION COMMANDED
    action_ptr[0] = 100000;
    action_ptr[1] = 100000;
    
    //////////// Reverse Multi-Goal Djistra's Algorihtm /////////////////
    
    // Create a 2d grid to hold the cells
    std::vector<std::vector<cell>> grid(x_size, std::vector<cell>(y_size));
    
    // Create a queue for Dijstra's search that holds the cell's addresses
    std::queue<cell*> ds_queue;
    
    // Initialize the cells corresponding to the trajectory coordinates and add them to the queue
    for (int i=0; i<target_steps; i++) {
        
        // Set the values of the cell corresponding with the trajectory coordinates
        int x = (int) target_traj[i];
        int y = (int) target_traj[target_steps + i];
        grid[x][y].x = x;
        grid[x][y].y = y;
        
        // Set the hueristic value of the cell
        grid[x][y].h = 0;
        
        // Push this cell's address onto the queue
        ds_queue.push(&grid[x][y]);
        
        // Print out the trajectories after being added to the queue
        printf("Grid - x: %d, y: %d, h: %d\n", grid[x][y].x, grid[x][y].y, grid[x][y].h);
    }
    
    // Search around using djistra's algorithm from the trajectory cells
    while (!ds_queue.empty()) {
        
        // Access the next cell's address from the queue
        cell* s = ds_queue.front();
        printf("cell s - x: %d, y: %d, h: %d\n", s->x, s->y, s->h);
        
        // Remove the next cell's address from the queue
        ds_queue.pop();
        
        // Check for surrounding nodes
        for (int i=0; i<NUMOFDIRS; i++) {
            
            // Determine the next x and y coordinates
            int new_x = s->x + dX[i];
            int new_y = s->y + dY[i];
            
            // Check if the new x and y values are within the boundaries of the map
            if (new_x >= 0 && new_x < x_size && new_y >= 0 && new_y < y_size) {
                
                // Find the cost of the cell from the map
                int new_cell_cost = (int)map[new_y * x_size + new_x];
                
                // Checks whether the new position is greater than or equal to 0 and below the collision threshold
                if (new_cell_cost >= 0 && new_cell_cost < collision_thresh) {
                    
                    // Check if the cell has been explored or can be updated (new h less than current h)
                    if (grid[new_x][new_y].h > s->h + new_cell_cost || grid[new_x][new_y].EXPLORED == false) {

                        // Set the x and y values of the cell
                        grid[new_x][new_y].x = new_x;
                        grid[new_x][new_y].y = new_y;

                        // Set the new h value
                        grid[new_x][new_y].h = s->h + new_cell_cost;
                        
                        // Add the address of the grid cell to the queue and mark it as explored
                        ds_queue.push(&grid[new_x][new_y]);
                        grid[new_x][new_y].EXPLORED = true;
                        
                        printf("cell s' - x: %d, y: %d, h: %d\n", grid[new_x][new_y].x, grid[new_x][new_y].y, grid[new_x][new_y].h);
                    }
                }
            }
        }
    }
    
    ///////////////////////// A* Algorihtm //////////////////////////////
    
    // Create a 3-dimensional space full of nodes for state space <x, y, t>
    std::vector<std::vector<std::vector<node>>> space(x_size, std::vector<std::vector<node>>(y_size, std::vector<node>(target_steps)));
    
    // Initialize the goal nodes
    for (int t=0; t<target_steps; t++) {
        
        // Determine the x and y coordinates of the trajectory
        int x = (int) target_traj[t];
        int y = (int) target_traj[target_steps + t];
        
        // Save the node's cell address
        space[x][y][t].c = &grid[x][y];
        
        // Set the time of the node
        space[x][y][t].t = t;
        
        // Set the node as a goal node
        space[x][y][t].GOAL = true;
        
        // Set the other node parameters to defaults
        space[x][y][t].g = 0;
        space[x][y][t].f = 0;
        space[x][y][t].parent = nullptr;
    }
    
    // Create a priority queue that will be used to explore
    std::priority_queue<node*, std::vector<node*>, compare_nodes> OPEN;
    
    // Initialize the starting node (at t = 0)
    space[robotposeX][robotposeY][0].c = &grid[robotposeX][robotposeY];
    space[robotposeX][robotposeY][0].t = 0;
    space[robotposeX][robotposeY][0].g = 0;
    space[robotposeX][robotposeY][0].f = space[robotposeX][robotposeY][0].c->h;
    space[robotposeX][robotposeY][0].parent = nullptr;
    
    // Add the starting node onto the OPEN queue
    OPEN.push(&space[robotposeX][robotposeY][0]);
    
    // Begin the A* algorithm and continue expanding nodes until OPEN is empty (or a goal node is reached)
    while (!OPEN.empty()) {
        
        // Remove the s node with the smallest f-value (next node in OPEN priority queue)
        node* s = OPEN.top();
        OPEN.pop();
        
        printf("node s - x: %d, y: %d, t: %d, h: %d\n", s->c->x, s->c->y, s->t, s->c->h);
        
        // Set node* s to closed
        s->CLOSED = true;
        
        // Check if node s is a goal node
        if (s->GOAL == true) {
            
            printf("Goal state reached at: x=%d, y=%d, t=%d\n", s->c->x, s->c->y, s->t);
            
            // Run the traceback function to determine the past coordinates and copy to a file
            traceback(s, robotposeX, robotposeY);
            
            return;
        }
        
        // For every successor s' of s that is not CLOSED
        for (int i=0; i<NUMOFDIRS; i++) {
            
            // Determine the next possible coordinates
            int new_x = s->c->x + dX[i];
            int new_y = s->c->y + dY[i];
            int new_t = s->t + 1;
            
            // Check if the possible coordinates are within the 3-dimensional space
            if (new_x >= 0 && new_x < x_size && new_y >= 0 && new_y < y_size && new_t >= 0 && new_t < target_steps) {
                
                // Find the cost of the node from the map
                int new_node_cost = (int)map[new_y * x_size + new_x];
                
                // Checks whether the new position is greater than or equal to 0 and below the collision threshold
                if (new_node_cost >= 0 && new_node_cost < collision_thresh) {
                    
                    // Check if the node has been CLOSED (already visited)
                    if (!space[new_x][new_y][new_t].CLOSED) {
                        
                        // Check if the g-value of the successor s' is greater than the current g-value + the edge cost
                        // Or if the g-value has not yet been set (g = 0)
                        if (space[new_x][new_y][new_t].g > s->g + new_node_cost || space[new_x][new_y][new_t].g == 0) {
                            
                            // Set the space with the new g-value
                            space[new_x][new_y][new_t].g = s->g + new_node_cost;
                            
                            // Connect the node to it's corresponding cell
                            space[new_x][new_y][new_t].c = &grid[new_x][new_y];
                            
                            // Add up the new f-value
                            space[new_x][new_y][new_t].f = space[new_x][new_y][new_t].g + space[new_x][new_y][new_t].c->h;
                            
                            // Set the parent node pointer to the current s node
                            space[new_x][new_y][new_t].parent = s;
                            
                            // Set the time it took to reach the node
                            space[new_x][new_y][new_t].t = new_t;
                            
                            // Insert the new node into the OPEN priority queue
                            OPEN.push(&space[new_x][new_y][new_t]);
                            
                            printf("node s' - x: %d, y: %d, t: %d, h: %d\n", space[new_x][new_y][new_t].c->x, space[new_x][new_y][new_t].c->y, space[new_x][new_y][new_t].t, space[new_x][new_y][new_t].c->h);
                            
                        }
                    }
                }
            }
        }
    }
    
    
    
    
    
    /*
    // Get the goal position as the current position of the target
    // Use the number of steps to determine the position from the target_traj array
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    
    // Print out the current position of the robot and the goal
    printf("robot: %d %d;\n", robotposeX, robotposeY);
    printf("goal: %d %d;\n", goalposeX, goalposeY);

    // Create variable to store the best X and Y movement of robot
    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    
    // Compute the Euclidean distance between the current robot position and the goal position
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    
    // Iterate over all possible movement directions
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        
        // Computes the new x and y position of the robot using the current position and the change
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        // Checks whether the new position is within the map boundaries
        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            
            // Checks whether the new position is greater than 0 and below the collision threshold
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                
                // Compute the distance from the new position to the target
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                
                // Update the old distance to target and the bestX and Y coordinates if distance is less
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    
    // Update the position of the robot and return updated values
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    */
    
    //action_ptr[0] = robotposeX;
    //action_ptr[1] = robotposeY;
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}