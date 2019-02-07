//There are 12 code additions you need to figure out for this homework.  Read through the comments they should be very
//helpful for this exercise.  The sections where you will need to add small amounts of code are numbered.  
//Turn in a printout of this finished code along with a print out of a completed path using option 7 to points some what far away from each other and with
// some obstacles in the way.  .  



// astar.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

//Tori Fujinami
//astar search algorithm
//complete program

#include <stdio.h>	//for printf and scanf
#include <stdlib.h>
#include <math.h>	//for abs

/*
functions implemented in pQueue.cpp 
priority queue functions:	--> necessary for open list and closed list
size()
peek()
push()
pop()
*/

/*
priority queue elements aka nodes properties
position (x and y position for this case)  called xPox, yPos
total distance (f), distance traveled from start (g), distance to goal(h) called totalDist, distTravelFromStart, distToGoal
--> defined in pQueue.h 
*/
#include "pQueue.h"

/*
also need dictionary of some sort to look up nodes
dictionary element properties:
parent row and column
heap specification, open or closed set
heap element index number
key: row and column position stored as 1d array
*/


/*
map size 16x11:
x = wall, not passable, 0 = open, - = path

0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0 
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
x x x x 0 0 0 x x x x  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
0 0 0 0 0 0 0 0 0 0 0  
 

hand made maze for testing-> 
for test mazes: start at (0,0), end at (15,10)
 
- x x x x x x x x x x 
- - - - x - - - x 0 x  
x x x - - - x - - - x  
x x x x x x x x x - x  
0 0 0 0 - - - - - - 0  
x x x x - x x x x 0 0 
- - - - - x - - - x x  
- x x x x x - x - x 0  
- - - - - - - x - x 0  
x x x x x x x x - - - 
x x x x - - - x x x -  
0 0 0 x - x - x 0 x -  
0 x x x - x - - - x - 
- - - - - x x x - - -  
- x x x x 0 x x x x x  
- - - - - - - - - - - 

mini-mazes for covering the basics with solution
5x5
m1: start (1,1) end (3,3)
x 0 x x x
x - 0 0 x
x - x x x 
x - - - x
x x x 0 x 

m2: start (1,1) end (3,3)
x 0 x x x
x - - - x
x 0 x - x 
x 0 x - x
x x x 0 x 

m3: start (1,1) end (3,1)
x 0 x x x
x - - - x
x x x - x 
x - - - x
x 0 x x x 

m4: start (1,1) end (3,3)
x 0 x x x
x - x 0 x
x - x 0 x 
x - - - x
x 0 x x x 

*/

char m0[25] = {'0', '0', '0', '0', '0',
'0', '0', '0', '0', '0',
'0', '0', '0', '0', '0', 
'0', '0', '0', '0', '0',
'0', '0', '0', '0', '0'};

char m1[25] = {'x', '0', 'x', 'x', 'x',
'x', '0', '0', '0', 'x',
'x', '0', 'x', 'x', 'x', 
'x', '0', '0', '0', 'x',
'x', 'x', 'x', '0', 'x'};

char m2[25] = {'x', '0', 'x', 'x', 'x',
'x', '0', '0', '0', 'x',
'x', '0', 'x', '0', 'x', 
'x', '0', 'x', '0', 'x',
'x', 'x', 'x', '0', 'x'};

char m3[25] = {'x', '0', 'x', 'x', 'x',
'x', '0', '0', '0', 'x',
'x', 'x', 'x', '0', 'x',
'x', '0', '0', '0', 'x',
'x', '0', 'x', 'x', 'x'}; 

char m4[25] = {'x', '0', 'x', 'x', 'x',
'x', '0', 'x', '0', 'x',
'x', '0', 'x', '0', 'x',
'x', '0', '0', '0', 'x',
'x', '0', 'x', 'x', 'x' };

char m5[42] = {'0', '0', '0', '0', '0','0',
'0', '0', '0', '0', '0','0',
'0', '0', '0', 'x', '0','0', 
'0', '0', '0', '0', '0','0',
'0', 'x', '0', 'x', '0','0',
'0', '0', '0', 'x', '0','0',
'0', '0', '0', '0', '0','0'};

char mapCourseStart[176] = 		//16x11
{	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',  
	'0', '0', '0', '0', '0', '0', '0', 'x', 'x', 'x', 'x', 
	'0', '0', '0', '0', '0', '0', '0', 'x', '0', '0', '0', 
	'x', 'x', '0', '0', '0', '0', '0', '0', '0', 'x', 'x', 
	'0', 'x', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
	'x', 'x', 'x', 'x', '0', '0', '0', '0', '0', '0', '0', 
	'0', 'x', '0', '0', '0', 'x', '0', '0', '0', '0', '0', 
	'0', 'x', 'x', 'x', 'x', '0', 'x', '0', '0', 'x', 'x', 
	'0', '0', '0', '0', '0', 'x', '0', '0', '0', 'x', '0', 
	'0', '0', '0', '0', '0', '0', '0', '0', '0', 'x', '0', 
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
	'x', 'x', 'x', '0', '0', '0', '0', '0', 'x', 'x', 'x',  
	'0', '0', '0', '0', 'x', '0', 'x', '0', '0', '0', '0', 
	'0', '0', '0', 'x', '0', 'x', '0', 'x', '0', '0', '0', 
	'0', '0', '0', '0', 'x', '0', 'x', '0', '0', '0', '0', 
	'0', '0', '0', '0', '0', 'x', 'x', 'x', '0', '0', '0' 	};
	
//char mapCourseStart[176] = 		//16x11
//{	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',  
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',  
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 
//	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0' 	};	
	
char maze[176] = 			//16x11
{
	'0', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 
	'0', '0', '0', '0', 'x', '0', '0', '0', 'x', '0', 'x',  
	'x', 'x', 'x', '0', '0', '0', 'x', '0', '0', '0', 'x',  
	'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', '0', 'x',  
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', 'x', '0', '0', 
	'0', '0', '0', '0', '0', 'x', '0', '0', '0', 'x', 'x',  
	'0', 'x', 'x', 'x', 'x', 'x', '0', 'x', '0', 'x', '0',
	'0', '0', '0', '0', '0', '0', '0', 'x', '0', 'x', '0',  
	'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', '0', '0', '0', 
	'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', '0',  
	'0', '0', '0', 'x', '0', 'x', '0', 'x', '0', 'x', '0',  
	'0', 'x', 'x', 'x', '0', 'x', '0', '0', '0', 'x', '0',
	'0', '0', '0', '0', '0', 'x', 'x', 'x', '0', '0', '0',  
	'0', 'x', 'x', 'x', 'x', '0', 'x', 'x', 'x', 'x', 'x',  
	'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'	};


/*
necessary helper functions for astar algorithm:
canTravel
heuristic or totalCost
getNeighbors
*/

/*
the algorithm in pseudo code:
// A*
Function A*(start,goal)
Initialize the open list
Initialize the closed list
Put the starting node on the open list
came_from = empty

while the open list is not empty
    Find the node with the least f on the open list, call it "q"
    Pop q off the open list
    Generate q's 4 neighbors
    For each neighbor
    	If neighbor is the goal, stop the search
        neighbor.g = q.g + distance between neighbor and q
		// this distance is the “Manhattan” distance on a square grid
        neighbor.h = distance from goal to neighbor
        neighbor.f = neighbor.g + neighbor.h

        If a node with the same position as neighbor is in the OPEN list \
            which has a lower f than neighbor, skip adding this neighbor

        if a node with the same position as neighbor is in the CLOSED list \ 
            which has a lower f than neighbor, skip adding this neighbor

        otherwise, add the node to the open list
	and came_from[neighor] = q //set neighbor’s parent equal to q
    end
    push q on the closed list
end

//After Astar has run call reconstruct_path passing it the final goal point to combine the total path (but in reverse order from finish to start.  
function reconstruct_path(came_from,current) // first current is goal 
    while current in came_from: 
	total_path.append(current)
	current = came_from[current] // assign current to the parent	    
    return total_path
*/

//global variables necessary for the algorithm
node_t neighbors[4];		//used to get a list of neighboring nodes
int currDist = 0;	//distance traveled from starting point
int pathLen = 0;			//used to keep track of number of points in reconstructed path
int pathRow[400];			//reconstructed path in reverse order
int pathCol[400];
char map[176];
int mapRowSize = 16;
int mapColSize = 11;
dictElem_t nodeTrack[176];		//to track location of nodes in the heap, and to track parents of nodes technically only needs to be mapRowSize*mapColsize long
heap_t openSet, closedSet;

//preconditions: rowCurr, colCurr, rowGoal, and colGoal are feasible locations in the matrix
//returns the distance between two points as the sum of the row and column differences "Manhattan" distance on a square grid
int heuristic(int rowCurr, int colCurr, int rowGoal, int colGoal)
{
	return abs(rowGoal-rowCurr) + abs(colGoal-colCurr);
}

//assumes that canTravel is called with neighboring points of a valid starting point
//precondition: row, col must be valid indices for a matrix
//returns 0 for false and 1 for true
//(row, col) is considered an unacceptable traveling point if it is represented as an 'x' in map, or if it is out of bounds of the map
int canTravel(int row, int col)
{
	//check for out of bounds
	int mapIdx = row*mapColSize + col;		//map data stored as 1d array
	if(mapIdx >= (mapRowSize*mapColSize) || mapIdx < 0)		//index out of bounds
		return 0;		//0 for cannot travel
	else if(col >= mapColSize || col < 0)			//too many columns, will result in a location in the next row
		return 0;
	else if(map[mapIdx] == 'x')	//wall reached, cannot travel
		return 0;
	else
		return 1;
}

//parameters rowCurr and colCurr must be valid starting points
//returns the number of neighbors which are viable traveling points
//fills the node_t array called neighbors, with neighboring nodes to which the path can travel
//note: not using diagonal neighbors here, so there are only for possibilities for neighboring nodes
int getNeighbors(int rowCurr, int colCurr)
{
	node_t nodeToAdd;
	int numNeighbors = 0;
	if(canTravel(rowCurr-1, colCurr) == 1)	//can travel up
	{
		nodeToAdd.row = rowCurr-1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr, colCurr-1) == 1)	//can travel left
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr-1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr,colCurr+1) == 1)	//can travel right
	{
		nodeToAdd.row = rowCurr;
		nodeToAdd.col = colCurr+1;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	if(canTravel(rowCurr+1, colCurr) == 1)	//can travel down
	{
		nodeToAdd.row = rowCurr+1;
		nodeToAdd.col = colCurr;
		neighbors[numNeighbors] = nodeToAdd;
		numNeighbors++;
	}
	return numNeighbors;
}
 
//helper function called inside astar to return the path from the goal point back to the starting point
//the list of points is put into the array pathRow and pathCol in reverse order, pathLen is the number of inserted points
void reconstructPath(int rowEnd, int colEnd, dictElem_t nodeTrack[])
{
	pathLen = 0;		//global variable, reset so start inserting at beginning of path array
	int currRow = rowEnd;
	int currCol = colEnd;
	while(currRow != 400 && currCol != 400)
	{
		//while node is not null "400", put it into path starting at end point
		pathRow[pathLen] = currRow;  //global array for Row
		pathCol[pathLen] = currCol;  //global array for Column
		pathLen++;
	//	printf("currPath: (%d, %d), %d\n", pathRow[pathLen-1], pathCol[pathLen-1], pathLen);
		int nodeTrackIdx = currRow*mapColSize+currCol;
		currRow = nodeTrack[nodeTrackIdx].parentRow;
		currCol = nodeTrack[nodeTrackIdx].parentCol;
	//	printf("next location: (%d, %d), %d\n", currRow, currCol, pathLen);
	}
//	printf("done with reconstruction\n");
}

//path planning algorithm
//parameters rowStart, colStart, rowEnd, and colEnd must be valid locations
//they must be both within the indices of the matrix size and must not be points where barriers ('x') exist
void astar(int rowStart, int colStart, int rowEnd, int colEnd)
{
	//pseudo code instruction: initialize open and closed sets
	//initialize a dictionary to keep track of parents of nodes for easy reconstruction and for keeping track of 
	//  heap indexes for easy retrieval from heaps
	//set the values of the dictionary, open and closed sets to be zero 
	// so that no old values sitting in memory will produce weird results
	int resetNodeCnt;
	for(resetNodeCnt=0; resetNodeCnt<176; resetNodeCnt++)
	{
		nodeTrack[resetNodeCnt].heapId = ' ';
		nodeTrack[resetNodeCnt].heapIdx = 0;
		nodeTrack[resetNodeCnt].parentRow = 0;
		nodeTrack[resetNodeCnt].parentCol = 0;
	}

	startHeap(&openSet);
	startHeap(&closedSet);
	currDist = 0;
	node_t start;
	
	/* initialize a start node to be the starting position
		since this is the start node, it should have zero distance traveled from the start, the normal predicted distance to the goal,
		and the total distance is the sum of the distance traveled from the start and the distance to the goal.*/
	/* update the dictionary, use row and column location as a key in the dictionary, 
		use 400 for null parent values and don't forget to indicate which heap, open or closed set, the node is being placed*/
	/* put starting node on open set*/
	
	start.row = rowStart;
	start.col = colStart;
	start.distTravelFromStart = 0;
	start.distToGoal = heuristic(rowStart,colStart,rowEnd,colEnd);
	start.totalDist = start.distTravelFromStart+start.distToGoal;
	int startIdx = (start.row*mapColSize)+start.col;
	(nodeTrack[startIdx]).heapId = 'o';		//o for open set
	nodeTrack[startIdx].parentRow = 400;	//use 400 as NULL, if 400, know reached beginning in reconstruction
	nodeTrack[startIdx].parentCol = 400;	//no parent value = 400 since out of bounds
	if(rowStart == rowEnd && colStart == colEnd)		//if start point is the end point, don't do anything more!!!
		return;
	push(start, &openSet, nodeTrack, mapColSize); // put start node on the openSet

	char goalFound = 'f'; //false
	
	/*while open set not empty and goal not yet found*/
	while(openSet.numElems > 0 && goalFound != 't')	
	{

		/*find the node with the least total distance (f_score) on the open list, call it q (called minDistNode in code)*/
		node_t minDistNode = peek(openSet);	//find node with least total cost, make that the current node
	
		/*set the current distance to the current node's distance traveled from the start.*/
		//1.  currDist is set to q's distance traveled from the Start.  Explain why this could be different from the Manhattan distance to the Start position 
		//    This question is just asking for comments.
		currDist = minDistNode.distTravelFromStart;
		
		(nodeTrack[(minDistNode.row*mapColSize)+minDistNode.col]).heapId = 'r';		//r for removed from any set
		// Manhattan Distance - piecewise addition of vertical and horizontal distances (from q to destination)
		// Regular Distance - Pythagorean distance from q to destination
		
		//2.  pop q (which is currently the minimum) off which queue? 
		// Choose one of these two lines of code
		// IF the Openset
		pop(&openSet, nodeTrack, mapColSize);
		// IF closedSet 
		//pop(&closedSet, nodeTrack, mapColSize);
		
		/*generate q's 4 neighbors*/
		// 3.  Pass q's row and col to getNeighbors
		int numNeighbors = getNeighbors(minDistNode.row, minDistNode.col);	//get list of neighbors
	
		/*for each neighbor*/
		int cnt = 0;
		for(cnt = 0; cnt<numNeighbors; cnt++)	//for each found neighbor
		{
			// 4. Just add comments here.  Find where the structure node_t is defined and inside commments here copy its definition for
			// viewing reference.  
			/*
			typedef struct node
{
	int row;
	int col;
	int distTravelFromStart;
	int distToGoal;
	int totalDist;
	int pushOrder;		//order that element was pushed onto the heap
} node_t;
			*/
			// All the answer for 4. will be commented.
			node_t next = neighbors[cnt];
			
			/*if neighbor is the goal, stop the search*/
				/*update the dictionary so this last node's parents are set to the current node*/
			if((next.row == rowEnd) && (next.col == colEnd))		//if neighbor is the goal, found the end, so stop the search
			{
				// 5.  set current neighbor's parents.  Set parentRow to q's row.  Set parentCol to q's col since q is the parent of this neighbor
				(nodeTrack[next.row*mapColSize+next.col]).parentRow = minDistNode.row;	 //set goal node's parent position to current position
				(nodeTrack[next.row*mapColSize+next.col]).parentCol = minDistNode.col;
				goalFound = 't';
				break;
			}
			
			/*neighbor.distTravelFromStart (g) = q.distTravelFromStart + distance between neighbor and q which is always 1 when search just top left bottom right*/
			// 6.  Set this neighbor's distance traveled from the start.  Remember you have the variable "currDist" that is the distance of q to Start
			next.distTravelFromStart = currDist + 1;
			
			/*neighbor.distToGoal (h) = distance from goal to neighbor, heuristic function	(estimated distance to goal)*/
			// 7.  Pass the correct parameters to "heuristic" to calculate the distance this neighbor is from the goal.
			//  Remember that we have the variables rowEnd and colEnd which are the grid coordinates of the goal 
			next.distToGoal = heuristic(next.row, next.col, rowEnd, colEnd);
			
			/*neighbor.totalDist (f) = neighbor.distTravelFromStart + neighbor.distToGoal
				(total estimated distance as sum of distance traveled from start and distance to goal)*/
			// 8.  Find f, (totalDist) for this neighbor
			next.totalDist = next.distTravelFromStart + next.distToGoal;
			
			
			// 9.  Just comments for this question.
			// Explain in your own words (not copying the comments below) what the next 19 lines of C code are doing
			
			/*if a node with the same position as neighbor is in the OPEN list
				which has a lower total distance than neighbor, skip putting this neighbor onto the open set*/
			//check if node is on the open set already
			int nodeTrackIdx = (next.row*mapColSize)+next.col;
			char skipPush = 'f';
			if(nodeTrack[nodeTrackIdx].heapId == 'o')	//on the open set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromOpen = openSet.elems[heapIndex];
				if(fromOpen.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, a better one already on there
			}
			
			/*if a node with the same position as neighbor is in the CLOSED list
				which has a lower f than neighbor, skip putting this neighbor onto the open set*/
			else if(nodeTrack[nodeTrackIdx].heapId == 'c')		//on closed set
			{
				int heapIndex = nodeTrack[nodeTrackIdx].heapIdx;
				node_t fromClosed = closedSet.elems[heapIndex];
				if(fromClosed.totalDist <= next.totalDist)
					skipPush = 't';		//skip putting this node onto the openset, already part of possible solution
			}
			
			/*if not skipping putting this node on the set, then push node onto the open set
				and update the dictionary to indicate the node is on the open set and set the parents of this node to the current node*/
				//(can't be an ordinary else to the things above because then it won't push onto the open set if already on open or closed set)
			if(skipPush != 't')
			{
				nodeTrack[nodeTrackIdx].heapId = 'o';		//mark in nodetrack that this is going onto the open set
				(nodeTrack[nodeTrackIdx]).parentRow = minDistNode.row;	 //set neighbor's parent position to current position
				(nodeTrack[nodeTrackIdx]).parentCol = minDistNode.col;
				//10.  push this neighbor on which queue? 
				// Choose one of these two lines of code
				// IF openSet
				push(next, &openSet, nodeTrack, mapColSize);
				// IF closedSet
				//push(next, &closedSet, nodeTrack, mapColSize);
				
			}
		}/* end for loop*/

		/* These lines of code look through the open set and verify that the neighboring node is either
		 * lower or higher than the minimum f. If the total distance is higher, do not add this node as
		 * there is a better path. Compute the same analysis for the closed list. If it is minimum, add 
		 * it to the open list. 
		 */
		
		int nodeTrackIdx = minDistNode.row*mapColSize+minDistNode.col;
		nodeTrack[nodeTrackIdx].heapId = 'c';
		//11.  push q (current node) on which queue? 
		// Choose one of these two lines of code
		// IF openSet
		//push(minDistNode, &openSet, nodeTrack, mapColSize);
		// IF closedSet
		push(minDistNode, &closedSet, nodeTrack, mapColSize);
		
	}  /*end while loop*/

	/*if a path was found from the start to the goal, then reconstruct the path*/
	if(goalFound == 't')
		// 12.  Pass the correct varaibles to "reconstructPath" in order for it to fill in the global arrays pathRow, pathCol
		//     and integer pathLen.  Note that the path is in reverse order in pathRow and pathCol.
		reconstructPath(rowEnd, colEnd, nodeTrack);
}


int _tmain(int argc, _TCHAR* argv[])
{
	int choice, startRow, startCol, endRow, endCol;
	while(1)
	{
		printf("------------------------------\n");
		printf("Menu:\n");
		printf("1 - solve 2 points in small block\n");
		printf("2 - solve m1\n");
		printf("3 - solve m2\n");
		printf("4 - solve m3\n");
		printf("5 - solve m4\n");
		printf("6 - solve m5\n");		
		printf("7 - solve 2 points in sample course\n");
		printf("8 - solve big maze\n");
		printf("9 - exit\n");
		printf("------------------------------\n");
		scanf("%d", &choice);
		switch(choice)
		{
			int i, j;		//for for loops
			case 1:
				printf("Solve 2 points in small block\n");
				printf("enter start and end points:\n");
				printf("startRow (an integer [0:4]): ");
				scanf("%d", &startRow);
				printf("startCol (an integer [0:4]): ");
				scanf("%d", &startCol);
				printf("endRow (an integer [0:4]): ");
				scanf("%d", &endRow);
				printf("endCol (an integer [0:4]): ");
				scanf("%d", &endCol);
				mapRowSize = 5;
				mapColSize = 5;
				for(i = 0; i<25; i++)		//copy m1 into map for solving
					map[i] = m0[i];
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				printf("\n");
				if(startRow>=0 && startRow<mapRowSize && startCol>=0 && startCol<mapColSize && endRow>=0 && endRow<mapRowSize && endCol>=0 && endCol<mapColSize)	//if both points in bounds
				{
					astar(startRow,startCol,endRow,endCol);		//solve
					printf("pathLen %d\n", pathLen);
					for(i = 0; i< pathLen; i++)		//put solution on map
					{
						int mapIdx = pathRow[i]*5 + pathCol[i];
						map[mapIdx] = '-';
					}
					//print map with solution
					for(i=0; i<5; i++)
					{
						for(j = 0; j<5; j++)
							printf("%c ", map[i*5+j]);
						printf("\n");
					}
				}
				else
					printf("at least one of the entered values not in range\n");
				break;
			
			case 2:
				printf("solving m1: m1 before\n");
				mapRowSize = 5;
				mapColSize = 5;
				for(i = 0; i<25; i++)		//copy m1 into map for solving
					map[i] = m1[i];
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				printf("\n");
				astar(1,1,3,3);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*5 + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				break;
				
			case 3:
				printf("solving m2: m2 before\n");
				mapRowSize = 5;
				mapColSize = 5;
				for(i = 0; i<25; i++)		//copy m1 into map for solving
					map[i] = m2[i];
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				printf("\n");
				astar(1,1,3,3);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*5 + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				break;
				
			case 4:
				printf("solving m3: m3 before\n");
				mapRowSize = 5;
				mapColSize = 5;
				for(i = 0; i<25; i++)		//copy m1 into map for solving
					map[i] = m3[i];
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				printf("\n");
				astar(1,1,3,1);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*5 + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				break;
				
			case 5:
				printf("solving m4: m4 before\n");
				mapRowSize = 5;
				mapColSize = 5;
				for(i = 0; i<25; i++)		//copy m1 into map for solving
					map[i] = m4[i];
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				printf("\n");
				astar(1,1,3,3);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*5 + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<5; i++)
				{
					for(j = 0; j<5; j++)
						printf("%c ", map[i*5+j]);
					printf("\n");
				}
				break;
				
			case 6:
				printf("solving m5: m5 before\n");
				mapRowSize = 7;
				mapColSize = 6;
				for(i = 0; i<42; i++)		//copy m0 into map for solving
					map[i] = m5[i];
				for(i=0; i<7; i++)
				{
					for(j = 0; j<6; j++)
						printf("%c ", map[i*6+j]);
					printf("\n");
				}
				printf("\n");
				astar(2,1,5,4);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*6 + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<7; i++)
				{
					for(j = 0; j<6; j++)
						printf("%c ", map[i*6+j]);
					printf("\n");
				}
				break;
				
			case 7:
				printf("Solve 2 points in course\n");
				printf("enter start and end points:\n");
				printf("startRow (an integer [0:15]): ");
				scanf("%d", &startRow);
				printf("startCol (an integer [0:10]): ");
				scanf("%d", &startCol);
				printf("endRow (an integer [0:15]): ");
				scanf("%d", &endRow);
				printf("endCol (an integer [0:10]): ");
				scanf("%d", &endCol);
				mapRowSize = 16;
				mapColSize = 11;
				for(i = 0; i<mapRowSize*mapColSize; i++)		//copy m1 into map for solving
					map[i] = mapCourseStart[i];
				for(i=0; i<mapRowSize; i++)
				{
					for(j = 0; j<mapColSize; j++)
						printf("%c ", map[i*mapColSize+j]);
					printf("\n");
				}
				printf("\n");
				if(startRow>=0 && startRow<mapRowSize && startCol>=0 && startCol<mapColSize && endRow>=0 && endRow<mapRowSize && endCol>=0 && endCol<mapColSize)		//if in bounds{
				{
					if(map[startRow*mapColSize+startCol]!='x' && map[endRow*mapColSize+endCol]!='x')		//make sure valid start and end points
					{
						astar(startRow,startCol,endRow,endCol);		//solve
						printf("pathLen %d\n", pathLen);
						for(i = 0; i< pathLen; i++)		//put solution on map
						{
							int mapIdx = pathRow[i]*mapColSize + pathCol[i];
							map[mapIdx] = '-';
						}
						//print map with solution
						for(i=0; i<mapRowSize; i++)
						{
							for(j = 0; j<mapColSize; j++)
								printf("%c ", map[i*mapColSize+j]);
							printf("\n");
						}	
					}
					else
						printf("start or stop point not a navigable point\n");
				}
				else
					printf("at least one entered value not in bounds of map size\n");
				break; 
			case 8:
				printf("Solve big maze\n");
				mapRowSize = 16;
				mapColSize = 11;
				for(i = 0; i<mapRowSize*mapColSize; i++)		//copy m1 into map for solving
					map[i] = maze[i];
				for(i=0; i<mapRowSize; i++)
				{
					for(j = 0; j<mapColSize; j++)
						printf("%c ", map[i*mapColSize+j]);
					printf("\n");
				}
				printf("\n");
				astar(0,0,15,10);		//solve
				printf("pathLen %d\n", pathLen);
				for(i = 0; i< pathLen; i++)		//put solution on map
				{
					int mapIdx = pathRow[i]*mapColSize + pathCol[i];
					map[mapIdx] = '-';
				}
				//print map with solution
				for(i=0; i<mapRowSize; i++)
				{
					for(j = 0; j<mapColSize; j++)
						printf("%c ", map[i*mapColSize+j]);
					printf("\n");
				}
				break; 
			
				
			case 9:
				exit(0);
				
			default:
				printf("Invalid choice!\n");
				exit(0);
				//break;
		}
	}
	
	return 0;
}



