/* 
 * File:   algorithm.h
 * Author: Anirudha Simha
 *
 * Created on November 10, 2016, 9:15 PM
 */

#ifndef ALGORITHM_H
#define	ALGORITHM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <math.h> 
#include <stdbool.h>


// Definitions of Values used in code 
#define MIN_ROTATION_INCREMENT 30 // the angle increment that the rover can rotate by
#define OBSACLE_AVOIDANCE_DISTANCE 25 // the minimum distance the rover should travel upon encountering an obstacle in order to get past said obstacle 



// Main algorithm function what will be called in the lead rover's loop
void algorithm(); 
// responsible for setting the turn at which rover will go when it encounters an obstacle
void turnHandler(); 
// calculation function to get a third leg (leg C) when doing law of cosines 
float getLegC();
// algorithm initializer function 
void algoInit();
// gets how far the rover has traveled since its last local origin point 
float getCurrentTravelDistance();
// set new local origin by messaging the motors that they should remeasure the distance traveled so far from the current point
void setLocalOrigin();


// States for ALgorithm State machine 
typedef enum {
	FINDING_PATH = 0,
	ROTATING,
	TRAVELING_PATH,
	OBSTACLE_ENCOUNTERED,
	FOUND_LINE,
	FOUND_ENDPOINT,
	OBSTACLE_ENCOUNTERED_ROTATE,
	OBSTACLE_ENCOUNTERED_MOVE
} ALGORITHM_STATES;


// Struct containing all of the algorithm's data 
typedef struct {
	// typically will be distToGo - <THE DISTANCE THE ROVER HAS TRAVELED SO FAR>
	float legA; 
	// typically will be the distance the rover traveled after moving around an obstacle  
	float legB;
	// Typically the angle between legA and legB
	float angC;
	// distance rover is supposed to go to reach endpoint assuming no obstacles in way
	float distToGo;
	// Angle rover should be at in order to be able to go straight and reach the endpoint
	float angToTurn;
	// number of object the rover has collided with so far
	float numObjsCollide;
	// should rotate by turning right (true) or rotate by turning left (false) 
	float turnRight;
	// current angular rotation of rover
	float currRotAng;

	int side;
	ALGORITHM_STATES states;
	
} ALGORITHM_DATA;
#ifdef	__cplusplus
}
#endif

#endif	/* ALGORITHM_H */
