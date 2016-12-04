/* 
 * File:   brains_public.h
 * Author: Anirudha Simha
 *
 * Created on October 5, 2016, 4:20 PM
 */

#ifndef BRAINS_PUBLIC_H
#define	BRAINS_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

 
#include <math.h> 
#include <stdbool.h>
    
// Definitions of Values used in code 
#define OBSACLE_AVOIDANCE_DISTANCE 35 // the minimum distance the rover should travel upon encountering an obstacle in order to get past said obstacle 
#define ROTATION_AMOUNT 60
#define DEBUG_START_DIST 25
#define DEBUG_START_DEGREES_TO_TURN 45
#define START_CURRENT_ANG_ROT 0
//#define DEBUG_START true
#define OBJECT_DETECT_DISTANCE 0x23 // 50 (cm))
#define PI 3.14159265


// responsible for setting the turn at which rover will go when it encounters an obstacle
void turnHandler(); 
// calculation function to get a third leg (leg C) when doing law of cosines 
int16_t getLegC();
// algorithm initializer function 
void algoInit();
// gets how far the rover has traveled since its last local origin point 
int16_t getCurrentTravelDistance();
// set new local origin by messaging the motors that they should remeasure the distance traveled so far from the current point
    //void setLocalOrigin();


// Struct containing all of the algorithm's data 
typedef struct {
	// typically will be distToGo - <THE DISTANCE THE ROVER HAS TRAVELED SO FAR>
	int16_t legA; 
	// typically will be the distance the rover traveled after moving around an obstacle  
	int16_t legB;
	// Typically the angle between legA and legB
	int16_t angC;
	// dangle across from leg a
    int16_t angA;
	// distance rover is supposed to go to reach endpoint assuming no obstacles in way
	int16_t distToGo;
	// Angle rover should be at in order to be able to go straight and reach the endpoint
	int16_t angToTurn;
	// number of object the rover has collided with so far
	int16_t numObjsCollide;
	// should rotate by turning right (true) or rotate by turning left (false) 
	bool turnRight;
	// current angular rotation of rover
	int16_t currRotAng;
    // current distance traveled
    int16_t currDistTrav;
    
	
} ALGORITHM_DATA;

#ifdef	__cplusplus
}
#endif

#endif	/* BRAINS_PUBLIC_H */

