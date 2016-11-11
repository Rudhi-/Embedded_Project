#include "algorithm.h"

ALGORITHM_DATA algData; 

// Main algorithm function what will be called in the lead rover's loop
void algorithm() {
	switch (algData.states)
	{
	case FINDING_PATH:
		
		//legB here is the distance traveled from the most recent start point
		algData.legB = getCurrentTravelDistance();// = sqrt(sq(algData.position.x - algData.origin.x) + sq(algData.position.y - algData.origin.y));
		
		algData.distToGo = getLegC(algData.legA, algData.legB, algData.angToTurn);
		float temp = fmod(asin(algData.legA * (sin(algData.angC) / algData.distToGo)), MIN_ROTATION_INCREMENT);
		algData.angToTurn =  temp + asin(algData.legA * (sin(algData.angC) / algData.distToGo));
		
		// determine when if its faster to rotate right or faster to rotate left 
		if (algData.currRotAng < algData.angToTurn) {
				if (fabs(algData.currRotAng - algData.angToTurn) < 180) {
					algData.turnRight = true;
				} else {
					algData.turnRight = false;
				}
			} else {
				if (fabs(algData.currRotAng - algData.angToTurn) < 180) {
					algData.turnRight = false;
				} else {
					algData.turnRight = true;
				}
			}
		
		algData.states = ROTATING;
		break;
	case ROTATING:
		// TODO wait here until messaged is received saying that rover has finished rotating 
		
		// TODO Send message telling motors to move in desired direction 

		algData.states = TRAVELING_PATH;
		break;
	case TRAVELING_PATH:
		// TODO rover will be in a moving state at this point 
		break;
	case OBSTACLE_ENCOUNTERED:

		if (algData.numObjsCollide == 1) {
			
			
			algData.legA = algData.distToGo - getCurrentTravelDistance();
			turnHandler();
		}
		else if (algData.numObjsCollide >= 2) {
			algData.legB = getCurrentTravelDistance();
			algData.legA = getLegC(algData.legA, algData.legB, algData.angC);
			
			turnHandler();
		}
		else {
			// This Should never happen 
			break;
		}

		algData.currRotAng = fmod(algData.currRotAng,360);
		algData.angToTurn =	fmod(algData.angToTurn,360);
		// which way to rotate?

		float travelDist = getCurrentTravelDistance();

		if (travelDist < 1) { // this is so that if the rover immediately encountering another obstacle it doesn't turn a different direction. 
			algData.turnRight = algData.turnRight;
		} else {
			if (algData.currRotAng < algData.angToTurn) {
				if (fabs(algData.currRotAng - algData.angToTurn) < 180) {
					algData.turnRight = true;
				} else {
					algData.turnRight = false;
				}
			} else {
				if (fabs(algData.currRotAng - algData.angToTurn) < 180) {
					algData.turnRight = false;
				} else {
					algData.turnRight = true;
				}
			}
		}

		// Send message to motors that they need to start measuring distance traveled so far from this current position now 
		setLocalOrigin();


		algData.states = OBSTACLE_ENCOUNTERED_ROTATE;
		break;
	case OBSTACLE_ENCOUNTERED_ROTATE:
		// TODO wait here until messaged is received saying that rover has finished rotating	
		
		// TODO Send message telling motors to move in desired direction 
		
		algData.states = OBSTACLE_ENCOUNTERED_MOVE;
		break;
	case OBSTACLE_ENCOUNTERED_MOVE:
		// TODO rover will be in a moving state at this point 
		break;
	case FOUND_LINE:
		// TODO Send end message to rest to rovers telling what happened
		break;
	case FOUND_ENDPOINT:
		// TODO Send end message to rest to rovers telling what happened
		break;
	default:
		break;
	}
}

// TODO implement distance traveled so far function
float getCurrentTravelDistance() {
    return 0;
}
// TODO implement set new local origin method 
void setLocalOrigin() {

}




// responsible for setting the turn at which rover will go when it encounters an obstacle
void turnHandler() {
	int leftObjs = 0; // number of obstacles on left  (-60)
	int rightObjs = 0; // number of obstacles on right (+60)
	
    // LeftAng and rightAng are the angles we should glance in to see which side is better 
    //float leftAng = (algData.currRotAng - 90) - fmod((algData.currRotAng - 90),MIN_ROTATION_INCREMENT);
	//float rightAng = fmod((algData.currRotAng + 90),MIN_ROTATION_INCREMENT) + (algData.currRotAng + 90);

	// TODO scan left and right side of rover to determine which has the least amount of obstacles, distances of 40, 60, 80
	
	
	// "weight" adjustment based upon distance of possible path's endpoint to final endpoint
	float leftMag;
	float tempAng = 90;
	if (algData.numObjsCollide >= 2) {
		tempAng = 180 - asin(algData.legA * (sin(algData.angC) / algData.legB)) - 90;
	}

	leftMag = getLegC(algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);
	float rightMag;
	tempAng = 90;
	if (algData.numObjsCollide >= 2) {
		tempAng = 180 - asin(algData.legB * (sin(algData.angC) / algData.legA)) + 90;
	}
	
	rightMag = getLegC(algData.legA, OBSACLE_AVOIDANCE_DISTANCE, tempAng);

	if (leftMag < rightMag) {
		leftObjs -= 3;
	} else if (rightMag < leftMag) {
		rightObjs -= 3;
	} else {
		// do nothing no weight change
	}

	if (leftObjs > rightObjs) { //turn left
		algData.angToTurn = (algData.currRotAng - 90) - fmod((algData.currRotAng - 90), MIN_ROTATION_INCREMENT);
		if (algData.angToTurn < 0) {
			algData.angToTurn += 360;
		}
		algData.angToTurn= fmod(algData.angToTurn, 360);
		if (algData.numObjsCollide == 1) {
			algData.angC = 90;
		} else if (algData.numObjsCollide >= 2) {
			algData.angC = 180 - asin(algData.legB * (sin(algData.angC) / algData.legA)) - 90;
		}

		algData.side = 1;
	} else {
		// turn right
		algData.angToTurn = fmod((algData.currRotAng + 90), MIN_ROTATION_INCREMENT) + (algData.currRotAng + 90);
		if (algData.angToTurn < 0) {
			algData.angToTurn += 360;
		}
		algData.angToTurn = fmod(algData.angToTurn,360);
		if (algData.numObjsCollide == 1) {
			algData.angC = 90;
		} else if (algData.numObjsCollide >= 2) {
			algData.angC = 180 - asin(algData.legB * (sin(algData.angC) / algData.legA)) + 90;
		}
		algData.side = -1;
	}
}

// calculation function to get a third leg (leg C) when doing law of cosines 
float getLegC() {
	return sqrt(pow(algData.legA,2) + pow(algData.legB,2) - (2 * algData.legA * algData.legB * cos(algData.angC)));
}
// algorithm initializer function 
void algoInit(){
	algData.legA = 0;
	algData.legB = 0;
	algData.angC = 0;
	algData.distToGo = 0;
	algData.angToTurn = 0;
	algData.numObjsCollide = 0;
	algData.turnRight = true;
	algData.states = 0;
	algData.currRotAng = 0;
}