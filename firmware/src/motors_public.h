/* 
 * File:   motors_public.h
 * Author: Ryan
 *
 * Created on October 5, 2016, 3:08 PM
 */

#ifndef MOTORS_PUBLIC_H
#define	MOTORS_PUBLIC_H

typedef enum
{     
    NEGATIVE=-1, //unused value to ensure enum is signed
    WALK=400,
    JOG=600,
    RUN=800,
    SPRINT=1000,            
} MOTOR_SPEEDS;

typedef enum
{
    LEFT=0,
    RIGHT=1,
} SIDE;

typedef enum
{
    INIT=0,
    TEST,
    WAIT,
    MOVE,
} MOVE_STATE;

QueueHandle_t MessageQueueM;

#define MOTOR_THREAD_ID 0x01

// Follower rover
void set_speed(MOTOR_SPEEDS leftSpeed, MOTOR_SPEEDS rightSpeed);
void set_dist(int leftDist, int rightDist);
MOTOR_SPEEDS get_speed(SIDE side);
void move_start();
void move_stop();
void spin_right();
void spin_left();

MOVE_STATE getMoveState();

// Leader Rover
void turn_right();
void turn_left();
void move_forward();
void move_backward();
int get_distance(SIDE side);


#endif	/* MOTORS_PUBLIC_H */

