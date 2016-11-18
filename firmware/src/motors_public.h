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
    STOP=0,
    CRAWL=300,
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

//motor settings for follower
#ifdef FOLLOWER

#define FORWARD 0
#define REVERSE 1
#define LEFT_CHANNEL PORT_CHANNEL_G 
#define LEFT_PORT PORTS_BIT_POS_1
#define LEFT_OC OC_ID_2
#define RIGHT_CHANNEL PORT_CHANNEL_C
#define RIGHT_PORT PORTS_BIT_POS_14
#define RIGHT_OC OC_ID_1

#else //motor settings for leader

#define FORWARD 1
#define REVERSE 0
#define LEFT_CHANNEL PORT_CHANNEL_C 
#define LEFT_PORT PORTS_BIT_POS_14
#define LEFT_OC OC_ID_1
#define RIGHT_CHANNEL PORT_CHANNEL_G
#define RIGHT_PORT PORTS_BIT_POS_1
#define RIGHT_OC OC_ID_2

#endif

// Follower rover
void set_speed(MOTOR_SPEEDS leftSpeed, MOTOR_SPEEDS rightSpeed);
void set_dist(int leftDist, int rightDist);
MOTOR_SPEEDS get_speed(SIDE side);
void inc_speed(SIDE side);
void dec_speed(SIDE side);
void move_start();
void move_stop();
void spin_right();
void spin_left();
void continue_moving(int leftDist, int rightDist);

MOVE_STATE getMoveState();

// Leader Rover
void turn_right();
void turn_left();
void move_forward();
void move_backward();
int get_distance(SIDE side);


#endif	/* MOTORS_PUBLIC_H */

