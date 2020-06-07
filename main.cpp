#include "mbed.h"
#include <cmath>

#define OOO 0
#define OOI 1
#define OIO 2
#define OII 3
#define IOO 4
#define IOI 5
#define IIO 6
#define III 7

#define MOTOR_LEFT_CLOCK_PIN p12
#define MOTOR_RIGHT_CLOCK_PIN p5
#define MOTOR_LEFT_DIR_PIN p6
#define MOTOR_RIGHT_DIR_PIN p7
#define MOTOR_LEFT_RESET_PIN p11
#define MOTOR_RIGHT_RESET_PIN p10
#define MOTOR_LEFT_TOR_PIN p8
#define MOTOR_RIGHT_TOR_PIN p9

#define LEFT_WING_LEFT_PIN p23
#define LEFT_WING_MID_PIN p22
#define LEFT_WING_RIGHT_PIN p21
#define RIGHT_WING_LEFT_PIN p26
#define RIGHT_WING_MID_PIN p25
#define RIGHT_WING_RIGHT_PIN p24

#define LEFT_DIST_PIN p15
#define FRONT_DIST_PIN p20
#define RIGHT_DIST_PIN p17
#define FRONT_PROX_PIN p27

#define NO_MOVE 0
#define IN_MOVE 1
#define DANCE_BLOCKED 0
#define DANCE_ALLOWED 1
#define FORWARD_CONTINUE 1
#define FORWARD_NO_CONTINUE 0
#define AT_Q4 1
#define NOT_AT_Q4 0
#define EMERGENCY_ON 1
#define EMERGENCY_OFF 0

#define IN_TURN 0
#define NOT_IN_TURN 1

#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

#define FORWARD 0
#define BACKWARD 1
#define TURN_LEFT 2
#define TURN_RIGHT 3

#define POSITIVE 0
#define NEGATIVE 1

#define MOTOR_ACC 25
#define MOTOR_MAX_FREQUENCY 300
#define MOTOR_START_FREQUENCY 100

#define IS_WALL 0
#define NO_WALL 1
#define UNKNOWN -3
#define PASS -4
#define NO_PASS -5

#define MAZE_SIZE 32
#define INIT_ROW 16
#define INIT_COL 16
#define END_ROW 1
#define END_COL 1
#define STEPS_PER_SQUARE 310
#define STEPS_PER_TURN 116

#define DIST_SIDE_IS_WALL_UB (3.3 / 3.3)
#define DIST_SIDE_IS_WALL_LB (1.3 / 3.3)

#define DIST_SIDE_IS_WALL_FAR_UB (0.7 / 3.3)
#define DIST_SIDE_IS_WALL_FAR_LB (0.2 / 3.3)

#define DIST_FRONT_IS_WALL_FAR_UB (0.7 / 3.3)
#define DIST_FRONT_IS_WALL_FAR_LB (0.2 / 3.3)

#define DIST_FRONT_IS_WALL_LB (2 / 3.3)
#define DIST_FRONT_IS_WALL_UB (3.3 / 3.3)
#define WALL_APPROACH_UB (3.05 / 3.3)
#define WALL_APPROACH_LB (1.05 / 3.3)
#define SWEET_SPOT_UB (3.05 / 3.3)
#define SWEET_SPOT_LB (2.85 / 3.3)

#define WALL_APPROACHING (((frontDistance <= WALL_APPROACH_UB) && (frontDistance >= WALL_APPROACH_LB)))
#define IS_WALL_ON_RIGHT (((rightDistance <= DIST_SIDE_IS_WALL_UB) && (rightDistance >= DIST_SIDE_IS_WALL_LB)))
#define IS_WALL_ON_RIGHT_FAR (((rightDistance <= DIST_SIDE_IS_WALL_FAR_UB) && (rightDistance >= DIST_SIDE_IS_WALL_FAR_LB)))
#define IS_WALL_ON_LEFT (((leftDistance <= DIST_SIDE_IS_WALL_UB) && (leftDistance >= DIST_SIDE_IS_WALL_LB)))
#define IS_WALL_ON_LEFT_FAR (((leftDistance <= DIST_SIDE_IS_WALL_FAR_UB) && (leftDistance >= DIST_SIDE_IS_WALL_FAR_LB)))

#define IS_WALL_IN_FRONT (((frontDistance <= DIST_FRONT_IS_WALL_UB) && (frontDistance >= DIST_FRONT_IS_WALL_LB)))
#define IS_WALL_IN_FRONT_FAR (((frontDistance <= DIST_FRONT_IS_WALL_FAR_UB) && (frontDistance >= DIST_FRONT_IS_WALL_FAR_LB)))

#define FRONT_SWEET_SPOT (((frontDistance >= SWEET_SPOT_LB) && (frontDistance < SWEET_SPOT_UB)))

struct square
{
    int north, south, west, east, value;
    
} maze[MAZE_SIZE][MAZE_SIZE];

struct position
{
    int row, col, direction;
} currentPosition, initialPosition, endPosition;

struct state
{
    int movement, dance, forward, speed, q4, emergency, turning;
} mouseState;

DigitalOut motorLeftClock(MOTOR_LEFT_CLOCK_PIN);
DigitalOut motorRightClock(MOTOR_RIGHT_CLOCK_PIN);
DigitalOut motorLeftDirection(MOTOR_LEFT_DIR_PIN);
DigitalOut motorRightDirection(MOTOR_RIGHT_DIR_PIN);
DigitalOut motorLeftTorque(MOTOR_LEFT_TOR_PIN);
DigitalOut motorRightTorque(MOTOR_RIGHT_TOR_PIN);
DigitalOut motorLeftReset(MOTOR_LEFT_RESET_PIN);
DigitalOut motorRightReset(MOTOR_RIGHT_RESET_PIN);

BusIn rightWing(LEFT_WING_LEFT_PIN, LEFT_WING_MID_PIN, LEFT_WING_RIGHT_PIN);
BusIn leftWing(RIGHT_WING_LEFT_PIN, RIGHT_WING_MID_PIN, RIGHT_WING_RIGHT_PIN);

DigitalIn frontProximity(FRONT_PROX_PIN);

AnalogIn leftDistance(LEFT_DIST_PIN);
AnalogIn frontDistance(FRONT_DIST_PIN);
AnalogIn rightDistance(RIGHT_DIST_PIN);

Ticker TM;
int squaresRemaining;
int halfStepsCount;
int nextMovement;
int numberOfWaits;
int numberOfDances;

int nextSolvedMove();
void initialise_maze();
void initialize_mouse();
void moveForward();
void turnLeft();
void turnRight();
void motor_turn();
void motor_forward();
bool compareSquares(struct position a, struct position b);
void updatePosition();
int nextMove();
void map();
void lee();
void solveMaze();
void moveR32();
void moveRR31();
void moveL23();
void moveLL13();
void verticalCorrection();
void miniDanceCorrection(int radius);
void crossCorrection();
void straightLineCorrection();
void doubleWallDynamicCorrection();
void rightWallOnlyDynamicCorrection();
void leftWallOnlyDynamicCorrection();
void dynamicAdvanceCorrection();

DigitalOut test1(LED1);
DigitalOut test2(LED2);
DigitalOut test3(LED3);
DigitalOut test4(LED4);



int main()
{
    initialize_mouse();
    do
    {
        while (((mouseState.q4 == NOT_AT_Q4) && (mouseState.movement == IN_MOVE)) || (mouseState.turning == IN_TURN))
        {
            wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
        }
        test1 = 1;
        wait(0.1);
        test1 = 0;
        mouseState.forward = FORWARD_NO_CONTINUE;
        map();
        map();
        map();
        nextMovement = nextMove();
        updatePosition();
        if (nextMovement == FORWARD)
        {
            mouseState.forward = FORWARD_CONTINUE;
            moveForward();
            while ((mouseState.q4 == AT_Q4) && (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
        }
        else if (nextMovement == TURN_LEFT)
        {
            
            test2 = 1;
            wait(0.1);
            test2 = 0;
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            
            crossCorrection();
            turnLeft();
            verticalCorrection();
            moveForward();
        }
        else if (nextMovement == TURN_RIGHT)
        {
            
            test3 = 1;
            wait(0.1);
            test3 = 0;
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            
            crossCorrection();
            turnRight();
            verticalCorrection();
            moveForward();
        }
        else
        {
                
            test4 = 1;
            wait(0.1);
            test4 = 0;
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            crossCorrection();
            moveForward();
        }
    } while (1);//!compareSquares(currentPosition, endPosition));
    
    wait(1);
    
    solveMaze();
    endPosition.col = INIT_COL;
    endPosition.row = INIT_ROW;
    wait(1);
    do
    {
        while (((mouseState.q4 == NOT_AT_Q4) && (mouseState.movement == IN_MOVE)) || (mouseState.turning == IN_TURN))
        {
            wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
        }
        test1 = 1;
        wait(0.1);
        test1 = 0;
        mouseState.forward = FORWARD_NO_CONTINUE;
        nextMovement = nextSolvedMove();
        updatePosition();
        if (nextMovement == FORWARD)
        {
            mouseState.forward = FORWARD_CONTINUE;
            moveForward();
            while ((mouseState.q4 == AT_Q4) && (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
        }
        else if (nextMovement == TURN_LEFT)
        {
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            crossCorrection();
            turnLeft();
            verticalCorrection();
            moveForward();
        }
        else if (nextMovement == TURN_RIGHT)
        {
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            crossCorrection();
            turnRight();
            verticalCorrection();
            moveForward();
        }
        else
        {
            while ((mouseState.q4 == AT_Q4) || (mouseState.movement == IN_MOVE))
            {
                wait(1.0 / (2 * MOTOR_MAX_FREQUENCY));
            }
            crossCorrection();
            moveForward();
        }
    } while (!compareSquares(currentPosition, endPosition));
}

int nextSolvedMove()
{
    if (maze[currentPosition.row][currentPosition.col].value > maze[currentPosition.row - 1][currentPosition.col].value)
    {
        if (currentPosition.direction == NORTH)
        {
            return FORWARD;
        }
        else if (currentPosition.direction == SOUTH)
        {
            return BACKWARD;
        }
        else if (currentPosition.direction == EAST)
        {
            return TURN_RIGHT;
        }
        else
        {
            return TURN_LEFT;
        }
    }
    else if (maze[currentPosition.row][currentPosition.col].value > maze[currentPosition.row + 1][currentPosition.col].value)
    {
        if (currentPosition.direction == NORTH)
        {
            return BACKWARD;
        }
        else if (currentPosition.direction == SOUTH)
        {
            return FORWARD;
        }
        else if (currentPosition.direction == EAST)
        {
            return TURN_RIGHT;
        }
        else
        {
            return TURN_LEFT;
        }
    }
    else if (maze[currentPosition.row][currentPosition.col].value > maze[currentPosition.row][currentPosition.col - 1].value)
    {
        if (currentPosition.direction == NORTH)
        {
            return TURN_LEFT;
        }
        else if (currentPosition.direction == SOUTH)
        {
            return TURN_RIGHT;
        }
        else if (currentPosition.direction == EAST)
        {
            return BACKWARD;
        }
        else
        {
            return FORWARD;
        }
    }
    else if (maze[currentPosition.row][currentPosition.col].value > maze[currentPosition.row][currentPosition.col + 1].value)
    {
        if (currentPosition.direction == NORTH)
        {
            return TURN_RIGHT;
        }
        else if (currentPosition.direction == SOUTH)
        {
            return TURN_LEFT;
        }
        else if (currentPosition.direction == EAST)
        {
            return FORWARD;
        }
        else
        {
            return BACKWARD;
        }
    }
    return FORWARD;
}

void initialise_maze()
{
    int i, j;
    for (i = 0; i < MAZE_SIZE; i++)
    {
        for (j = 0; j < MAZE_SIZE; j++)
        {
            maze[i][j].north = UNKNOWN;
            maze[i][j].south = UNKNOWN;
            maze[i][j].east = UNKNOWN;
            maze[i][j].west = UNKNOWN;
            maze[i][j].value = UNKNOWN;
        }
    }
}

void initialize_mouse()
{
    initialise_maze();
    mouseState.speed = 0;
    mouseState.turning = NOT_IN_TURN;
    mouseState.emergency = EMERGENCY_OFF;
    currentPosition.direction = EAST;
    currentPosition.row = INIT_ROW;
    currentPosition.col = INIT_COL;
    endPosition.row = END_ROW;
    endPosition.col = END_COL;
    maze[currentPosition.row][currentPosition.col].west = IS_WALL;
    maze[currentPosition.row][currentPosition.col].value = PASS;
    mouseState.q4 = NOT_AT_Q4;
    mouseState.forward = FORWARD_NO_CONTINUE;
    halfStepsCount = 0;
    squaresRemaining = 0;
    motorLeftReset.write(1);
    motorRightReset.write(1);
    motorLeftTorque.write(1);
    motorRightTorque.write(1);
    mouseState.movement = NO_MOVE;
    wait(3);
}

void moveForward()
{
    if (mouseState.movement == IN_MOVE)
    {
        squaresRemaining++;
    }
    else
    {
        mouseState.movement = IN_MOVE;
        mouseState.q4 = NOT_AT_Q4;
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(POSITIVE);
        if (mouseState.speed == 0)
            mouseState.speed = MOTOR_START_FREQUENCY;
        squaresRemaining++;
        TM.attach(&motor_forward, 1.0 / mouseState.speed);
    }
}

void turnLeft()
{
    while (mouseState.movement == IN_MOVE)
    {
        wait(1.0 / MOTOR_MAX_FREQUENCY);
    }
    mouseState.turning = IN_TURN;
    mouseState.movement = IN_MOVE;
    motorRightDirection.write(POSITIVE);
    motorLeftDirection.write(NEGATIVE);
    if (mouseState.speed == 0)
        mouseState.speed = MOTOR_START_FREQUENCY;
    TM.attach(&motor_turn, 1.0 / mouseState.speed);
    while (mouseState.movement == IN_MOVE)
    {
        wait(1.0 / MOTOR_MAX_FREQUENCY);
    }
}

void turnRight()
{
    while (mouseState.movement == IN_MOVE)
    {
        wait(1.0 / MOTOR_MAX_FREQUENCY);
    }
    mouseState.movement = IN_MOVE;
    motorLeftDirection.write(POSITIVE);
    motorRightDirection.write(NEGATIVE);
    if (mouseState.speed == 0)
        mouseState.speed = MOTOR_START_FREQUENCY;
    TM.attach(&motor_turn, 1.0 / mouseState.speed);
    while (mouseState.movement == IN_MOVE)
    {
        wait(1.0 / MOTOR_MAX_FREQUENCY);
    }
}

void motor_turn()
{
    motorLeftClock.write(!motorLeftClock.read());
    motorRightClock.write(!motorRightClock.read());
    halfStepsCount++;
    if (halfStepsCount == (2 * STEPS_PER_TURN))
    {
        mouseState.speed = 0;
        halfStepsCount = 0;
        mouseState.movement = NO_MOVE;
        TM.detach();
    }
    else 
    {
        if (((halfStepsCount % 20) == 0))
        {
            if (((2 * STEPS_PER_TURN) - halfStepsCount) > ((2 * STEPS_PER_TURN) / 4))
            {
                if (mouseState.speed != MOTOR_MAX_FREQUENCY)
                {
                    TM.attach(&motor_turn, 1.0 / ((mouseState.speed + MOTOR_ACC) < MOTOR_MAX_FREQUENCY ? mouseState.speed = mouseState.speed + MOTOR_ACC : mouseState.speed = MOTOR_MAX_FREQUENCY));
                }
            }
            else
            {
                if ((mouseState.speed != MOTOR_START_FREQUENCY))
                {
                    TM.attach(&motor_turn, 1.0 / ((mouseState.speed - MOTOR_ACC) > MOTOR_START_FREQUENCY ? mouseState.speed = mouseState.speed - MOTOR_ACC : mouseState.speed = MOTOR_START_FREQUENCY));
                }
            }
        }
    }
}

void motor_forward()
{
    if (mouseState.emergency == EMERGENCY_ON)
    {
        if (numberOfWaits > 0)
        {
            numberOfWaits--;
        }
        else
        {
            mouseState.emergency = EMERGENCY_OFF;
            mouseState.q4 = NOT_AT_Q4;
            mouseState.movement = NO_MOVE;
            TM.detach();
        }
    }
    else
    {
        if (frontProximity != NO_WALL)
        {
            mouseState.speed = 0;
            squaresRemaining = 0;
            halfStepsCount = 0;
            mouseState.q4 = AT_Q4;
            mouseState.emergency = EMERGENCY_ON;
            numberOfWaits = 10;
            TM.attach(&motor_forward, 1.0 / MOTOR_START_FREQUENCY);
        }
        else if (squaresRemaining)
        {
            motorLeftClock.write(!motorLeftClock.read());
            motorRightClock.write(!motorRightClock.read());
            halfStepsCount++;
            if ((((2 * STEPS_PER_SQUARE) - halfStepsCount) < ((2 * STEPS_PER_SQUARE) / 4)))
            {
                mouseState.q4 = AT_Q4;
            }
            else
            {
                mouseState.q4 = NOT_AT_Q4;
            }
            if (halfStepsCount == (2 * STEPS_PER_SQUARE))
            {
                squaresRemaining--;
                halfStepsCount = 0;
                mouseState.q4 = NOT_AT_Q4;
            }
            else
            {
                if ((halfStepsCount % 16) == 0)
                {
                    if (WALL_APPROACHING)
                    {
                        if (mouseState.q4 == AT_Q4)
                        {
                            if (mouseState.forward == FORWARD_NO_CONTINUE)
                            {
                                if (mouseState.speed != MOTOR_START_FREQUENCY)
                                {
                                    TM.attach(&motor_forward, 1.0 / ((mouseState.speed - MOTOR_ACC) > MOTOR_START_FREQUENCY ? mouseState.speed = mouseState.speed - MOTOR_ACC : mouseState.speed = MOTOR_START_FREQUENCY));
                                }
                            }
                        }
                        else
                        {
                            TM.attach(&motor_forward, 1.0 / ((mouseState.speed - MOTOR_ACC) > MOTOR_START_FREQUENCY ? mouseState.speed = mouseState.speed - MOTOR_ACC : mouseState.speed = MOTOR_START_FREQUENCY));
                        }
                    }
                    else
                    {
                        if ((mouseState.speed != MOTOR_MAX_FREQUENCY))
                        {
                            TM.attach(&motor_forward, 1.0 / ((mouseState.speed + MOTOR_ACC) < MOTOR_MAX_FREQUENCY ? mouseState.speed = mouseState.speed + MOTOR_ACC : mouseState.speed = MOTOR_MAX_FREQUENCY));
                        }
                    }
                }
                if ((halfStepsCount % 4) == 0)
                {
                    dynamicAdvanceCorrection();
                }
            }
        }
        else
        {
            mouseState.speed = 0;
            squaresRemaining = 0;
            halfStepsCount = 0;
            mouseState.movement = NO_MOVE;
            mouseState.q4 = NOT_AT_Q4;
            TM.detach();
        }
    }
    if (mouseState.turning == IN_TURN)
    {
        mouseState.turning = NOT_IN_TURN;
    }
}

bool compareSquares(struct position a, struct position b)
{
    if ((a.col == b.col) && (b.row == a.row))
        return true;
    else
        return false;
}

void updatePosition()
{
    if (currentPosition.direction == NORTH)
    {
        if (nextMovement == FORWARD)
        {
            currentPosition.row--;
        }
        else if (nextMovement == BACKWARD)
        {
            currentPosition.row++;
            currentPosition.direction = SOUTH;
        }
        else if (nextMovement == TURN_LEFT)
        {
            currentPosition.col--;
            currentPosition.direction = WEST;
        }
        else
        {
            currentPosition.col++;
            currentPosition.direction = EAST;
        }
    }
    else if (currentPosition.direction == SOUTH)
    {
        if (nextMovement == FORWARD)
        {
            currentPosition.row++;
        }
        else if (nextMovement == BACKWARD)
        {
            currentPosition.row--;
            currentPosition.direction = NORTH;
        }
        else if (nextMovement == TURN_LEFT)
        {
            currentPosition.direction = EAST;
            currentPosition.col++;
        }
        else
        {
            currentPosition.direction = WEST;
            currentPosition.col--;
        }
    }
    else if (currentPosition.direction == EAST)
    {
        if (nextMovement == FORWARD)
        {
            currentPosition.col++;
        }
        else if (nextMovement == BACKWARD)
        {
            currentPosition.col--;
            currentPosition.direction = WEST;
        }
        else if (nextMovement == TURN_LEFT)
        {
            currentPosition.row--;
            currentPosition.direction = NORTH;
        }
        else
        {
            currentPosition.row++;
            currentPosition.direction = SOUTH;
        }
    }
    else
    {
        if (nextMovement == FORWARD)
        {
            currentPosition.col--;
        }
        else if (nextMovement == BACKWARD)
        {
            currentPosition.col++;
            currentPosition.direction = EAST;
        }
        else if (nextMovement == TURN_LEFT)
        {
            currentPosition.row++;
            currentPosition.direction = SOUTH;
        }
        else
        {
            currentPosition.row--;
            currentPosition.direction = NORTH;
        }
    }
}

int nextMove()
{
    if (currentPosition.direction == NORTH)
    {
        if (maze[currentPosition.row][currentPosition.col].east == NO_WALL)
            return TURN_RIGHT;
        else if (maze[currentPosition.row][currentPosition.col].north == NO_WALL)
            return FORWARD;
        else if (maze[currentPosition.row][currentPosition.col].west == NO_WALL)
            return TURN_LEFT;
        else
            return BACKWARD;
    }
    else if (currentPosition.direction == SOUTH)
    {   
        if (maze[currentPosition.row][currentPosition.col].west == NO_WALL)
            return TURN_RIGHT;
        else if (maze[currentPosition.row][currentPosition.col].south == NO_WALL)
            return FORWARD;
        else if (maze[currentPosition.row][currentPosition.col].east == NO_WALL)
            return TURN_LEFT;
        else
            return BACKWARD;
    }
    else if (currentPosition.direction == WEST)
    {   
        if (maze[currentPosition.row][currentPosition.col].north == NO_WALL)
            return TURN_RIGHT;
        else if (maze[currentPosition.row][currentPosition.col].west == NO_WALL)
            return FORWARD;
        else if (maze[currentPosition.row][currentPosition.col].south == NO_WALL)
            return TURN_LEFT;
        else
            return BACKWARD;
    }
    else
    {   
        if (maze[currentPosition.row][currentPosition.col].south == NO_WALL)
            return TURN_RIGHT;
        else if (maze[currentPosition.row][currentPosition.col].east == NO_WALL)
            return FORWARD;
        else if (maze[currentPosition.row][currentPosition.col].north == NO_WALL)
            return TURN_LEFT;
        else
            return BACKWARD;
    }
}

void map()
{
    maze[currentPosition.row][currentPosition.col].value = PASS;
    if (IS_WALL_ON_LEFT)
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].west = IS_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].east = IS_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].south = IS_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].north = IS_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
    }
    if (IS_WALL_ON_RIGHT)
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].east = IS_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].west = IS_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].north = IS_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].south = IS_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
    }
    if (IS_WALL_IN_FRONT || (frontProximity == IS_WALL))
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].north = IS_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].south = IS_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].west = IS_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].east = IS_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
    }
    
    if (IS_WALL_ON_LEFT_FAR)
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = IS_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = IS_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = IS_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = IS_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
    }
    if (IS_WALL_ON_RIGHT_FAR)
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = IS_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = IS_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = IS_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = IS_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
    }
    if (IS_WALL_IN_FRONT_FAR)
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = IS_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = IS_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = IS_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = IS_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
    }
    else
    {
        if (currentPosition.direction == NORTH)
        {
            maze[currentPosition.row - 1][currentPosition.col].value = PASS;
            maze[currentPosition.row - 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row - 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row][currentPosition.col].north = NO_WALL;
        }
        else if (currentPosition.direction == SOUTH)
        {
            maze[currentPosition.row + 1][currentPosition.col].value = PASS;
            maze[currentPosition.row + 1][currentPosition.col].south = NO_WALL;
            maze[currentPosition.row + 1][currentPosition.col].north = NO_WALL;
            maze[currentPosition.row][currentPosition.col].south = NO_WALL;
        }
        else if (currentPosition.direction == WEST)
        {
            maze[currentPosition.row][currentPosition.col - 1].value = PASS;
            maze[currentPosition.row][currentPosition.col - 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col - 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col].west = NO_WALL;
        }
        else
        {
            maze[currentPosition.row][currentPosition.col + 1].value = PASS;
            maze[currentPosition.row][currentPosition.col + 1].east = NO_WALL;
            maze[currentPosition.row][currentPosition.col + 1].west = NO_WALL;
            maze[currentPosition.row][currentPosition.col].east = NO_WALL;
        }
    }
    
    if (currentPosition.direction == NORTH)
    {
        maze[currentPosition.row][currentPosition.col].south = NO_WALL;
    }
    else if (currentPosition.direction == SOUTH)
    {
        maze[currentPosition.row][currentPosition.col].north = NO_WALL;
    }
    else if (currentPosition.direction == EAST)
    {
        maze[currentPosition.row][currentPosition.col].west = NO_WALL;
    }
    else
    {
        maze[currentPosition.row][currentPosition.col].east = NO_WALL;
    }
}

void lee()
{
    int value, col, row, i, j;
    bool detect;
    col = INIT_COL;
    row = INIT_ROW;
    value = 0;
    maze[row][col].value = value;
    while (maze[currentPosition.row][currentPosition.col].value == PASS)
    {
        detect = true;
        while (detect)
        {
            detect = false;
            for (i = 0; i < MAZE_SIZE; i++)
            {
                for (j = 0; j < MAZE_SIZE; j++)
                {
                    if (maze[i][j].value == value)
                    {
                        if ((i - 1) >= 0)
                        {
                            if ((maze[i][j].north == NO_WALL) && (maze[i - 1][j].value == PASS))
                            {
                                detect = true;
                                maze[i - 1][j].value = value + 1;
                            }
                        }
                        if ((i + 1) < MAZE_SIZE)
                        {
                            if ((maze[i][j].south == NO_WALL) && (maze[i + 1][j].value == PASS))
                            {
                                detect = true;
                                maze[i + 1][j].value = value + 1;
                            }
                        }
                        if ((j - 1) >= 0)
                        {
                            if ((maze[i][j].west == NO_WALL) && (maze[i][j - 1].value == PASS))
                            {
                                detect = true;
                                maze[i][j - 1].value = value + 1;
                            }
                        }
                        if ((j + 1) < MAZE_SIZE)
                        {
                            if ((maze[i][j].east == NO_WALL) && (maze[i][j + 1].value == PASS))
                            {
                                detect = true;
                                maze[i][j + 1].value = value + 1;
                            }
                        }
                    }
                }
            }
        }
        value++;
    }
}


void solveMaze()
{
    int i, j;
    for (i = 0; i < MAZE_SIZE; i++)
    {
        for (j = 0; j < MAZE_SIZE; j++)
        {
            if (maze[i][j].value == UNKNOWN)
            {
                maze[i][j].value = NO_PASS;
            }
            if (maze[i][j].north == UNKNOWN)
            {
                maze[i][j].north = IS_WALL;
            }
            if (maze[i][j].south == UNKNOWN)
            {
                maze[i][j].south = IS_WALL;
            }
            if (maze[i][j].east == UNKNOWN)
            {
                maze[i][j].east = IS_WALL;
            }
            if (maze[i][j].west == UNKNOWN)
            {
                maze[i][j].west = IS_WALL;
            }
        }
    }
    lee();
}

void moveR32()
{
    
    float fullPeriod = 1.0 / (1.25 * mouseState.speed);
    if (fullPeriod == 0)
    {
        fullPeriod = 1.0 / MOTOR_START_FREQUENCY;
    }
    
    wait(fullPeriod);
    motorLeftClock.write(!motorLeftClock.read());
    
    wait(fullPeriod);
    motorLeftClock.write(!motorLeftClock.read());
}

void moveRR31()
{
    moveR32();
}

void moveL23()
{
    float fullPeriod = 1.0 / (1.25 * mouseState.speed);
    if (fullPeriod == 0)
    {
        fullPeriod = 1.0 / MOTOR_START_FREQUENCY;
    }
    wait(fullPeriod);
    motorRightClock.write(!motorRightClock.read());
    wait(fullPeriod);
    motorRightClock.write(!motorRightClock.read());
}

void moveLL13()
{
    moveL23();
}

void verticalCorrection()
{
    int i;
    if (IS_WALL_IN_FRONT || (frontProximity == IS_WALL))
    {
        if (frontProximity == NO_WALL)
        {
            motorLeftDirection.write(POSITIVE);
            motorRightDirection.write(POSITIVE);
            while ((frontProximity != IS_WALL))
            {
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);    
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);    
            }
        }
        else
        {
            motorLeftDirection.write(NEGATIVE);
            motorRightDirection.write(NEGATIVE);
            i = 0;
            while ((i < (STEPS_PER_SQUARE / 3) && (frontProximity == IS_WALL)))
            {
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);    
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);
                i++;
            }
            if (i >= (STEPS_PER_SQUARE / 3))
            {                        
                motorLeftDirection.write(POSITIVE);
                motorRightDirection.write(POSITIVE);
                while ((frontProximity == NO_WALL))
                {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);    
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                }
            }
        }
    
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(POSITIVE);
        for (i = 0; i < 16; i++)
        {
            motorLeftClock.write(!motorLeftClock.read());
            motorRightClock.write(!motorRightClock.read());
            wait(1.0 / MOTOR_START_FREQUENCY);    
            motorLeftClock.write(!motorLeftClock.read());
            motorRightClock.write(!motorRightClock.read());
            wait(1.0 / MOTOR_START_FREQUENCY);    
        
        }        
    }
}

void miniDanceCorrection(int radius)
{
    int i;    
    bool detect;
    if (((leftWing == III) && (rightWing == III)) && (IS_WALL_ON_LEFT || IS_WALL_ON_RIGHT))
    {
        detect = false;
        i = 0;
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(NEGATIVE);
        while ((i < radius) && (detect == false))
        {
            if ((leftWing == III) && (rightWing == III))
            {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
            }
            else
            {
                motorLeftDirection.write(POSITIVE);
                motorRightDirection.write(POSITIVE);
                for (i = 0; i < 6; i++)
                {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                }
                detect = true;
            }
            i++;
        }
        motorRightDirection.write(POSITIVE);
        motorLeftDirection.write(NEGATIVE);
        i = 0;
        while ((i < (2 * radius)) && (detect == false))
        {
            if ((leftWing == III) && (rightWing == III))
            {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);    
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
            }
            else
            {
                motorLeftDirection.write(POSITIVE);
                motorRightDirection.write(POSITIVE);
                for (i = 0; i < 6; i++)
                {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                }
                detect = true;
            }
            i++;
        }
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(NEGATIVE);
        i = 0;
        while ((i < radius) && (detect == false))
        {
            if ((leftWing == III) && (rightWing == III))
            {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);    
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
            }
            else
            {
                motorLeftDirection.write(POSITIVE);
                motorRightDirection.write(POSITIVE);
                for (i = 0; i < 6; i++)
                {
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                    motorLeftClock.write(!motorLeftClock.read());
                    motorRightClock.write(!motorRightClock.read());
                    wait(1.0 / MOTOR_START_FREQUENCY);
                }
                detect = true;
            }
            i++;
        }
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(POSITIVE);
    }
    if (!detect)
    {
        motorLeftDirection.write(POSITIVE);
        motorRightDirection.write(POSITIVE);
        for (i = 0; i < 10; i++)
        {
            motorLeftClock.write(!motorLeftClock.read());
            motorRightClock.write(!motorRightClock.read());
            wait(1.0 / MOTOR_START_FREQUENCY);
            motorLeftClock.write(!motorLeftClock.read());
            motorRightClock.write(!motorRightClock.read());
            wait(1.0 / MOTOR_START_FREQUENCY);
        }
        if (numberOfDances > 0)
        {
            numberOfDances--;
            miniDanceCorrection(radius + (radius / 2));
        }
        else
        {
            for (i = 0; i < 14; i++)
            {
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);
                motorLeftClock.write(!motorLeftClock.read());
                motorRightClock.write(!motorRightClock.read());
                wait(1.0 / MOTOR_START_FREQUENCY);
            }
            numberOfDances = 4;
            miniDanceCorrection(15);
        }
    }
}

void crossCorrection()
{
    if (IS_WALL_IN_FRONT || (frontProximity == IS_WALL))
    {
        if (IS_WALL_ON_LEFT && IS_WALL_ON_RIGHT)
        {
            verticalCorrection();
            turnLeft();
            verticalCorrection();
            if (nextMovement == BACKWARD)
            {
                turnLeft();
            }
            else
            {
                turnRight();
            }
        }
        else if ((!IS_WALL_ON_LEFT) && IS_WALL_ON_RIGHT)
        {
            verticalCorrection();
            turnRight();
            verticalCorrection();
            turnLeft();
        }
        else if (IS_WALL_ON_LEFT && (!IS_WALL_ON_RIGHT))
        {
            verticalCorrection();
            turnLeft();
            verticalCorrection();
            turnRight();
        }
    }
    else
    {
        if ((!IS_WALL_ON_LEFT) && IS_WALL_ON_RIGHT)
        {
            turnRight();
            verticalCorrection();
            turnLeft();
            verticalCorrection();
        }
        else if (IS_WALL_ON_LEFT && (!IS_WALL_ON_RIGHT))
        {
            turnLeft();
            verticalCorrection();
            turnRight();
            verticalCorrection();
        }
    }
}

void doubleWallDynamicCorrection()
{
    motorLeftDirection.write(POSITIVE);
    motorRightDirection.write(POSITIVE);
    if ((rightWing == OOI) && (leftWing == OII))
    {
        moveL23();
    }
    else if ((rightWing == OII) && (leftWing == OII))
    {
        moveL23();
    }
    else if ((rightWing == IIO) && (leftWing == IIO))
    {
        moveRR31();
    }
    else if ((rightWing == IOO) && (leftWing == IIO))
    {
        moveRR31();
    }
    else if ((rightWing == OOI) && (leftWing == OOI))
    {
        moveL23();
    }
    else if ((rightWing == OOI) && (leftWing == IOI))
    {       
        moveL23();
    }
    else if ((rightWing == OII) && (leftWing == OOI))
    {
        moveL23();
    }
    else if ((rightWing == OII) && (leftWing == IOO))
    {   
        moveL23();
    }
    else if ((rightWing == OII) && (leftWing == IOI))
    {
        moveL23();
    }
    else if ((rightWing == IOO) && (leftWing == OII))
    {
       moveL23();
    }
    else if ((rightWing == IOI) && (leftWing == OOI))
    {
        moveL23();
    }
    else if ((rightWing == IOI) && (leftWing == OII))
    {
        moveL23();
    }
    else if ((rightWing == IOO) && (leftWing == IOO))
    {
        moveR32();
    }
    else if ((rightWing == IOO) && (leftWing == IOI))
    {
        moveR32();
    }
    else if ((rightWing == IOI) && (leftWing == IOO))
    {
        moveR32();
    }
    else if ((rightWing == IOI) && (leftWing == IIO))
    {
        moveR32();
    }
    else if ((rightWing == IIO) && (leftWing == OOI))
    {
        moveR32();
    }
    else if ((rightWing == IIO) && (leftWing == IOO))
    {
        moveR32();
    }
    else if ((rightWing == IIO) && (leftWing == IOI))
    {
        moveR32();
    }
    else if ((rightWing == OOI) && (leftWing == IIO))
    {
        moveR32();
    }
}

void rightWallOnlyDynamicCorrection()
{
    motorLeftDirection.write(POSITIVE);
    motorRightDirection.write(POSITIVE);
    if (rightWing == IIO)
    {
        moveRR31();
    }
    else if (rightWing == IOO)
    {
        moveR32();
    }
    else if (rightWing == OII)
    {
        moveLL13();
    }
    else if (rightWing == OOI)
    {
        moveL23();
    }
}

void leftWallOnlyDynamicCorrection()
{
    motorLeftDirection.write(POSITIVE);
    motorRightDirection.write(POSITIVE);
    if (leftWing == IIO)
    {
        moveRR31();
    }
    else if (leftWing == IOO)
    {
        moveR32();
    }
    else if (leftWing == OII)
    {
        moveLL13();
    }
    else if (leftWing == OOI)
    {
        moveL23();
    }
}

void dynamicAdvanceCorrection()
{
    if ((leftWing != OOO) || (rightWing != OOO))
    {
        if (((leftWing == III) && (rightWing == III)) && (IS_WALL_ON_LEFT || IS_WALL_ON_RIGHT))
        {
            mouseState.speed = 0;
            numberOfDances = 4;
            miniDanceCorrection(10);
            mouseState.speed = MOTOR_START_FREQUENCY;
        }
        else if (IS_WALL_ON_LEFT && IS_WALL_ON_RIGHT)
        {
            if ((leftWing == III) && (rightWing != III))
            {
                rightWallOnlyDynamicCorrection();
            }
            else if ((leftWing != III) && (rightWing == III))
            {
                leftWallOnlyDynamicCorrection();
            }
            doubleWallDynamicCorrection();
        }
        else if ((!IS_WALL_ON_LEFT) && (IS_WALL_ON_RIGHT))
        {
            rightWallOnlyDynamicCorrection();
        }
        else if ((IS_WALL_ON_LEFT) && (!IS_WALL_ON_RIGHT))
        {
            leftWallOnlyDynamicCorrection();
        }
    }
}
