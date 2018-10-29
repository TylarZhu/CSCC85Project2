/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather information about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#include <stdbool.h>

int redflag = 0;

int map[400][4];            
                            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
int rbt_x, rbt_y, rbt_dir = -1;
double beliefs[400][4];     // Beliefs for each location and motion direction
int rgb[3];
double possibility[8];
int Black[3],Blue[3],Green[3],Yellow[3],Red[3],White[3];
int tl = 0, tr = 0, br = 0, bl = 0;

int turn_choice = 0;
int turn = -1;

int dest_x, dest_y;


#define FILE_NAME "rgb.dat" //save for RGB initial value
#define ROBOT_INIT 0
#define ON_THE_ROAD 1
#define FIND_ROAD 2
#define ADJUST 3
#define FIND_YELLOW 4
#define FIND_RED 5
#define ROBOT_STOP 6
void update_beliefs(int last_act, int intersection_reading[4]);







int main(int argc, char *argv[]) {
    char mapname[1024];
    int rx, ry;
    unsigned char *map_image;


    //read the RGB initail value from rgb.dat
    FILE *fp;
    fp = fopen(FILE_NAME, "rb");
    if (fp == NULL) {
        printf("cann't open %s\n", FILE_NAME);
        exit(EXIT_FAILURE);
    }
    int temp;
    for (int i = 0; i < 3; i++) {
        fscanf(fp, "%i\n", &temp);
        Black[i] = temp;
        fscanf(fp, "%i\n", &temp);
        Blue[i] = temp;
        fscanf(fp, "%i\n", &temp);
        Green[i] = temp;
        fscanf(fp, "%i\n", &temp);
        Yellow[i] = temp;
        fscanf(fp, "%i\n", &temp);
        Red[i] = temp;
        fscanf(fp, "%i\n", &temp);
        White[i] = temp;
    }
    fclose(fp);

    printf("Black  is %i %i %i\n", Black[0], Black[1], Black[2]);
    printf("Blue   is %i %i %i\n", Blue[0], Blue[1], Blue[2]);
    printf("Green  is %i %i %i\n", Green[0], Green[1], Green[2]);
    printf("Yellow is %i %i %i\n", Yellow[0], Yellow[1], Yellow[2]);
    printf("RED    is %i %i %i\n", Red[0], Red[1], Red[2]);
    printf("White  is %i %i %i\n", White[0], White[1], White[2]);


    memset(&map[0][0], 0, 400 * 4 * sizeof(int));
    sx = 0;
    sy = 0;

    if (argc < 4) {
        fprintf(stderr, "Usage: EV3_Localization map_name dest_x dest_y\n");
        fprintf(stderr, "    map_name - should correspond to a properly formatted .ppm map image\n");
        fprintf(stderr,
                "    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
        exit(1);
    }

    strcpy(&mapname[0], argv[1]);
    dest_x = atoi(argv[2]);
    dest_y = atoi(argv[3]);

    /******************************************************************************************************************
    * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
    *   read your calibration data for use in your localization code. Skip this if you are not using calibration
    * ****************************************************************************************************************/


    // Your code for reading any calibration information should not go below this line //

    map_image = readPPMimage(&mapname[0], &rx, &ry);
    if (map_image == NULL) {
        fprintf(stderr, "Unable to open specified map image\n");
        exit(1);
    }

    if (parse_map(map_image, rx, ry) == 0) {
        fprintf(stderr, "Unable to parse input image map. Make sure the image is properly formatted\n");
        free(map_image);
        exit(1);
    }

    /******************************************************************************************************************
    * Bluetooth open, then calibrate sensor.
    * ****************************************************************************************************************/

    // Open a socket to the EV3 for remote controlling the bot.
    if (BT_open(HEXKEY) != 0) {
        fprintf(stderr, "Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
        fprintf(stderr, " hex key for the EV3 matches the one in EV3_Localization.h\n");
        free(map_image);
        exit(1);
    }

    fprintf(stderr, "All set, ready to go!\n");

    if (dest_x == -1 && dest_y == -1) {
        calibrate_sensor();
        BT_close();
        free(map_image);
        exit(1);
    }

    if (dest_x < 0 || dest_x >= sx || dest_y < 0 || dest_y >= sy) {
        fprintf(stderr, "Destination location is outside of the map\n");
        free(map_image);
        exit(1);
    }

    // Initialize beliefs - uniform probability for each location and direction
    for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {
            beliefs[i + (j * sx)][0] = 1.0 / (double) (sx * sy * 4);
            beliefs[i + (j * sx)][1] = 1.0 / (double) (sx * sy * 4);
            beliefs[i + (j * sx)][2] = 1.0 / (double) (sx * sy * 4);
            beliefs[i + (j * sx)][3] = 1.0 / (double) (sx * sy * 4);
        }
    }


    /*******************************************************************************************************************************
    *
    *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
    *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
    *
    *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
    *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
    *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
    *          and 0<=j<=sy-1), index=i+(j*sx)
    *
    *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
    *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
    *
    *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
    *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
    *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
    *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
    *
    *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
    *          belief values based on agreement between what the robot sensed, and the colours in the map.
    *
    *          You have two main tasks these are organized into two major functions:
    *
    *          robot_localization()    <---- Runs the localization loop until the robot's location is found
    *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
    *
    *          The target location, read from the command line, is left in dest_x, dest_y
    *
    *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
    *          that even if your bot managed to find its location, it can become lost again while driving to the target
    *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place -
    *          a very solid implementation should give your robot the ability to determine it's lost and needs to
    *          run localization again.
    *
    *******************************************************************************************************************************/

    // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
    // robot to complete its task should be here.

    //TODO: the color between the road and intersection have some problem need to fix!!!!!!!!!!!
    
    drive_along_street();


    // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
    BT_close();
    free(map_image);
    exit(0);
}

/*!
 * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
 * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
 *
 * @return int, 1 success 0 fail
 */
int find_street(void) {
    bool flag = true;
    printf("find street !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    while(flag) {
        if (Distinguish_Color() == 1 || Distinguish_Color() == 4 || Distinguish_Color() == 5) {
            flag = false;
            drive_along_street();
        } else {
            while(Distinguish_Color() != 1 && Distinguish_Color() != 4 && Distinguish_Color() != 5) {
                BT_motor_port_start(MOTOR_A | MOTOR_B, 5);
            }
            if(Distinguish_Color() == 1) {
                forward_small_2();
                while(Distinguish_Color() != 1) {
                    turn_backwards();
                    turn_left_small();
                    forward_small_2();
                }
            }
        }
    }
    return 1;
}

/*!
 * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
 * the map. It stops at an intersection.
 *
 * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
 * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
 * or the course instructor for help carrying out your plan.
 *
 * @return int, 1 fail 0 success
 */
int drive_along_street(void) {
    printf("ON THE ROAD !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    while (1) {
        // if the robot is on the road, then follow it.
        while (Distinguish_Color() == 1) {
            BT_motor_port_start(MOTOR_A | MOTOR_B, 10);
            // if the robot is on the intersection, then go to FIND YELLOW state.
        }
        for (int i = 0; i <= 90000000; i++);
        BT_all_stop(1);
        int color = double_check();
        if (color == 4) {

            //BT_all_stop(1);
            int scan_comp = 1;
            /*
            while (scan_comp == 1) {

                if (scan_comp == 1) {
                    printf("rescan intersection !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                    //rescan();
                }
            }*/
            scan_comp = scan_intersection();
            //if(tl != 3 || tr != 6 || br != 2 || bl != 6) {
            printf("turn intersection !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            if(turn_choice == 1) {
                turn_choice = 0;
                //turn_at_intersection(0);

            }else if (turn_choice == 0){
                turn_choice = 1;
                //turn_at_intersection(1);
            }
            int sc[4];
            sc[0] = tl;
            sc[1] = tr;
            sc[2] = br;
            sc[3] = bl;
            printf("last turn choice is %i\n",turn);
            update_beliefs(turn,sc);
            robot_localization();
            turn = go_to_target(rbt_x,rbt_y,rbt_dir,dest_x, dest_y);
            if (turn == 1){
                turn_at_intersection(0);

            }else if (turn == 2){
                turn_at_intersection(0);
                turn_at_intersection(0);

            }else if (turn == 3){
                turn_at_intersection(1);
            }

            printf("forward intersection !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            forward_small_1();
            redflag = 0;
            //
            //    forward_small_1();
            //}
            // if the robot hit the wall, then go to FIND RED state.
        } else if (color == 5) {
            //BT_all_stop(1);
            find_red();
            // if the robot is not on the road, the road must be near the robot,
            // then go to ADJUST state.
        } else {
            //BT_all_stop(1);
            adjust();
        }
    }
    BT_all_stop(0);
    printf("arrive intersection !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    return 0;
}

/*!
 *
 * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
 * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
 * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
 * it needs to read, repeatably, and as the robot moves over the map.
 *
 * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and
 * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
 *
 * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
 * in the code below so your TA can quickly understand how it works.
 *
 * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
 * variables:
 *
 * @param tl top left building colour
 * @param tr top right building colour
 * @param br bottom right building colour
 * @param bl bottom left building colour
 *
 * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
 * after this call.
 *
 * @return int 1 fail 0 success
 */
int scan_intersection() {
    //turn_left_angle(45);
    turn_45_degree_both_wheel(1);
    forward_small_1();
    tl = Distinguish_Color();
    backward_small_1();
    backward_small_1();
    br = Distinguish_Color();
    forward_small_1();
    turn_90_degree_both_wheel(0);
    turn_right_small();
    forward_small_1();
    //forward_small_3();
    tr = Distinguish_Color();
    backward_small_1();
    backward_small_1();
    bl = Distinguish_Color();
    forward_small_1();
    printf("tl = %i\n", tl);
    printf("tr = %i\n", tr);
    printf("br = %i\n", br);
    printf("bl = %i\n", bl);
    //forward_small_2();
    //for(int i = 0; i <= 100000000; i ++);
    turn_45_degree_both_wheel(1);
    printf("scan intersection complete!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    if(tl == 1 || tr == 1 || br == 1 || bl == 1) {
        printf("scan intersection fail*************************************************!\n");
        return 1;
    }
    return 0;

}

/*!
 * This function is used to have the robot turn either left or right at an intersection
 * (obviously your bot can not justdrive forward!).
 * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
 * and on a street it can follow.
 * @param turn_direction 0 right, 1 left
 * @return indicate success or failure, or to inform your code of the state of the bot
 */
int turn_at_intersection(int turn_direction) {
    if(turn_direction == 0) {
        turn_90_degree_both_wheel(0);
    } else {
        turn_90_degree_both_wheel(1);
        //turn_left_small();
        //turn_left_small();
    }
    // move the robot out of the intersection.
    /*
    while(Distinguish_Color() == 4) {
        forward_small_2();
    }*/
    return (0);
}

void update_beliefs(int last_act, int intersection_reading[4]){
    double last_beliefs[400][4];
    double C;

    for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {
            last_beliefs[i + (j * sx)][0] = beliefs[i + (j * sx)][0];
            last_beliefs[i + (j * sx)][1] = beliefs[i + (j * sx)][1];
            last_beliefs[i + (j * sx)][2] = beliefs[i + (j * sx)][2];
            last_beliefs[i + (j * sx)][3] = beliefs[i + (j * sx)][3];
        }
    }
    //acting
    if (redflag){// last point read is red
        printf("REDFLAG is on!!!!!!!!!!!!!!!!!!!!!!!!!!");
        for (int j = 0; j < sy; j++) {
            for (int i = 0; i < sx; i++) {

                //acting
                if (last_act == 0) { // going up
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = 0;
                    } else {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + (j * sx)][1] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j - 1) * sx)][1] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j + 1) * sx)][1] * 0.05;
                        }
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = 0;
                    } else {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + (j * sx)][0] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + (j * sx)][0] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + (j * sx)][0] * 0.05;
                        }
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = 0;
                    } else {
                        beliefs[i + (j * sx)][1] = last_beliefs[i + (j * sx)][3] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][3] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j + 1) * sx)][3] * 0.05;
                        }
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = 0;
                    } else {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + (j * sx)][2] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + (j * sx)][2] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + (j * sx)][2] * 0.05;
                        }
                    }
                }
                if (last_act == 1) {// turn right
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = 0;
                    } else {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + (j * sx)][0] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j - 1) * sx)][0] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j + 1) * sx)][0] * 0.05;
                        }
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = 0;
                    } else {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + (j * sx)][3] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + (j * sx)][3] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + (j * sx)][3] * 0.05;
                        }
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = 0;
                    } else {
                        beliefs[i + (j * sx)][1] = last_beliefs[i + (j * sx)][2] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][2] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][2] * 0.05;
                        }
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = 0;
                    } else {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + (j * sx)][1] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + (j * sx)][1] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + (j * sx)][1] * 0.05;
                        }
                    }
                }
                if (last_act == 2) {//turn back
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = 0;
                    } else {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + (j * sx)][3] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j - 1) * sx)][3] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j + 1) * sx)][3] * 0.05;
                        }
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = 0;
                    } else {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + (j * sx)][2] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + (j * sx)][2] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + (j * sx)][2] * 0.05;
                        }
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = 0;
                    } else {
                        beliefs[i + (j * sx)][1] = last_beliefs[i + (j * sx)][1] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][1] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][1] * 0.05;
                        }
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = 0;
                    } else {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + (j * sx)][0] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + (j * sx)][0] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + (j * sx)][0] * 0.05;
                        }
                    }
                }
                if (last_act == 3) {//turn left
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = 0;
                    } else {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + (j * sx)][2] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j - 1) * sx)][2] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][3] += last_beliefs[i + ((j + 1) * sx)][2] * 0.05;
                        }
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = 0;
                    } else {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + (j * sx)][1] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + (j * sx)][1] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + (j * sx)][1] * 0.05;
                        }
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = 0;
                    } else {
                        beliefs[i + (j * sx)][1] = last_beliefs[i + (j * sx)][0] * 0.8;
                        if (j - 1 >= 0) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][0] * 0.05;
                        }
                        if (j + 1 < sy) {
                            beliefs[i + (j * sx)][1] += last_beliefs[i + ((j - 1) * sx)][0] * 0.05;
                        }
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = 0;
                    } else {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + (j * sx)][3] * 0.8;
                        if (i - 1 >= 0) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + (j * sx)][3] * 0.05;
                        }
                        if (i + 1 < sx) {
                            beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + (j * sx)][3] * 0.05;
                        }
                    }
                }

                //stay at the point
                beliefs[i + (j * sx)][0] += last_beliefs[i + (j * sx)][0] * 0.1;
                beliefs[i + (j * sx)][1] += last_beliefs[i + (j * sx)][1] * 0.1;
                beliefs[i + (j * sx)][2] += last_beliefs[i + (j * sx)][2] * 0.1;
                beliefs[i + (j * sx)][3] += last_beliefs[i + (j * sx)][3] * 0.1;

                C = C + beliefs[i + (j * sx)][0]
                    + beliefs[i + (j * sx)][1]
                    + beliefs[i + (j * sx)][2]
                    + beliefs[i + (j * sx)][3];

            }
        }
    }else if (last_act != -1){ //first time move, no act before
        for (int j = 0; j < sy; j++) {
            for (int i = 0; i < sx; i++) {

                //acting
                if (last_act == 0) { // going up
                    //do great job
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + 1 + (j * sx)][3] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][3] = 0;
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + ((j - 1) * sx)][2] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][2] = 0;
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = last_beliefs[i - 1 + (j * sx)][1] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][1] = 0;
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + ((j + 1) * sx)][0] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][0] = 0;
                    }

                    //from left one
                    if (i + 1 < sx && j - 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j - 1) * sx)][3] * 0.05;
                    }
                    if (j - 1 >= 0 && i - 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + ((j - 1) * sx)][2] * 0.05;
                    }
                    if (i - 1 >= 0 && j + 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j + 1) * sx)][1] * 0.05;
                    }
                    if (j + 1 < sy && i + 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + ((j + 1) * sx)][0] * 0.05;
                    }

                    //from right one
                    if (i + 1 < sx && j + 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j + 1) * sx)][3] * 0.05;
                    }
                    if (j - 1 >= 0 && i + 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + ((j - 1) * sx)][2] * 0.05;
                    }
                    if (i - 1 >= 0 && j - 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j - 1) * sx)][1] * 0.05;
                    }
                    if (j + 1 < sy && i - 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + ((j + 1) * sx)][0] * 0.05;
                    }
                }
                if (last_act == 1) {// turn right
                    //do great job
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + 1 + (j * sx)][2] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][3] = 0;
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + ((j - 1) * sx)][1] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][2] = 0;
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = last_beliefs[i - 1 + (j * sx)][0] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][1] = 0;
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + ((j + 1) * sx)][3] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][0] = 0;
                    }

                    //from the left one
                    if (i + 1 < sx && j - 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j - 1) * sx)][2] * 0.05;
                    }
                    if (j - 1 >= 0 && i - 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + ((j - 1) * sx)][1] * 0.05;
                    }
                    if (i - 1 >= 0 && j + 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j + 1) * sx)][0] * 0.05;
                    }
                    if (j + 1 < sy && i + 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + ((j + 1) * sx)][3] * 0.05;
                    }

                    //from the right one
                    if (i + 1 < sx && j + 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j + 1) * sx)][2] * 0.05;
                    }
                    if (j - 1 >= 0 && i + 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + ((j - 1) * sx)][1] * 0.05;
                    }
                    if (i - 1 >= 0 && j - 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j - 1) * sx)][0] * 0.05;
                    }
                    if (j + 1 < sy && i - 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + ((j + 1) * sx)][3] * 0.05;
                    }
                }
                if (last_act == 2) {//turn back
                    //do great job
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + 1 + (j * sx)][1] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][3] = 0;
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + ((j - 1) * sx)][0] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][2] = 0;
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = last_beliefs[i - 1 + (j * sx)][3] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][1] = 0;
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + ((j + 1) * sx)][2] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][0] = 0;
                    }

                    //from the left one
                    if (i + 1 < sx && j - 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j - 1) * sx)][1] * 0.05;
                    }
                    if (j - 1 >= 0 && i - 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + ((j - 1) * sx)][0] * 0.05;
                    }
                    if (i - 1 >= 0 && j + 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j + 1) * sx)][3] * 0.05;
                    }
                    if (j + 1 < sy && i + 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + ((j + 1) * sx)][2] * 0.05;
                    }

                    //from the right one
                    if (i + 1 < sx && j + 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j + 1) * sx)][1] * 0.05;
                    }
                    if (j - 1 >= 0 && i + 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + ((j - 1) * sx)][0] * 0.05;
                    }
                    if (i - 1 >= 0 && j - 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j - 1) * sx)][3] * 0.05;
                    }
                    if (j + 1 < sy && i - 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + ((j + 1) * sx)][2] * 0.05;
                    }
                }
                if (last_act == 3) {//turn left
                    //do great job
                    if (i + 1 < sx) {
                        beliefs[i + (j * sx)][3] = last_beliefs[i + 1 + (j * sx)][0] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][3] = 0;
                    }
                    if (j - 1 >= 0) {
                        beliefs[i + (j * sx)][2] = last_beliefs[i + ((j - 1) * sx)][3] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][2] = 0;
                    }
                    if (i - 1 >= 0) {
                        beliefs[i + (j * sx)][1] = last_beliefs[i - 1 + (j * sx)][2] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][1] = 0;
                    }
                    if (j + 1 < sy) {
                        beliefs[i + (j * sx)][0] = last_beliefs[i + ((j + 1) * sx)][1] * 0.7;
                    } else {
                        beliefs[i + (j * sx)][0] = 0;
                    }
                    //from the left one
                    if (i + 1 < sx && j - 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j - 1) * sx)][0] * 0.05;
                    }
                    if (j - 1 >= 0 && i - 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i - 1 + ((j - 1) * sx)][3] * 0.05;
                    }
                    if (i - 1 >= 0 && j + 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j + 1) * sx)][2] * 0.05;
                    }
                    if (j + 1 < sy && i + 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i + 1 + ((j + 1) * sx)][1] * 0.05;
                    }
                    //from the right one
                    if (i + 1 < sx && j + 1 >= 0) {
                        beliefs[i + (j * sx)][3] += last_beliefs[i + 1 + ((j + 1) * sx)][0] * 0.05;
                    }
                    if (j - 1 >= 0 && i + 1 >= 0) {
                        beliefs[i + (j * sx)][2] += last_beliefs[i + 1 + ((j - 1) * sx)][3] * 0.05;
                    }
                    if (i - 1 >= 0 && j - 1 < sy) {
                        beliefs[i + (j * sx)][1] += last_beliefs[i - 1 + ((j - 1) * sx)][2] * 0.05;
                    }
                    if (j + 1 < sy && i - 1 < sx) {
                        beliefs[i + (j * sx)][0] += last_beliefs[i - 1 + ((j + 1) * sx)][1] * 0.05;
                    }
                }

                //stay at the point
                beliefs[i + (j * sx)][0] += last_beliefs[i + (j * sx)][0] * 0.2;
                beliefs[i + (j * sx)][1] += last_beliefs[i + (j * sx)][1] * 0.2;
                beliefs[i + (j * sx)][2] += last_beliefs[i + (j * sx)][2] * 0.2;
                beliefs[i + (j * sx)][3] += last_beliefs[i + (j * sx)][3] * 0.2;

                C = C + beliefs[i + (j * sx)][0]
                    + beliefs[i + (j * sx)][1]
                    + beliefs[i + (j * sx)][2]
                    + beliefs[i + (j * sx)][3];
            }
        }

        printf("After acting\n");
        for (int j = 0; j < sy; j++) { //normolize and print
            for (int i = 0; i < sx; i++) {
                beliefs[i + (j * sx)][0] = beliefs[i + (j * sx)][0] / C;
                beliefs[i + (j * sx)][1] = beliefs[i + (j * sx)][1] / C;
                beliefs[i + (j * sx)][2] = beliefs[i + (j * sx)][2] / C;
                beliefs[i + (j * sx)][3] = beliefs[i + (j * sx)][3] / C;
                printf("i is %i j is %i :\n",i , j );
                printf("direction is 0 belief is %2f \n", beliefs[i + (j * sx)][0]);
                printf("direction is 1 belief is %2f \n", beliefs[i + (j * sx)][1]);
                printf("direction is 2 belief is %2f \n", beliefs[i + (j * sx)][2]);
                printf("direction is 3 belief is %2f \n", beliefs[i + (j * sx)][3]);
                printf("\n");
            }
        }
    }
    //sensing
    for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {

            //sensing
            if (map[i + (j * sx)][0] == intersection_reading[0] 
                && map[i + (j * sx)][1] == intersection_reading[1]
                && map[i + (j * sx)][2] == intersection_reading[2]
                && map[i + (j * sx)][3] == intersection_reading[3]){
                beliefs[i + (j * sx)][0] *= .7;
            } else {
                beliefs[i + (j * sx)][0] *= .3;
            }

            if (map[i + (j * sx)][1] == intersection_reading[0] 
                && map[i + (j * sx)][2] == intersection_reading[1]
                && map[i + (j * sx)][3] == intersection_reading[2]
                && map[i + (j * sx)][0] == intersection_reading[3]){
                beliefs[i + (j * sx)][1] *= .7;
            } else {
                beliefs[i + (j * sx)][1] *= .3;
            }

            if (map[i + (j * sx)][2] == intersection_reading[0] 
                && map[i + (j * sx)][3] == intersection_reading[1]
                && map[i + (j * sx)][0] == intersection_reading[2]
                && map[i + (j * sx)][1] == intersection_reading[3]){
                beliefs[i + (j * sx)][2] *= .7;
            } else {
                beliefs[i + (j * sx)][2] *= .3;
            }

            if (map[i + (j * sx)][3] == intersection_reading[0] 
                && map[i + (j * sx)][0] == intersection_reading[1]
                && map[i + (j * sx)][1] == intersection_reading[2]
                && map[i + (j * sx)][2] == intersection_reading[3]){
                beliefs[i + (j * sx)][3] *= .7;
            } else {
                beliefs[i + (j * sx)][3] *= .3;
            }
            C = C + beliefs[i + (j * sx)][0] 
                  + beliefs[i + (j * sx)][1]
                  + beliefs[i + (j * sx)][2]
                  + beliefs[i + (j * sx)][3];
        }
    }
    for (int j = 0; j < sy; j++) { //normolize and print
        for (int i = 0; i < sx; i++) {
            beliefs[i + (j * sx)][0] = beliefs[i + (j * sx)][0] / C;
            beliefs[i + (j * sx)][1] = beliefs[i + (j * sx)][1] / C;
            beliefs[i + (j * sx)][2] = beliefs[i + (j * sx)][2] / C;
            beliefs[i + (j * sx)][3] = beliefs[i + (j * sx)][3] / C;
            printf("i is %i j is %i :\n",i , j );
            printf("direction is 0 belief is %2f \n", beliefs[i + (j * sx)][0]);
            printf("direction is 1 belief is %2f \n", beliefs[i + (j * sx)][1]);
            printf("direction is 2 belief is %2f \n", beliefs[i + (j * sx)][2]);
            printf("direction is 3 belief is %2f \n", beliefs[i + (j * sx)][3]);
            printf("\n");
        }
    }
}

int robot_localization() {
    /*  This function implements the main robot localization process. You have to write all code that will control the robot
     *  and get it to carry out the actions required to achieve localization.
     *
     *  Localization process:
     *
     *  - Find the street, and drive along the street toward an intersection
     *  - Scan the colours of buildings around the intersection
     *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
     *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
     *
     *  * We have provided headers for the following functions:
     *
     *  find_street()
     *  drive_along_street()
     *  scan_intersection()
     *  turn_at_intersection()
     *
     *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
     *  provided as a suggestion.
     *
     *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
     *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
     *  other streets than the one your bot was initially placed on.
     *
     *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
     *  it.
     *
     *  In terms of sensor management - the API allows you to read colours either as indexed values or rgb, it's up to
     *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
     *  in order to update beliefs.
     *
     *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
     *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
     *
     *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
     *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
     *
     *  The function receives as input pointers to three integer values, these will be used to store the estimated
     *   robot's location and facing direction. The direction is specified as:
     *   0 - UP
     *   1 - RIGHT
     *   2 - BOTTOM
     *   3 - LEFT
     *
     *  The function's return value is 1 if localization was successful, and 0 otherwise.
     */

    /************************************************************************************************************************
     *   TO DO  -   Complete this function
     ***********************************************************************************************************************/

    // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
    // with your code).
    double max = -1;
    rbt_x, rbt_y, rbt_dir = -1;


    for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {
            if (max < beliefs[i + (j * sx)][0]){
                max = beliefs[i + (j * sx)][0];
                rbt_x = i;
                rbt_y = j;
                rbt_dir = 0;
            }else if (max < beliefs[i + (j * sx)][1]){
                max = beliefs[i + (j * sx)][1];
                rbt_x = i;
                rbt_y = j;
                rbt_dir = 1;
            }else if (max < beliefs[i + (j * sx)][2]){
                max = beliefs[i + (j * sx)][2];
                rbt_x = i;
                rbt_y = j;
                rbt_dir = 2;
            }else if (max < beliefs[i + (j * sx)][3]){
                max = beliefs[i + (j * sx)][3];
                rbt_x = i;
                rbt_y = j;
                rbt_dir = 3;
            }
        }
    }
    printf("Max is %2f rbt_x is %i rbt_y is %i rbt_dir is %i\n", max,rbt_x,rbt_y,rbt_dir);
    for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {
            if ((max - beliefs[i + (j * sx)][0]) < 0.00001  && (rbt_x != i || rbt_y != j || rbt_dir != 0)){
                rbt_x = -1;
                rbt_y = -1;
                rbt_dir = -1;
                return(-1);
            }else if (max == beliefs[i + (j * sx)][1] && (rbt_x != i || rbt_y != j || rbt_dir != 1)){
                rbt_x = -1;
                rbt_y = -1;
                rbt_dir = -1;
                return(-1);
            }else if (max == beliefs[i + (j * sx)][2] && (rbt_x != i || rbt_y != j || rbt_dir != 2)){
                rbt_x = -1;
                rbt_y = -1;
                rbt_dir = -1;
                return(-1);
            }else if (max == beliefs[i + (j * sx)][3] && (rbt_x != i || rbt_y != j || rbt_dir != 3)){
                rbt_x = -1;
                rbt_y = -1;
                rbt_dir = -1;
                return(-1);
            }
        }
    }
    printf("Find the localization: i is %i j is %i direction is %i \n", rbt_x,rbt_y,rbt_dir);
    return (0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y) {
    /*
     * This function is called once localization has been successful, it performs the actions required to take the robot
     * from its current location to the specified target location.
     *
     * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
     * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
     * understand how everything works.
     *
     * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
     * should be able to recover.
     *
     * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
     *          The target's intersection location
     *
     * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
     */

    /************************************************************************************************************************
     *   TO DO  -   Complete this function
     ***********************************************************************************************************************/
    if (robot_x == target_x && robot_y == target_y){
        printf("success!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        exit(0);
    }

    if (robot_x == -1 && robot_y == -1){
        printf("can not localization, keep driving!!!!!!!!!!!!!!!!!!!!!!\n");
        return(0);
    }

    if (robot_x == target_x){
        if (robot_y < target_y){
            if (direction == 0){
                return 2;
            }else if (direction == 1){
                return 1;
            }else if (direction == 2){
                return 0;
            }else{ // directiont ==3
                return 3;
            }
        }else{//robot_y > target_y
            if (direction == 0){
                return 0;
            }else if (direction == 1){
                return 3;
            }else if (direction == 2){
                return 2;
            }else{ // directiont ==3
                return 1;
            }
        }

    }else if (robot_x < target_x){
        if (direction == 0){
            return 1;
        }else if (direction == 1){
            return 0;
        }else if (direction == 2){
            return 3;
        }else{ // directiont ==3
            return 2;
        }
    }else{ //robot_x > target_x
        if (direction == 0){
            return 3;
        }else if (direction == 1){
            return 2;
        }else if (direction == 2){
            return 1;
        }else{ // direction == 3
            return 0;
        }

    }

    return (0);
}

/*!
 * This function is called when the program is started with -1  -1 for the target location.
 *
 * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
 * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
 * level.
 *
 * The principle is - Your code should allow you to sample the different colours in the map, and store representative
 * values that will help you figure out what colours the sensor is reading given the current conditions.
 *
 * Inputs - None
 * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
 *
 * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
 */
void calibrate_sensor(void) {

    /************************************************************************************************************************
     *   OIPTIONAL TO DO  -   Complete this function
     ***********************************************************************************************************************/

    printf("=Select which the colour you want to calibration =\n");
    printf("1 Black  calibration,Please enter: b\n");
    printf("2 Blue   calibration,Please enter: u\n");
    printf("3 Green  calibration,Please enter: g\n");
    printf("4 Yellow calibration,Please enter: y\n");
    printf("5 Red    calibration,Please enter: r\n");
    printf("6 White  calibration,Please enter: w\n");
    printf("Q Exit   calibration,Please enter: q\n");
    printf("=Select which the colour you want to calibration =\n");
    char c = getchar();
    while (c != 'q') {
        getchar();
        switch (c) {
            case 'b':
                printf("1 Black  calibration\n");
                Read_sensor();
                Black[0] = rgb[0];
                Black[1] = rgb[1];
                Black[2] = rgb[2];
                printf("Black_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;
            case 'u':
                printf("2 Blue   calibration\n");
                Read_sensor();
                Blue[0] = rgb[0];
                Blue[1] = rgb[1];
                Blue[2] = rgb[2];
                printf("Blue_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;
            case 'g':
                printf("3 Green  calibration\n");
                Read_sensor();
                Green[0] = rgb[0];
                Green[1] = rgb[1];
                Green[2] = rgb[2];
                printf("Green_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;
            case 'y':
                printf("4 Yellow calibration\n");
                Read_sensor();
                Yellow[0] = rgb[0];
                Yellow[1] = rgb[1];
                Yellow[2] = rgb[2];
                printf("Yellow_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;
            case 'r':
                printf("5 Red    calibration\n");
                Read_sensor();
                Red[0] = rgb[0];
                Red[1] = rgb[1];
                Red[2] = rgb[2];
                printf("Red_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;
            case 'w':
                printf("6 White  calibrationU\n");
                Read_sensor();
                White[0] = rgb[0];
                White[1] = rgb[1];
                White[2] = rgb[2];
                printf("Black_RGB %i %i %i\n", rgb[0], rgb[1], rgb[2]);
                break;

            default:
                printf("Please enter b u g y r w\n");
        }
        printf("=Select which the colour you want to calibration =\n");
        printf("1 Black  calibration,Please enter: b\n");
        printf("2 Blue   calibration,Please enter: u\n");
        printf("3 Green  calibration,Please enter: g\n");
        printf("4 Yellow calibration,Please enter: y\n");
        printf("5 Red    calibration,Please enter: r\n");
        printf("6 White  calibration,Please enter: w\n");
        printf("Q Exit   calibration,Please enter: q\n");
        printf("=================================================\n");
        c = getchar();

    }
    //save the initail value
    FILE *fp;

    fp = fopen(FILE_NAME, "wb");
    if (fp == NULL) {
        printf("cann't open %s\n", FILE_NAME);
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < 3; i++) {
        fprintf(fp, "%i\n", Black[i]);
        fprintf(fp, "%i\n", Blue[i]);
        fprintf(fp, "%i\n", Green[i]);
        fprintf(fp, "%i\n", Yellow[i]);
        fprintf(fp, "%i\n", Red[i]);
        fprintf(fp, "%i\n", White[i]);
    }
    fclose(fp);


    fprintf(stderr, "Calibration function called!\n");
}

int parse_map(unsigned char *map_img, int rx, int ry) {
    /*
      This function takes an input image map array, and two integers that specify the image size.
      It attempts to parse this image into a representation of the map in the image. The size
      and resolution of the map image should not affect the parsing (i.e. you can make your own
      maps without worrying about the exact position of intersections, roads, buildings, etc.).

      However, this function requires:

      * White background for the image  [255 255 255]
      * Red borders around the map  [255 0 0]
      * Black roads  [0 0 0]
      * Yellow intersections  [255 255 0]
      * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
      (any other colour values are ignored - so you can add markings if you like, those
       will not affect parsing)

      The image must be a properly formated .ppm image, see readPPMimage below for details of
      the format. The GIMP image editor saves properly formatted .ppm images, as does the
      imagemagick image processing suite.

      The map representation is read into the map array, with each row in the array corrsponding
      to one intersection, in raster order, that is, for a map with k intersections along its width:

       (row index for the intersection)

       0     1     2    3 ......   k-1

       k    k+1   k+2  ........

       Each row will then contain the colour values for buildings around the intersection
       clockwise from top-left, that is


       top-left               top-right

               intersection

       bottom-left           bottom-right

       So, for the first intersection (at row 0 in the map array)
       map[0][0] <---- colour for the top-left building
       map[0][1] <---- colour for the top-right building
       map[0][2] <---- colour for the bottom-right building
       map[0][3] <---- colour for the bottom-left building

       Color values for map locations are defined as follows (this agrees with what the
       EV3 sensor returns in indexed-colour-reading mode):

       1 -  Black
       2 -  Blue
       3 -  Green
       4 -  Yellow
       5 -  Red
       6 -  White

       If you find a 0, that means you're trying to access an intersection that is not on the
       map! Also note that in practice, because of how the map is defined, you should find
       only Green, Blue, or White around a given intersection.

       The map size (the number of intersections along the horizontal and vertical directions) is
       updated and left in the global variables sx and sy.

       Feel free to create your own maps for testing (you'll have to print them to a reasonable
       size to use with your bot).

    */

    int last3[3];
    int x, y;
    unsigned char R, G, B;
    int ix, iy;
    int bx, by, dx, dy, wx, wy;         // Intersection geometry parameters
    int tgl;
    int idx;

    ix = iy = 0;       // Index to identify the current intersection

    // Determine the spacing and size of intersections in the map
    tgl = 0;
    for (int i = 0; i < rx; i++) {
        for (int j = 0; j < ry; j++) {
            R = *(map_img + ((i + (j * rx)) * 3));
            G = *(map_img + ((i + (j * rx)) * 3) + 1);
            B = *(map_img + ((i + (j * rx)) * 3) + 2);
            if (R == 255 && G == 255 && B == 0) {
                // First intersection, top-left pixel. Scan right to find width and spacing
                bx = i;           // Anchor for intersection locations
                by = j;
                for (int k = i; k < rx; k++)        // Find width and horizontal distance to next intersection
                {
                    R = *(map_img + ((k + (by * rx)) * 3));
                    G = *(map_img + ((k + (by * rx)) * 3) + 1);
                    B = *(map_img + ((k + (by * rx)) * 3) + 2);
                    if (tgl == 0 && (R != 255 || G != 255 || B != 0)) {
                        tgl = 1;
                        wx = k - i;
                    }
                    if (tgl == 1 && R == 255 && G == 255 && B == 0) {
                        tgl = 2;
                        dx = k - i;
                    }
                }
                for (int k = j; k < ry; k++)        // Find height and vertical distance to next intersection
                {
                    R = *(map_img + ((bx + (k * rx)) * 3));
                    G = *(map_img + ((bx + (k * rx)) * 3) + 1);
                    B = *(map_img + ((bx + (k * rx)) * 3) + 2);
                    if (tgl == 2 && (R != 255 || G != 255 || B != 0)) {
                        tgl = 3;
                        wy = k - j;
                    }
                    if (tgl == 3 && R == 255 && G == 255 && B == 0) {
                        tgl = 4;
                        dy = k - j;
                    }
                }

                if (tgl != 4) {
                    fprintf(stderr, "Unable to determine intersection geometry!\n");
                    return (0);
                } else break;
            }
        }
        if (tgl == 4) break;
    }
    fprintf(stderr,
            "Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",
            bx, by, wx, wy, dx, dy);

    sx = 0;
    for (int i = bx + (wx / 2); i < rx; i += dx) {
        R = *(map_img + ((i + (by * rx)) * 3));
        G = *(map_img + ((i + (by * rx)) * 3) + 1);
        B = *(map_img + ((i + (by * rx)) * 3) + 2);
        if (R == 255 && G == 255 && B == 0) sx++;
    }

    sy = 0;
    for (int j = by + (wy / 2); j < ry; j += dy) {
        R = *(map_img + ((bx + (j * rx)) * 3));
        G = *(map_img + ((bx + (j * rx)) * 3) + 1);
        B = *(map_img + ((bx + (j * rx)) * 3) + 2);
        if (R == 255 && G == 255 && B == 0) sy++;
    }

    fprintf(stderr, "Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n", sx, sy);

    // Scan for building colours around each intersection
    idx = 0;
    for (int j = 0; j < sy; j++)
        for (int i = 0; i < sx; i++) {
            x = bx + (i * dx) + (wx / 2);
            y = by + (j * dy) + (wy / 2);

            fprintf(stderr, "Intersection location: %d, %d\n", x, y);
            // Top-left
            x -= wx;
            y -= wy;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0) map[idx][0] = 3;
            else if (R == 0 && G == 0 && B == 255) map[idx][0] = 2;
            else if (R == 255 && G == 255 && B == 255) map[idx][0] = 6;
            else fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Left rgb=%d,%d,%d\n", i, j, R, G, B);

            // Top-right
            x += 2 * wx;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0) map[idx][1] = 3;
            else if (R == 0 && G == 0 && B == 255) map[idx][1] = 2;
            else if (R == 255 && G == 255 && B == 255) map[idx][1] = 6;
            else fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Right rgb=%d,%d,%d\n", i, j, R, G, B);

            // Bottom-right
            y += 2 * wy;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0) map[idx][2] = 3;
            else if (R == 0 && G == 0 && B == 255) map[idx][2] = 2;
            else if (R == 255 && G == 255 && B == 255) map[idx][2] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Right rgb=%d,%d,%d\n", i, j, R, G,
                        B);

            // Bottom-left
            x -= 2 * wx;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0) map[idx][3] = 3;
            else if (R == 0 && G == 0 && B == 255) map[idx][3] = 2;
            else if (R == 255 && G == 255 && B == 255) map[idx][3] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Left rgb=%d,%d,%d\n", i, j, R, G,
                        B);

            fprintf(stderr, "Colours for this intersection: %d, %d, %d, %d\n", map[idx][0], map[idx][1], map[idx][2],
                    map[idx][3]);

            idx++;
        }

    return (1);
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry) {
    // Reads an image from a .ppm file. A .ppm file is a very simple image representation
    // format with a text header followed by the binary rgb data at 24bits per pixel.
    // The header has the following form:
    //
    // P6
    // # One or more comment lines preceded by '#'
    // 340 200
    // 255
    //
    // The first line 'P6' is the .ppm format identifier, this is followed by one or more
    // lines with comments, typically used to inidicate which program generated the
    // .ppm file.
    // After the comments, a line with two integer values specifies the image resolution
    // as number of pixels in x and number of pixels in y.
    // The final line of the header stores the maximum value for pixels in the image,
    // usually 255.
    // After this last header line, binary data stores the rgb values for each pixel
    // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
    //
    // NOTE: Windows file handling is rather crotchetty. You may have to change the
    //       way this file is accessed if the images are being corrupted on read
    //       on Windows.
    //

    FILE *f;
    unsigned char *im;
    char line[1024];
    int i;
    unsigned char *tmp;
    double *frgb;

    im = NULL;
    f = fopen(filename, "rb+");
    if (f == NULL) {
        fprintf(stderr, "Unable to open file %s for reading, please check name and path\n", filename);
        return (NULL);
    }
    fgets(&line[0], 1000, f);
    if (strcmp(&line[0], "P6\n") != 0) {
        fprintf(stderr, "Wrong file format, not a .ppm file or header end-of-line characters missing\n");
        fclose(f);
        return (NULL);
    }
    fprintf(stderr, "%s\n", line);
    // Skip over comments
    fgets(&line[0], 511, f);
    while (line[0] == '#') {
        fprintf(stderr, "%s", line);
        fgets(&line[0], 511, f);
    }
    sscanf(&line[0], "%d %d\n", rx, ry);                  // Read image size
    fprintf(stderr, "nx=%d, ny=%d\n\n", *rx, *ry);

    fgets(&line[0], 9, f);                    // Read the remaining header line
    fprintf(stderr, "%s\n", line);
    im = (unsigned char *) calloc((*rx) * (*ry) * 3, sizeof(unsigned char));
    if (im == NULL) {
        fprintf(stderr, "Out of memory allocating space for image\n");
        fclose(f);
        return (NULL);
    }
    fread(im, (*rx) * (*ry) * 3 * sizeof(unsigned char), 1, f);
    fclose(f);

    return (im);
}

/************************************************************************************************************************
 *   HELPER FUNCTION SECTION
 ***********************************************************************************************************************/

/*!
 * turn a side 90 degree
 * @param side 1 left 0 right
 */
void turn_45_degree_both_wheel(int side) {
    if(side == 1) {
        BT_timed_motor_port_start(MOTOR_B, 20, 80, 500, 80);
        BT_timed_motor_port_start(MOTOR_A, -20, 60, 600, 60);
    } else {
        BT_timed_motor_port_start(MOTOR_A, 21, 60, 600, 60);
        BT_timed_motor_port_start(MOTOR_B, -20, 80, 500, 80);
    }
    for(int i = 0; i <= 1000000000; i ++);
}

/*!
 * turn a side 90 degree
 * @param side 1 left 0 right
 */
void turn_90_degree_both_wheel(int side) {
    if(side == 1) {//turn left
        BT_timed_motor_port_start(MOTOR_B, 18, 60, 1000, 60);
        BT_timed_motor_port_start(MOTOR_A, -18, 60, 1000, 60);

    } else {
        BT_timed_motor_port_start(MOTOR_A, 21, 60, 1000, 60);
        BT_timed_motor_port_start(MOTOR_B, -20, 60, 1000, 60);
    }
    for(int i = 0; i <= 2000000000; i ++);
}

/*!
 * turn a side 180 degree
 */
void turn_180_degree_both_wheel(void) {
    BT_timed_motor_port_start(MOTOR_B, 20, 60, 2200, 60);
    BT_timed_motor_port_start(MOTOR_A, -20, 60, 2200, 60);
    for(int i = 0; i <= 1000000000; i ++);
}

/*!
 * This function read the sensor and return the most likely color
 * and calculate the possibility of others
 * @return index of that color.
 */
int Distinguish_Color(void) {
    double possibility[7];


    BT_read_colour_sensor_RGB(PORT_1, rgb);
    printf("sensor value: R %i B %i G %i \n", rgb[0], rgb[1], rgb[2]);
    double colour_error[7];

    int Black_Error = pow(Black[0] - rgb[0], 2) + pow(Black[1] - rgb[1], 2) + pow(Black[2] - rgb[2], 2);
    int Blue_Error = pow(Blue[0] - rgb[0], 2) + pow(Blue[1] - rgb[1], 2) + pow(Blue[2] - rgb[2], 2);
    int Green_Error = pow(Green[0] - rgb[0], 2) + pow(Green[1] - rgb[1], 2) + pow(Green[2] - rgb[2], 2);
    int Yellow_Error = pow(Yellow[0] - rgb[0], 2) + pow(Yellow[1] - rgb[1], 2) + pow(Yellow[2] - rgb[2], 2);
    int Red_Error = pow(Red[0] - rgb[0], 2) + pow(Red[1] - rgb[1], 2) + pow(Red[2] - rgb[2], 2);
    int White_Error = pow(White[0] - rgb[0], 2) + pow(White[1] - rgb[1], 2) + pow(White[2] - rgb[2], 2);
    double Sum_Of_Square_Error = sqrt((double)Black_Error) +
                                 sqrt((double)Blue_Error) +
                                 sqrt((double)Green_Error) +
                                 sqrt((double)Yellow_Error) +
                                 sqrt((double)Red_Error) +
                                 sqrt((double)White_Error);


    possibility[1] = (Sum_Of_Square_Error - sqrt((double) Black_Error)) / Sum_Of_Square_Error;
    possibility[2] = (Sum_Of_Square_Error - sqrt((double) Blue_Error)) / Sum_Of_Square_Error;
    possibility[3] = (Sum_Of_Square_Error - sqrt((double) Green_Error)) / Sum_Of_Square_Error;
    possibility[4] = (Sum_Of_Square_Error - sqrt((double) Yellow_Error)) / Sum_Of_Square_Error;
    possibility[5] = (Sum_Of_Square_Error - sqrt((double) Red_Error)) / Sum_Of_Square_Error;
    possibility[6] = (Sum_Of_Square_Error - sqrt((double) White_Error)) / Sum_Of_Square_Error;

    double max=-1;
    int colour_value=1;
    for (int i = 1; i < 7; i++) {
        if (max <= possibility[i] ) {
            max = possibility[i];
            colour_value = i;
        }
    }
    printf("colour value: %i\n", colour_value);
    return colour_value;


}

void turn_left_small(void) {
    BT_timed_motor_port_start(MOTOR_B, 30, 60, 80, 60);
    BT_timed_motor_port_start(MOTOR_A, -30, 60, 80, 60);
    for(int i = 0; i <= 1000000000; i ++);
}

void turn_right_small(void) {
    BT_timed_motor_port_start(MOTOR_A, 30, 60, 80, 60);
    BT_timed_motor_port_start(MOTOR_B, -30, 60, 80, 60);
    for(int i = 0; i <= 1000000000; i ++);
}

/*!
 * find road turn the robot upright after find road left
 */
void turn_upright(int side) {
    if(side == 1) {
        BT_timed_motor_port_start(MOTOR_A, 30, 80, 600, 80);
        BT_timed_motor_port_start(MOTOR_B, -30, 80, 600, 80);
    } else {
        BT_timed_motor_port_start(MOTOR_B, 30, 80, 600, 80);
        BT_timed_motor_port_start(MOTOR_A, -30, 80, 600, 80);
    }

}

void turn_backwards(void) {
    bool flag = true;
    while(flag) {
        BT_motor_port_start(MOTOR_A | MOTOR_B, -5);
        if(Distinguish_Color() == 1) {
            printf("delay !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            //for(int i = 0; i <= 500000; i ++);
            flag = false;
        }
    }
    for(int i = 0; i <= 500000000; i ++);
}

/*!
 * get the average of the color
 */
void Read_sensor(void) {
    int sum[]={0,0,0};
    for(int i=0;i<10;i++){
        BT_read_colour_sensor_RGB(PORT_1, rgb);
        printf("sensor value: R %i B %i G %i \n", rgb[0], rgb[1], rgb[2]);
        sum[0]=sum[0]+rgb[0];
        sum[1]=sum[1]+rgb[1];
        sum[2]=sum[2]+rgb[2];
    }
    printf("read color... \n");
    //avrage the rgb
    rgb[0] = (int)(sum[0]/10);
    rgb[1] = (int)(sum[1]/10);
    rgb[2] = (int)(sum[2]/10);

}
/*
void command(void){
    bool flag = true;
    while(flag) {
        printf("robot init !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        // go across the map until find the road, hit a intersection or wall
        while (Distinguish_Color() != 1 && Distinguish_Color() != 4 && Distinguish_Color() != 5) {
            BT_drive(MOTOR_A, MOTOR_B, 10);
        }
        // if the robot hit the wall, go to FIND RED state.
        if (Distinguish_Color() == 5) {
            BT_all_stop(1);
            find_red();
            flag = false;
            // if the robot find the intersection, then go to find FIND YELLOW state.
        } else if (Distinguish_Color() == 4) {
            //printf("delay !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            //for(int i = 0; i <= 100000000; i ++);
            BT_all_stop(1);
            turn_at_intersection(0);
            // assume the robot is placed anywhere on the map
            // go to FIND ROAD state.
        } else {
            BT_all_stop(1);
            drive_along_street();
        }
    }
    printf("command over !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}*/

void find_red(void) {
    printf("find RED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    redflag = 1;
    turn_180_degree_both_wheel();
    printf("delay ***************************************************************************\n");
    for(int i = 0; i <= 1000000000; i ++);
    while(Distinguish_Color() == 5) {
        forward_small_1();
    }
}

void adjust(void) {
    int left_num = 0, right_num = 0, turn_limit = 1, last_turn = 0;
    bool flag = true;
    printf("ADJUST !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    while(Distinguish_Color() != 1 && Distinguish_Color() != 4 && flag) {
        turn_backwards();
        printf("ADJUST !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        if(left_num < turn_limit) {
            printf("TURN LEFT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            turn_left_small();
            left_num += 1;
            last_turn = 1;
        } else if(right_num < 2 * turn_limit){
            printf("TURN RIGHT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            turn_right_small();
            right_num += 1;
            last_turn = 0;
        }
        forward_small_2();
        // if the robot does not find street and intersection, then continue scanning.
        if(Distinguish_Color() != 1 &&
           Distinguish_Color() != 4 &&
           Distinguish_Color() != 5) {
            // finish scan left and right but still does not find the road.
            if (left_num >= turn_limit && right_num >= 2 * turn_limit) {
                left_num = 0;
                right_num = 0;
                turn_limit += 1;
            }
        } else {
            backward_small_2();
            if(last_turn == 1) {
                turn_left_small();
            } else {
                turn_right_small();
            }
            forward_small_2();
            flag = false;
        }
    }
}
/*
int double_check(void) {
    printf("double check !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    for(int i = 0; i <= 1000000000; i ++);
    forward_small_3();
    return Distinguish_Color();

}*/

void forward_small_3(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, 15, 80, 80, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void forward_small_2(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, 20, 80, 200, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void forward_small_1(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, 25, 80, 400, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void backward_small_3(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, -15, 80, 80, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void backward_small_2(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, -20, 80, 200, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void backward_small_1(void) {
    BT_timed_motor_port_start(MOTOR_A | MOTOR_B, -25, 80, 400, 80);
    for(int i = 0; i <= 1000000000; i ++);
}

void rescan(void) {
    while(Distinguish_Color() == 4) {
        forward_small_2();
    }
    turn_right_small();
    while(Distinguish_Color() != 4) {
        BT_motor_port_start(MOTOR_A | MOTOR_B, -10);
    }
}

int double_check(void) {
    forward_small_3();
    return Distinguish_Color();
}
/*
int get_true_angle(void) {
    if (0 < angle && angle < 63) {
        angle_jump = 0;
    }

    for (int i = 0; i < 15; i++) {
        if ((92 + 256 * i) < angle && angle < (292 + 256 * i)) {
            angle_jump = i + 1;
        }

        if ((-420 - 256 * i) < angle && angle < (-220 - 256 * i)) {
            angle_jump = -i - 1;
        }
    }
    printf("angle_jump=%i\n", angle_jump);

    if (-128 <= angle && angle <= -1) {
        real_angle = angle + 256 * angle_jump;
    } else {
        real_angle = angle;
    }
    return real_angle;
}

void turn_left_angle(int angle) {
    //left angle
    int start_angle = get_true_angle();
    BT_turn(MOTOR_A, -20, MOTOR_B, 20);
    for(int i = 0; i <= 1000000000; i ++);
    //while (fabs((float) (get_true_angle() - start_angle)) < angle - 6);
    printf("stop....\n");
    BT_all_stop(1);
    for(int i = 0; i <= 1000000000; i ++);
}

void turn_right_angle(int angle) {
    //left angle
    int start_angle = get_true_angle();
    BT_turn(MOTOR_A, 22, MOTOR_B, -19);
    //while (fabs((float) (get_true_angle() - start_angle)) < angle - 6);
    printf("stop....\n");
    BT_all_stop(1);
    for(int i = 0; i <= 1000000000; i ++);
}
*/