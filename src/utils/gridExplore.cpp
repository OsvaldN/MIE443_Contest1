#include "gridExplore.h"

int numStored = 50; //number of past steps to store (will only store unique areas)

float pastDist[50][2] = { 0.0 }; //create an empty array. [i][0] is x direction and [i][1] is y direction (arbitrary definition) 
float sizeOfBox = 3; //number of individual steps per side of the box

//initialize array to current location (assumes the initial spin covers a good amount of this box)
float xCorr = roundToNearest(posX,sizeOfBox);
float yCorr = roundToNearest(posY,sizeOfBox);

bool firstPass = true; //runs the first SpinAndStep 
bool inFirstBox = false; //can be used for other code to be run at the beginning
bool verbose = true;
int numStuck = 0;

float roundToNearest(float num, float increment){
    float numTimes, roundedNum;
    numTimes = num/increment;
    numTimes = roundf(numTimes);
    roundedNum = numTimes * increment;

    return roundedNum;
}

bool updateStep(float arr[][2], int numOfSteps, float boxSize, bool verbose){
    //updates the location tracking array if it is in a new area
    // returns true if it has moved to a new place, false if not
    float xCorr = roundToNearest(posX,boxSize);
    float yCorr = roundToNearest(posY,boxSize);


    if (fabs(xCorr - arr[0][0]) > boxSize/3 && fabs(yCorr - arr[0][1])> boxSize/3){ //check if the position is different than before
        // fabs to get abs value of float, should always be off by ~boxSize
        // if true, probably new place

        // update the array
        int n = numOfSteps;
        for (int i=n-1;i>=0;i--){ //move array over, and have new position in the 0'th column
            if(i>0){
                arr[i][0] = arr[i-1][0];
                arr[i][1] = arr[i-1][1];
            } else {
                arr[0][0] = xCorr;
                arr[0][1] = yCorr;
            }
        }


        if (verbose){
            ROS_INFO("Moved from location (%f,%f) to (%f,%f).",arr[1][0],arr[1][1],arr[0][0],arr[0][1]);
            ROS_INFO("Print last 10 locations:");
            for(int i=0;i<10;i++){
                ROS_INFO("(%f, %f)",arr[i][0], arr[i][1]);
            }
        }

        return true;
    } else {
        if (verbose){
            ROS_INFO("Still at location (%f,%f)", arr[0][0],arr[0][1]);
        }
    }

    return false;
}

bool beenHere(float arr[][2], int numOfSteps, float sizeOfBox, bool verbose){
    // checks if this location has been visited any time other than now
    float xCorr, yCorr; // make the position at the center of any box
    xCorr = roundToNearest(posX,sizeOfBox);
    yCorr = roundToNearest(posY,sizeOfBox);

    for (int i = 1; i< numOfSteps;i++){ //do not want to check current box
        
        if ((fabs(xCorr-arr[i][0]) < sizeOfBox/3) && (fabs (yCorr-arr[i][1]) < sizeOfBox/3)){
            // found a similar location in the saved data
            if (verbose){
                ROS_INFO("Already been to location (%f,%f).",arr[i][0],arr[i][1]);
            }
            return true;
        }
    }
    if (verbose){
        ROS_INFO("Not been to location (%f,%f) before.",xCorr,yCorr);
    }
    return false;
}

void gridExplore(ros::Publisher vel_pub) {
     if (firstPass){
            spinAndStep(1, SPEED_LIM, 5, &vel_pub);
            firstPass = false;
            if (verbose){
                ROS_INFO("Did first spin.");
            }
        } else if (inFirstBox) { //i am not sure if this is an edge case
            wallFollow(&vel_pub); 
            if ((fabs(roundToNearest(posX,sizeOfBox)-xCorr) < sizeOfBox/3) && (fabs (roundToNearest(posY,sizeOfBox)-yCorr) < sizeOfBox/3)){
                // no longer in the first box and can begin logic
                inFirstBox = false;
            }
        } else {
            if (!beenHere(pastDist,numStored,sizeOfBox)){
                wallFollow(&vel_pub);
                numStuck = 0;
                if (verbose){
                    ROS_INFO("Continue with wall follower. At a new location.");
                }
            } else {
                numStuck++; 
                if (numStuck < 15){//to let the wall follow continue a bit to see if it can unstuck itself
                    wallFollow(&vel_pub);
                    if (verbose){
                        ROS_INFO("In an explored location, but trying wall follow.");
                    }
                } else if (numStuck<100){
                    spinToDist(2, SPEED_LIM, 1, 1, 30, &vel_pub);
                    if (verbose){
                        ROS_INFO("In an explored location, but trying to spin to dist.");
                    }
                } else{ //very stuck or has been everywhere
                    if (verbose){
                        ROS_INFO("In an explored location, trying everything.");
                    }
                    if (numStuck % 100 ==0){
                        spinAndStep(1, SPEED_LIM, 5, &vel_pub);
                    } else if (numStuck % 100 < 40){
                        wallFollow(&vel_pub);
                    } else {
                        spinToDist(2, SPEED_LIM, 1, 1, 30, &vel_pub);
                    }
                }
            }
        }

        // ensure data is updated
        updateStep(pastDist,numStored,sizeOfBox);
}