#ifndef __GRID_EXPLORE_H_INCLUDED__
#define __GRID_EXPLORE_H_INCLUDED__

#include "contest1.h"

// ==== WALL FOLLOW FORWARD DECLARATIONS ====== 
extern int numStored;
extern float pastDist[50][2];
extern float sizeOfBox; 
extern float xCorr;
extern float yCorr; 
extern bool firstPass;
extern bool inFirstBox; 
extern bool verbose;
extern int numStuck;

// ==== WALL FOLLOW FUNCTION DECLARATIONS ====== 
extern float roundToNearest(float num, float increment);
extern bool updateStep(float arr[][2], int numOfSteps, float boxSize, bool verbose = true);
extern bool beenHere(float arr[][2], int numOfSteps, float sizeOfBox, bool verbose = true);
extern void gridExplore(ros::Publisher vel_pub);

#endif