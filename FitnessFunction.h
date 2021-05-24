#pragma once
#ifndef _FUC
#define _FUC


#include "PSO.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <io.h>
#include<stdio.h>
#define PI 3.1415926

using namespace std;


class fitness
{
public:
	void setnum(float* location , float cam_angle);
	float* read_angle();
	unsigned long int  NumOfMap;

protected:
	float position [3];
	float angle;
	float mapangle[60000];
	float  mappoints[30000][3];

};
#endif


