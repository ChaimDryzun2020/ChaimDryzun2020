// Constants.h
// Contains general declarations and definitions applied for all the files 
//
// Contains general header files for Eigen, PCL,  and windows
// Contains conversion factor definitions
// Contains constants for calculations. 
//
// First created by Chaim Dryzun at 11.04.2021

#pragma once

// General windows header files
#include <limits>
#include <ctime>
#include <memory>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen header file
#include <Eigen/Dense>

// Constant and Conversion factor definitions

// Angle conversion factor definitions
#define DEG_TO_RAD 0.01745329251994329576923690768489
#define RAD_TO_DEG 57.295779513082320876798154814105

// Distance constants and Conversion factor definitions
#define DISTANCE_FACTOR 1000.0
#define M_TO_MM 1000.0
#define MM_TO_M 0.001
#define M_TO_CM 100.0
#define CM_TO_M 0.01

// PI definitions
#define PI 3.1415926535897932384626433832795
#define TWOPI 6.283185307179586476925286766559
#define TAU 6.283185307179586476925286766559
#define PHI 1.6180339887498948482045868343656


// Zero definition
#define ZEPS 0.0000000001
#define E_MINUS_NINE 0.000000001
#define POINTS_SENSITIVITY 0.000001

// Infinity definition
#define INFTY 10000000000.0

// Error float value
#define FLOAT_ERROR -1.0
