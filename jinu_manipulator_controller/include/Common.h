#ifndef COMMON_H
#define COMMON_H

#include <unistd.h>
#include <math.h>

#include "IndexNotation.h"
#include "rmd_motor.h"
#include "rmd_can.h"

#include "Eigen/Dense"
#include "Eigen/Core"

using namespace Eigen;
using namespace std;

#define PI          3.141592

#define MAX_MC      12
#define MAX_JOINT   12
#define MAX_LEG      4

//#define RT_MS       1

#endif // COMMON_H
