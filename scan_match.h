#ifndef _SCAN_MATCH_H
#define _SCAN_MATCH_H

#include <eigen/Eigen/Dense>

extern int cox_complete;

extern int x_cox;
extern int y_cox;
extern int a_cox;
extern Eigen::MatrixXd c_cox;

extern double ddx;
extern double ddy;
extern double dda;

extern void *Scan_Match(void *);

#endif

