#ifndef LIDAR_SERVER
#define LIDAR_SERVER


#include <eigen/Eigen/Dense>

void *Lidar_Server(void *);

extern Eigen::MatrixXd dis;
extern Eigen::MatrixXd ang;

extern int Array_full_flag;

#endif