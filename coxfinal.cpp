#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

void Cox_LineFit(const VectorXd& ANG, const VectorXd& DIS, const Vector3d& POSE, const MatrixXd& LINEMODEL, const Vector3d& SensorPose, double& ddx, double& ddy, double& dda)
{
    // Initialize variables
    ddx = 0; ddy = 0; dda = 0;
    double Rx = POSE(0); double Ry = POSE(1); double Ra = POSE(2); 
    double sALFA = SensorPose(0); double sBETA = SensorPose(1); double sGAMMA = SensorPose(2);
    int max_iterations = 100; // <------YOU NEED TO CHANGE THIS NUMBER
    int no_update = 0;
    
    // Step 0 - Normal vectors (length = 1) to the line segments
    // -> Add your code here
    // Line segments
    MatrixXd orig_lines(2, LINEMODEL.cols());
    orig_lines.row(0) = LINEMODEL.row(2) - LINEMODEL.row(0);
    orig_lines.row(1) = LINEMODEL.row(3) - LINEMODEL.row(1);
    
    // Lines that are perpendicular to the line segments
    MatrixXd perp_lines = orig_lines.rowwise().reverse();
    perp_lines.row(0) *= -1;
    
    MatrixXd ui = perp_lines.array().rowwise() / perp_lines.array().colwise().norm();
    
    MatrixXd Zi(2, LINEMODEL.cols());
    Zi.row(0) = LINEMODEL.row(2);
    Zi.row(1) = LINEMODEL.row(3);
    
    VectorXd Ri = ui.rowwise().dot(Zi).transpose();
    
    // The Loop - REPEAT UNTIL THE PROCESS CONVERGE
    for (int iteration = 1; iteration <= max_iterations; iteration++)
    {
        // Add your code here
        
        // Termination condition
        //if (no_update >= 2)
        //{
        //    break;
        //}

        VectorXd x = DIS.array() * ANG.array().cos();
        VectorXd y = DIS.array() * ANG.array().sin();

// 1.2) Sensor coordinates => Robot coordinates
        MatrixXd R_1(3, 3);
    R_1 << cos(sGAMMA), -sin(sGAMMA), sALFA,
        sin(sGAMMA), cos(sGAMMA), sBETA,
        0, 0, 1;

    MatrixXd Xs = (R_1 * MatrixXd::Ones(1, x.size())).colwise() + (R_1 * Vector3d(x, y, VectorXd::Ones(x.size()))).colwise().homogeneous();

    // 1.3) Robot coordinates => World coordinates
    MatrixXd R_2(3, 3);
    R_2 << cos(Ra + dda), -sin(Ra + dda), Rx + ddx,
        sin(Ra + dda), cos(Ra + dda), Ry + ddy,
        0, 0, 1;

    MatrixXd Xw = (R_2 * Xs.colwise().homogeneous()).topRows(2);
    // Step 2 - Find targets for data points
    MatrixXd vi = Xw.topRows(2);

    VectorXd yi = Ri.array() - (ui * vi).colwise().sum();

    VectorXd::Index min_line_idx;
    yi.array().abs().colwise().minCoeff(&min_line_idx);

    MatrixXd yi_min_abs(yi.size(), 3);
    yi_min_abs.col(0) = VectorXd::LinSpaced(1, yi.size());
    yi_min_abs.col(1) = min_line_idx;
    yi_min_abs.col(2) = yi.array().abs();

    VectorXd yi_min(yi_min_abs.rows());
    for (int k = 0; k < yi_min_abs.rows(); k++) {
        yi_min(k) = yi(yi_min_abs(k, 1));
    }

    double med = yi_min_abs.col(2).median();

    MatrixXd non_out = yi_min_abs.rowwise().partialPivLu().solve(VectorXd::Ones(yi_min_abs.rows())).cwiseProduct(yi_min_abs.col(2).array() <= med).select(yi_min_abs, 0).colwise().head<3>();

    double tmp = 0;
    // Step 3 - Set up linear equation system, find b = (dx, dy, da)' from the LS
    VectorXd vm(2);
    vm << Xw.row(0).col(non_out.col(0)).mean(), Xw.row(1).col(non_out.col(0)).mean();

    VectorXd xi1 = ui(non_out.col(1), 0);
    VectorXd xi2 = ui(non_out.col(1), 1);
    VectorXd xi3(non_out.rows());
    for (int k = 0; k < non_out.rows(); k++) {
        xi3(k) = (ui.row(non_out(k, 1)) * Matrix2d(0, -1, 1, 0) * (vi.topRows(2).col(non_out(k, 0)) - vm)).sum();
    }

    MatrixXd A(non_out.rows(), 3);
    A << xi1, xi2, xi3;

    VectorXd yi_min_ = yi_min.segment(non_out.col(0));
    MatrixXd yi_min_1 = yi_min_.reshaped(non_out.rows(), 1);

    MatrixXd B = (A.transpose() * A).inverse() * A.transpose() * yi_min_1;

    double S2 = ((yi_min_1 - A * B).transpose() * (yi_min_1 - A * B)) / (A.rows() - 4);

    VectorXd b = B;
    MatrixXd C = S2 * (A.transpose() * A).inverse();

    // Print the sizes for debugging
    std::cout << "Size of yi_min_: " << yi_min_.size() << std::endl;
    std::cout << "Size of yi_min_1: " << yi_min_1.size() << std::endl;
    // Step 4 - Add the latest contribution to the overall congruence
    ddx += b(0);
    ddy += b(1);
    dda = fmod(dda + b(2), 2 * M_PI);

    // Step 5 - Check if the process has converged
    if ((sqrt(b(0) * b(0) + b(1) * b(1)) < 5) && (std::abs(b(2)) < 0.1 * M_PI / 180)) {
        break;
    }
    }
}

int main()
{
    // Example usage of Cox_LineFit function
    
    // Create sample inputs
    VectorXd ANG(10);
    ANG << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
    
    VectorXd DIS(10);
    DIS << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0;
    
    Vector3d POSE(1.0, 2.0, 3.0);
    MatrixXd LINEMODEL(4, 7);
    LINEMODEL << 0, 0, 2420, 0, 1213, 0, 1213,
             2420, 0, 2420, 3620, 2420, 1820, 1820,
             2420, 3620, 0, 3620, 1213, 3620, 1213,
             0, 3620, 0, 0, 0, 1820, 1820;
    
    Vector3d SensorPose(0.1, 0.2, 0.3);
    
    double ddx, ddy, dda;
    
    Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose, ddx, ddy, dda);
    
    // Print the results
    std::cout << "ddx: " << ddx << std::endl;
    std::cout << "ddy: " << ddy << std::endl;
    std::cout << "dda: " << dda << std::endl;
    
    return 0;
}// Step 1 - Translate and rotate data points
// 1.1) Relative measurements => Sensor coordinates


    