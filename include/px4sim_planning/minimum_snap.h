#ifndef MINIMUM_SNAP_H
#define MINIMUM_SNAP_H

#include <cmath>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;

class MinimumSnap {
public:
    VectorXd axis_weight;
    float cost;
    MinimumSnap(unsigned int poly_order, double avg_vel, unsigned int continuity_order);
    MinimumSnap() = delete;
    ~MinimumSnap() = default;
    VectorXd AllocateTime(const MatrixXd &waypoints) const;
    MatrixXd SolveQPClosedForm(const MatrixXd &waypoints, const VectorXd &time_allocation,
                            const MatrixXd &front_constraints, const MatrixXd &end_constraints);
    MatrixXd SolveQPClosedForm(const MatrixXd &waypoints, const VectorXd &time_allocation);
    bool IsPolyGenerated() const;
    unsigned int GetPolyOrder() const;

private:
    unsigned int poly_order_;
    double avg_vel_;
    unsigned int ctn_order_; // continuity order
    bool check_eq(int a, int b) const;
    bool success;
    MatrixXd form_constraints_mat(MatrixXd input_consts, int rows, int dim);
    MatrixXd form_constraints_mat(VectorXd input_const, int rows, int dim);
    MatrixXd form_cost_mat(const VectorXd &time_allocation, int poly_order, int deriv_order);
    RowVectorXd poly_derivative(double t, unsigned int der_order, unsigned int poly_order);
    MatrixXd multi_poly_derivative(double t, unsigned int min_der_order, unsigned int max_der_order, unsigned int poly_order);
};

#endif