#include "px4sim_planning/minimum_snap.h"

MinimumSnap::MinimumSnap(unsigned int poly_order, double avg_vel, unsigned int continuity_order):
    poly_order_(poly_order), avg_vel_(avg_vel), ctn_order_(continuity_order){
    if(poly_order < 5){
        ROS_INFO("Warning: Polynomial order less than 5 will result in discontinuity of acceleration!");
    }
    if(continuity_order > int((poly_order-1)/2)){
        ROS_INFO("Warning: inappropriate continuity order!");
        ctn_order_ = int((poly_order-1)/2);
    }
    success = false;
    cost = 0;
}

bool MinimumSnap::check_eq(int a, int b) const {
    return (a == b);
}

MatrixXd MinimumSnap::form_constraints_mat(MatrixXd input_consts, int rows, int dim){
    MatrixXd consts = MatrixXd::Zero(rows, dim);
    if(input_consts.rows() < rows){
        consts.topRows(input_consts.rows()) = input_consts;
    }
    else if(input_consts.rows() > rows){
        consts = input_consts.topRows(rows);
    }
    else{
        consts = input_consts;
    }
    return consts;
}

MatrixXd MinimumSnap::form_constraints_mat(VectorXd input_const, int rows, int dim){
    MatrixXd consts = MatrixXd::Zero(rows, dim);
    consts.row(0) = input_const;
    return consts;
}

MatrixXd MinimumSnap::form_cost_mat(const VectorXd &time_allocation, int poly_order, int deriv_order=4){
    const int n_seg = time_allocation.size();
    MatrixXd Q = MatrixXd::Zero((poly_order+1)*n_seg, (poly_order+1)*n_seg);
    RowVectorXd tmp_vec = RowVectorXd::LinSpaced(deriv_order, 0.0, deriv_order-1.0);
    for(int n=0; n<n_seg; n++){
        for(int i=0; i<poly_order+1; i++){
            for(int j=0; j<poly_order+1; j++){
                if(i >= deriv_order && j >= deriv_order){
                    int pow_ = i + j - 2*deriv_order + 1;
                    RowVectorXd coef_mat = (i*RowVectorXd::Ones(tmp_vec.size()) - tmp_vec) * 
                        (j*RowVectorXd::Ones(tmp_vec.size()) - tmp_vec).adjoint();
                    Q((poly_order+1)*n+i,(poly_order+1)*n+j) = coef_mat.prod() * pow(time_allocation(n),pow_) / pow_;
                }
            }
        }
    }
    return Q;
}

RowVectorXd MinimumSnap::poly_derivative(double t, unsigned int der_order, unsigned int poly_order){
    RowVectorXd result = RowVectorXd::Zero(poly_order+1u);
    if(der_order > poly_order){
        return result;
    }
    if(t == 0){
        unsigned int fact = 1;
        for(int i=1; i<= der_order; i++){
            fact *= i;
        }
        result(der_order) = fact;
        return result;
    }
    for(int i=der_order; i<poly_order+1; i++){
        result(i) = pow(t, i-der_order);
        for(int j=i; j>i-der_order; j--){
            result(i) *= j;
        }
    }
    return result;
}

MatrixXd MinimumSnap::multi_poly_derivative(double t, unsigned int min_der_order, unsigned int max_der_order, unsigned int poly_order){
    if(max_der_order < min_der_order){
        return MatrixXd(max_der_order-min_der_order+1u, poly_order+1u);
    }
    MatrixXd result = MatrixXd::Zero(max_der_order-min_der_order+1u, poly_order+1u);
    for(unsigned int i=min_der_order; i<=max_der_order; i++){
        result.row(i-min_der_order) = poly_derivative(t, i, poly_order);
    }
    return result;
}

VectorXd MinimumSnap::AllocateTime(const MatrixXd &waypoints) const {
    if(waypoints.rows() <= 1){
        ROS_INFO("Invalid Waypoints Dimension!");
        exit(-1);
    }
    VectorXd times = VectorXd::Zero(waypoints.rows() - 1);
    double segment_t;
    for (unsigned int i = 1; i < waypoints.rows(); ++i) {
        double delta_dist = (waypoints.row(i) - waypoints.row(i - 1)).norm();
        segment_t = delta_dist / avg_vel_;
        times[i - 1] = segment_t;
    }
    return times;
}

MatrixXd MinimumSnap::SolveQPClosedForm(const MatrixXd &waypoints, const VectorXd &time_allocation,
                                        const MatrixXd &front_constraints, const MatrixXd &end_constraints){
    if(!check_eq(waypoints.cols(), front_constraints.cols()) ||
        !check_eq(waypoints.cols(), end_constraints.cols()) ||
        !check_eq(waypoints.rows(), time_allocation.size() + 1u)){
        ROS_INFO("Invalid Input Dimension!");
        success = false;
        return MatrixXd(1,1);
    }

    const unsigned int dim = waypoints.cols();
    unsigned int start_constraint_num = 0;
    unsigned int end_constraint_num = 0;
    if(poly_order_%2 != 0){
        start_constraint_num = int((poly_order_+1)/2);
        end_constraint_num = int((poly_order_+1)/2);
    }
    else{
        start_constraint_num = int(poly_order_/2+1);
        end_constraint_num = int(poly_order_/2);
    }
    MatrixXd front_deriv_consts = form_constraints_mat(front_constraints, start_constraint_num-1, dim);
    MatrixXd end_deriv_consts = form_constraints_mat(end_constraints, end_constraint_num-1, dim);

    const unsigned int num_poly_coeff = poly_order_ + 1;
    const unsigned int num_segments = time_allocation.size();
    const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;
    const unsigned int known_consts_num = start_constraint_num + end_constraint_num + (ctn_order_+2)*(num_segments-1);
    const unsigned int free_vars_num = num_segments*num_poly_coeff - known_consts_num;

    MatrixXd A_cons = MatrixXd::Zero(known_consts_num, num_segments*num_poly_coeff);
    MatrixXd B_cons = MatrixXd::Zero(known_consts_num, dim);
    MatrixXd A_free = MatrixXd::Zero(free_vars_num, num_segments*num_poly_coeff);

    // start and end constraints
    A_cons.block(0,0,start_constraint_num,num_poly_coeff) = multi_poly_derivative(0.0, 0, start_constraint_num-1, poly_order_);
    B_cons.row(0) = waypoints.row(0);
    B_cons.block(1,0,start_constraint_num-1,dim) = front_deriv_consts;
    A_cons.block(start_constraint_num,(num_segments-1)*num_poly_coeff,end_constraint_num,num_poly_coeff) = 
        multi_poly_derivative(time_allocation(num_segments-1), 0, end_constraint_num-1, poly_order_);
    B_cons.row(start_constraint_num) = waypoints.row(num_segments);
    B_cons.block(start_constraint_num+1,0,end_constraint_num-1,dim) = end_deriv_consts;

    // midpoint constraints
    const unsigned int tmp_row1 = start_constraint_num + end_constraint_num;
    for(unsigned int i=0; i<num_segments-1; i++){
        A_cons.block(tmp_row1+i,num_poly_coeff*i,1,num_poly_coeff) = 
            poly_derivative(time_allocation(i), 0, poly_order_);
        B_cons.row(tmp_row1+i) = waypoints.row(1+i);
    }

    // continuity constraints
    const unsigned int tmp_row2 = tmp_row1 + num_segments - 1;
    for(unsigned int i=0; i<num_segments-1; i++){
        A_cons.block(tmp_row2+i*(ctn_order_+1),i*num_poly_coeff,ctn_order_+1,num_poly_coeff) = 
            multi_poly_derivative(time_allocation(i), 0, ctn_order_, poly_order_);
        A_cons.block(tmp_row2+i*(ctn_order_+1),(i+1)*num_poly_coeff,ctn_order_+1,num_poly_coeff) = 
            - multi_poly_derivative(0.0, 0, ctn_order_, poly_order_);
    }

    // t=T_i free variables
    for(unsigned int i=0; i<num_segments-1; i++){
        A_free.block(i*(end_constraint_num-1),i*num_poly_coeff,end_constraint_num-1,num_poly_coeff) = 
            multi_poly_derivative(time_allocation(i), 1, end_constraint_num-1, poly_order_);
    }

    // t=0 free variables
    const unsigned int tmp_row3 = (num_segments-1) * (end_constraint_num - 1);
    if(ctn_order_ < start_constraint_num-1){
        unsigned int drow = start_constraint_num - ctn_order_ - 1;
        for(unsigned int i=0; i<num_segments-1; i++){
            A_free.block(tmp_row3+i*drow,(i+1)*num_poly_coeff,drow,num_poly_coeff) = 
                multi_poly_derivative(0.0, ctn_order_+1, start_constraint_num-1, poly_order_);
        }
    }

    // QP formulation
    axis_weight = VectorXd::Ones(dim);
    MatrixXd cost_Q = form_cost_mat(time_allocation, poly_order_);
    MatrixXd optim_coeffs = MatrixXd::Zero(num_segments*num_poly_coeff, dim);

    if(free_vars_num > 0){
        MatrixXd A = MatrixXd::Zero(num_segments*num_poly_coeff, num_segments*num_poly_coeff);
        A.topRows(known_consts_num) = A_cons;
        A.bottomRows(free_vars_num) = A_free;
        MatrixXd invA = A.inverse();
        MatrixXd R = invA.transpose() * cost_Q * invA;
        MatrixXd dp = - R.bottomRightCorner(free_vars_num, free_vars_num).inverse() * 
            R.topRightCorner(known_consts_num, free_vars_num).transpose() * B_cons;
        MatrixXd B = MatrixXd::Zero(num_segments*num_poly_coeff, dim);
        B.topRows(known_consts_num) = B_cons;
        B.bottomRows(free_vars_num) = dp;
        optim_coeffs = invA * B;
    }
    else{
        optim_coeffs = A_cons.inverse() * B_cons;
    }

    // if(optim_coeffs.array().isNaN().any()){
    //     success = false;
    // }
    // else{
    //     success = true;
    // }
    success = true;
    return optim_coeffs;
}

MatrixXd MinimumSnap::SolveQPClosedForm(const MatrixXd &waypoints, const VectorXd &time_allocation){
    if(!check_eq(waypoints.rows(), time_allocation.size() + 1u)){
        ROS_INFO("Invalid Input Dimension!");
        success = false;
        return MatrixXd(1,1);
    }

    const unsigned int dim = waypoints.cols();
    unsigned int start_constraint_num = 0;
    unsigned int end_constraint_num = 0;
    if(poly_order_%2 != 0){
        start_constraint_num = int((poly_order_+1)/2);
        end_constraint_num = int((poly_order_+1)/2);
    }
    else{
        start_constraint_num = int(poly_order_/2+1);
        end_constraint_num = int(poly_order_/2);
    }
    MatrixXd front_deriv_consts = MatrixXd::Zero(start_constraint_num-1, dim);
    MatrixXd end_deriv_consts = MatrixXd::Zero(end_constraint_num-1, dim);

    const unsigned int num_poly_coeff = poly_order_ + 1;
    const unsigned int num_segments = time_allocation.size();
    const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;
    const unsigned int known_consts_num = start_constraint_num + end_constraint_num + (ctn_order_+2)*(num_segments-1);
    const unsigned int free_vars_num = num_segments*num_poly_coeff - known_consts_num;

    MatrixXd A_cons = MatrixXd::Zero(known_consts_num, num_segments*num_poly_coeff);
    MatrixXd B_cons = MatrixXd::Zero(known_consts_num, dim);
    MatrixXd A_free = MatrixXd::Zero(free_vars_num, num_segments*num_poly_coeff);

    // start and end constraints
    A_cons.block(0,0,start_constraint_num,num_poly_coeff) = multi_poly_derivative(0.0, 0, start_constraint_num-1, poly_order_);
    B_cons.row(0) = waypoints.row(0);
    B_cons.block(1,0,start_constraint_num-1,dim) = front_deriv_consts;
    A_cons.block(start_constraint_num,(num_segments-1)*num_poly_coeff,end_constraint_num,num_poly_coeff) = 
        multi_poly_derivative(time_allocation(num_segments-1), 0, end_constraint_num-1, poly_order_);
    B_cons.row(start_constraint_num) = waypoints.row(num_segments);
    B_cons.block(start_constraint_num+1,0,end_constraint_num-1,dim) = end_deriv_consts;

    // midpoint constraints
    const unsigned int tmp_row1 = start_constraint_num + end_constraint_num;
    for(unsigned int i=0; i<num_segments-1; i++){
        A_cons.block(tmp_row1+i,num_poly_coeff*i,1,num_poly_coeff) = 
            poly_derivative(time_allocation(i), 0, poly_order_);
        B_cons.row(tmp_row1+i) = waypoints.row(1+i);
    }

    // continuity constraints
    const unsigned int tmp_row2 = tmp_row1 + num_segments - 1;
    for(unsigned int i=0; i<num_segments-1; i++){
        A_cons.block(tmp_row2+i*(ctn_order_+1),i*num_poly_coeff,ctn_order_+1,num_poly_coeff) = 
            multi_poly_derivative(time_allocation(i), 0, ctn_order_, poly_order_);
        A_cons.block(tmp_row2+i*(ctn_order_+1),(i+1)*num_poly_coeff,ctn_order_+1,num_poly_coeff) = 
            - multi_poly_derivative(0.0, 0, ctn_order_, poly_order_);
    }

    // t=T_i free variables
    for(unsigned int i=0; i<num_segments-1; i++){
        A_free.block(i*(end_constraint_num-1),i*num_poly_coeff,end_constraint_num-1,num_poly_coeff) = 
            multi_poly_derivative(time_allocation(i), 1, end_constraint_num-1, poly_order_);
    }

    // t=0 free variables
    const unsigned int tmp_row3 = (num_segments-1) * (end_constraint_num - 1);
    if(ctn_order_ < start_constraint_num-1){
        unsigned int drow = start_constraint_num - ctn_order_ - 1;
        for(unsigned int i=0; i<num_segments-1; i++){
            A_free.block(tmp_row3+i*drow,(i+1)*num_poly_coeff,drow,num_poly_coeff) = 
                multi_poly_derivative(0.0, ctn_order_+1, start_constraint_num-1, poly_order_);
        }
    }

    // QP formulation
    axis_weight = VectorXd::Ones(dim);
    MatrixXd cost_Q = form_cost_mat(time_allocation, poly_order_);
    MatrixXd optim_coeffs = MatrixXd::Zero(num_segments*num_poly_coeff, dim);

    if(free_vars_num > 0){
        MatrixXd A = MatrixXd::Zero(num_segments*num_poly_coeff, num_segments*num_poly_coeff);
        A.topRows(known_consts_num) = A_cons;
        A.bottomRows(free_vars_num) = A_free;
        MatrixXd invA = A.inverse();
        MatrixXd R = invA.transpose() * cost_Q * invA;
        MatrixXd dp = - R.bottomRightCorner(free_vars_num, free_vars_num).inverse() * 
            R.topRightCorner(known_consts_num, free_vars_num).transpose() * B_cons;
        MatrixXd B = MatrixXd::Zero(num_segments*num_poly_coeff, dim);
        B.topRows(known_consts_num) = B_cons;
        B.bottomRows(free_vars_num) = dp;
        optim_coeffs = invA * B;
    }
    else{
        optim_coeffs = A_cons.inverse() * B_cons;
    }

    // if(optim_coeffs.array().isNaN().any()){
    //     success = false;
    // }
    // else{
    //     success = true;
    // }
    success = true;
    return optim_coeffs;
}

bool MinimumSnap::IsPolyGenerated() const {
    return success;
}

unsigned int MinimumSnap::GetPolyOrder() const {
    return poly_order_;
}