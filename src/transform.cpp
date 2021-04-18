#include "scan_matching/transform.h"

/**
 * Overload function for not equal to operator
 * @param t     Other Transform object
 * @return      True or False
**/
bool Transform::operator != (const Transform &t) {
    return std::abs(x_trans_ - t.x_trans_) < EPSILON && std::abs(y_trans_ - t.y_trans_) < EPSILON && std::abs(theta_ - t.theta_) < EPSILON;
}

/**
 * Apply the transform to a point
 * @param p     Point to apply transorm to
 * @return      Transformed point
**/
Point Transform::apply(Point p) const { 
    // Perform the rotation
    p.theta_ += theta_;
    // Perform the translation
    float x = p.getX() + x_trans_;
    float y = p.getY() + y_trans_;
    p.r_ = std::sqrt(x * x + y * y);
    p.theta_ = std::atan2(y,x);
    return p;
}

/**
 * Convert the transformation into a matrix form
 * @return The transformation matrix
**/
Eigen::Matrix3f Transform::getMatrix() const {
    Eigen::Matrix3f trans_mat;
    trans_mat << std::cos(theta_), -std::sin(theta_), x_trans_,
                 std::sin(theta_), std::cos(theta_) , y_trans_,
                 0               , 0                , 1;
    return trans_mat;
}

/**
 * Apply a transform to a vector of points
 * @param t         Transform to apply
 * @param points    Vector of points
 * @return          Transformed points
**/
void transformPoints(const Transform &t, const std::vector<Point> &points, std::vector<Point> &transformed_points) {
    // Clear the old points
    transformed_points.clear();
    for(int i = 0; i < points.size(); i++) {
        transformed_points.push_back(t.apply(points[i]));
    }
}

/**
 * Perform the optimization and update the transform
 * @param corr          The correspondences vector
 * @param curr_trans    The transform to be updated
**/
void updateTransform(const std::vector<Correspondence> &corr, Transform &curr_trans) {
    // Define the optimization matrices
    Eigen::MatrixXf M_i(2, 4);
    M_i.setZero();
    M_i(0,0) = 1;
    M_i(1,1) = 1; 
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;

    Eigen::Matrix4f M, W;
    M.setZero();
    W << 0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1;
    Eigen::MatrixXf g(4, 1);
    g.setZero();

    for(auto &c:corr) {
        M_i(0,2) = c.p_.getX();
        M_i(0,3) = -c.p_.getY();
        M_i(1,2) = c.p_.getY();
        M_i(1,3) = c.p_.getX();
        C_i = c.n_ * c.n_.transpose();
        
        M += M_i.transpose() * C_i * M_i;
        g += (-2 * c.p1_.getVector().transpose() * C_i * M_i).transpose();
    }
    
    // Define sub-matrices A, B, D from M
    Eigen::Matrix2f A, B, D;
    A = 2 * M.block<2,2>(0,0);
    B = 2 * M.block<2,2>(0,2);
    D = 2 * M.block<2,2>(2,2);

    // Define S and S_A matrices from the matrices A B and D
    Eigen::Matrix2f S;
    Eigen::Matrix2f S_A;
    S = D - B.transpose() * A.inverse() * B;
    float S_det = S.determinant();
    S_A = S_det * S.inverse();

    // Find the coefficients of the quadratic function of lambda
    double pow_4, pow_3, pow_2, pow_1, pow_0;

    pow_4 = 16;
    double con1 = 2 * (S(0,0) + S(1,1));
    pow_3 = 8 * con1;
    Eigen::Matrix4f A_pow_2;
    A_pow_2.block<2,2>(0,0) = A.inverse() * B * B.transpose() * A.inverse().transpose();
    A_pow_2.block<2,2>(0,2) = - A.inverse() * B;
    A_pow_2.block<2,2>(2,0) = (- A.inverse() * B).transpose();
    A_pow_2.block<2,2>(2,2) = Eigen::Matrix2f::Identity(2,2);

    pow_2 = 8 * S_det + con1 * con1 - (4 * g.transpose() * A_pow_2 * g)(0,0);

    Eigen::Matrix4f A_pow_1;
    A_pow_1.block<2,2>(0,0) = A.inverse() * B * S_A * B.transpose() * A.inverse().transpose();
    A_pow_1.block<2,2>(0,2) = - A.inverse() * B * S_A;
    A_pow_1.block<2,2>(2,0) = (- A.inverse() * B * S_A).transpose();
    A_pow_1.block<2,2>(2,2) = S_A;

    pow_1 = 2 * con1 * S_det - (4 * g.transpose() * A_pow_1 * g)(0,0);

    Eigen::Matrix4f A_pow_0;
    A_pow_0.block<2,2>(0,0) = A.inverse() * B * S_A.transpose() * S_A * B.transpose() * A.inverse().transpose();
    A_pow_0.block<2,2>(0,2) = - A.inverse() * B * S_A.transpose() * S_A;
    A_pow_0.block<2,2>(2,0) = (- A.inverse() * B * S_A.transpose() * S_A).transpose();
    A_pow_0.block<2,2>(2,2) = S_A.transpose() * S_A;

    pow_0 = S_det * S_det - (g.transpose() * A_pow_0 * g)(0,0);

    double x1, x2, x3, x4;
    int nbroots = solve_deg4(pow_4, pow_3, pow_2, pow_1, pow_0, x1, x2, x3, x4);
    double lambda = std::max({x1,x2,x3,x4});

    // Find the value of x which is the vector for translation and rotation
    Eigen::Vector4f x = -(2 * (M + lambda * W)).inverse().transpose() * g;

    // Convert from x to new transform
    double theta = atan2(x(3), x(2));
    curr_trans = Transform(x(0), x(1), theta);
}