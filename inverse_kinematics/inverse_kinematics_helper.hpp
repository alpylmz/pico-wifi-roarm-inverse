#ifndef INVERSE_KINEMATICS_HELPER_H
#define INVERSE_KINEMATICS_HELPER_H

#include "inverse_kinematics_config.hpp"
#include <string.h>
#include <math.h>

// 3x3 matrix multiplication
void matrix_mult_3x3(double a[3][3], double b[3][3], double c[3][3]){
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 3x3 matrix by 3x1 vector multiplication
void matrix_mult_3x1(double a[3][3], double b[3], double c[3]){
    for (int i = 0; i < 3; i++) {
        c[i] = 0;
        for (int j = 0; j < 3; j++) {
            c[i] += a[i][j] * b[j];
        }
    }
}

inverseSE3 actInv(inverseSE3 oMdes, inverseSE3 oMi){

    double oMi_rot_transpose[3][3] = {
        {oMi.rotation[0][0], oMi.rotation[1][0], oMi.rotation[2][0]},
        {oMi.rotation[0][1], oMi.rotation[1][1], oMi.rotation[2][1]},
        {oMi.rotation[0][2], oMi.rotation[1][2], oMi.rotation[2][2]}
    };

    // multiply oMdes_rot_transpose with oMi.rotation
    double result_rot[3][3];

    matrix_mult_3x3(oMi_rot_transpose, oMdes.rotation, result_rot);

    // rot.transpose() * (m2.translation() - translation())
    double right_side[3] = {
        oMdes.translation[0] - oMi.translation[0],
        oMdes.translation[1] - oMi.translation[1],
        oMdes.translation[2] - oMi.translation[2]
    };

    double result_trans[3];

    matrix_mult_3x1(oMi_rot_transpose, right_side, result_trans);

    inverseSE3 result;
    result.translation[0] = result_trans[0];
    result.translation[1] = result_trans[1];
    result.translation[2] = result_trans[2];

    result.rotation[0][0] = result_rot[0][0];
    result.rotation[0][1] = result_rot[0][1];
    result.rotation[0][2] = result_rot[0][2];
    result.rotation[1][0] = result_rot[1][0];
    result.rotation[1][1] = result_rot[1][1];
    result.rotation[1][2] = result_rot[1][2];
    result.rotation[2][0] = result_rot[2][0];
    result.rotation[2][1] = result_rot[2][1];
    result.rotation[2][2] = result_rot[2][2];

    return result;
}


// Compute the skew-symmetric matrix (hat operator) of a 3D vector.
void hat(const double omega[3], double omega_hat[3][3]) {
    omega_hat[0][0] = 0.0;         omega_hat[0][1] = -omega[2];  omega_hat[0][2] =  omega[1];
    omega_hat[1][0] =  omega[2];    omega_hat[1][1] = 0.0;         omega_hat[1][2] = -omega[0];
    omega_hat[2][0] = -omega[1];    omega_hat[2][1] =  omega[0];  omega_hat[2][2] = 0.0;
}

// Multiply two 3x3 matrices: C = A * B.
void mat3x3_mult(const double A[3][3], const double B[3][3], double C[3][3]) {
    int i, j, k;
    for (i = 0; i < 3; i++){
        for (j = 0; j < 3; j++){
            C[i][j] = 0.0;
            for (k = 0; k < 3; k++){
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Compute the inverse of the left Jacobian of SO(3).
// Corrected: note the minus sign before the hat(omega) term.
void compute_Jinv(const double omega[3], double theta, double Jinv[3][3]) {
    int i, j;
    if (theta < LOG6_EPS) {
        // For small theta, use first-order approximation: Jinv ≈ I - 0.5 * hat(omega)
        Jinv[0][0] = 1.0; Jinv[0][1] =  0.5 * omega[2]; Jinv[0][2] = -0.5 * omega[1];
        Jinv[1][0] = -0.5 * omega[2]; Jinv[1][1] = 1.0; Jinv[1][2] =  0.5 * omega[0];
        Jinv[2][0] =  0.5 * omega[1]; Jinv[2][1] = -0.5 * omega[0]; Jinv[2][2] = 1.0;
    } else {
        double a = 0.5;
        double b = 1.0/(theta*theta) - (1.0+cos(theta))/(2.0*theta*sin(theta));
        double omega_hat[3][3];
        hat(omega, omega_hat);
        double omega_hat2[3][3];
        mat3x3_mult(omega_hat, omega_hat, omega_hat2);
        for (i = 0; i < 3; i++){
            for (j = 0; j < 3; j++){
                Jinv[i][j] = ((i == j) ? 1.0 : 0.0) - a * omega_hat[i][j] + b * omega_hat2[i][j];
            }
        }
    }
}

// The log6_SE3 function computes the 6D logarithm (twist) of an SE3 transformation.
// Convention: first three components are the angular part, and the last three are the translational part.
void log6_SE3(const inverseSE3 *T, double xi[6]) {
    int i, j;
    // Extract rotation matrix R and translation vector t.
    double R[3][3];
    double t[3];
    for (i = 0; i < 3; i++){
        for (j = 0; j < 3; j++){
            R[i][j] = T->rotation[i][j];
        }
        t[i] = T->translation[i];
    }
    
    // Compute the rotation angle theta from the trace of R.
    double trace = R[0][0] + R[1][1] + R[2][2];
    double cos_theta = (trace - 1.0) / 2.0;
    if(cos_theta > 1.0)  cos_theta = 1.0;
    if(cos_theta < -1.0) cos_theta = -1.0;
    double theta = acos(cos_theta);
    
    // Compute the rotation vector omega.
    double omega[3] = {0.0, 0.0, 0.0};
    if (theta < LOG6_EPS) {
        omega[0] = (R[2][1] - R[1][2]) / 2.0;
        omega[1] = (R[0][2] - R[2][0]) / 2.0;
        omega[2] = (R[1][0] - R[0][1]) / 2.0;
    } else {
        double factor = theta / (2.0 * sin(theta));
        omega[0] = factor * (R[2][1] - R[1][2]);
        omega[1] = factor * (R[0][2] - R[2][0]);
        omega[2] = factor * (R[1][0] - R[0][1]);
    }
    
    // Compute the inverse left Jacobian for SO(3).
    double Jinv[3][3];
    compute_Jinv(omega, theta, Jinv);
    
    // Compute the translational component: v = Jinv * t.
    double v[3] = {0.0, 0.0, 0.0};
    for (i = 0; i < 3; i++){
        for (j = 0; j < 3; j++){
            v[i] += Jinv[i][j] * t[j];
        }
    }
    
    // Assemble the twist vector: [omega; v].
    xi[0] = v[0];
    xi[1] = v[1];
    xi[2] = v[2];
    xi[3] = omega[0];
    xi[4] = omega[1];
    xi[5] = omega[2];
}
// --- End log6 implementation ---


// Done by chatgpt
// Compute the joint Jacobian for the chain up to 'joint_id'.
// The forward kinematics of each joint is given in the SE3s structure 'fk'.
// The computed Jacobian J is expressed in the joint (local) frames and uses our [v; omega] ordering.
void computeJointJacobian(const SE3s *fk, int joint_id, Matrix6x *J)
{
    int i, k, l;
    // Clear the Jacobian matrix.
    for (i = 0; i < 6; i++) {
        for (int j = 0; j < NU; j++) {
            J->data[i][j] = 0.0;
        }
    }
    
    // Get the end-effector transformation (T_e) from the FK results.
    const inverseSE3 *T_e = &fk->SE3[joint_id];
    
    // For each joint i from the base (i = 0) up to joint_id:
    for (i = 0; i <= joint_id; i++)
    {
        // Compute the relative transformation from joint i to the end-effector:
        // T_ie = T_i^{-1} * T_e.
        inverseSE3 T_ie = actInv(fk->SE3[i], *T_e);
        
        // Extract rotation R and translation p from T_ie.
        double R[3][3], p[3];
        for (k = 0; k < 3; k++) {
            p[k] = T_ie.translation[k];
            for (l = 0; l < 3; l++) {
                R[k][l] = T_ie.rotation[k][l];
            }
        }
        
        // Compute the joint axis in joint i’s frame.
        // In our model, for a revolute joint the motion is along the local z-axis.
        // When transformed by R, this gives R * [0; 0; 1] = the third column of R.
        double z_i[3] = { R[0][2], R[1][2], R[2][2] };
        
        // Compute the linear part: v_i = hat(p) * (R * [0;0;1]).
        // Here, hat(p)*z_i is equivalent to p cross z_i.
        double v_i[3] = {
            p[1]*z_i[2] - p[2]*z_i[1],
            p[2]*z_i[0] - p[0]*z_i[2],
            p[0]*z_i[1] - p[1]*z_i[0]
        };
        
        // The angular part is simply z_i.
        double omega_i[3] = { z_i[0], z_i[1], z_i[2] };
        
        // Set the i-th column of the Jacobian.
        // Our ordering is: first three rows: linear part, last three rows: angular part.
        J->data[0][i] = v_i[0];
        J->data[1][i] = v_i[1];
        J->data[2][i] = v_i[2];
        J->data[3][i] = omega_i[0];
        J->data[4][i] = omega_i[1];
        J->data[5][i] = omega_i[2];
    }
}

// --- Begin inverse_SE3 helper ---
// Compute the inverse of an SE3 transform using the standard formula:
// T^{-1} = [ R^T, -R^T * p ]
inverseSE3 inverse_SE3(const inverseSE3 *T)
{
    inverseSE3 T_inv;
    int i, j;
    double R_inv[3][3];

    // Compute the transpose of R.
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            R_inv[i][j] = T->rotation[j][i]; // Transpose: (i,j) <- (j,i)
        }
    }

    // Set the inverse rotation.
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            T_inv.rotation[i][j] = R_inv[i][j];
        }
    }

    // Compute the inverse translation: t_inv = -R^T * p.
    double t_inv[3] = {0.0, 0.0, 0.0};
    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            t_inv[i] += R_inv[i][j] * T->translation[j];
        }
        t_inv[i] = -t_inv[i];
    }
    for(i = 0; i < 3; i++){
        T_inv.translation[i] = t_inv[i];
    }

    return T_inv;
}
// --- End inverse_SE3 helper ---

//---------------------------------------------------------------------
// Helper function: Compute the logarithm of a rotation matrix (log3).
// It computes the angular part 'w' and its norm 'theta' such that
//   R = exp( hat(w) )  and  w = theta * axis.
void log3(const double R[3][3], double w[3], double *theta)
{
    double trace = R[0][0] + R[1][1] + R[2][2];
    double cos_theta = (trace - 1.0) / 2.0;
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    *theta = acos(cos_theta);
    double sin_theta = sin(*theta);
    if (fabs(*theta) < 1e-8 || fabs(sin_theta) < 1e-8)
    {
        w[0] = 0.0; w[1] = 0.0; w[2] = 0.0;
    }
    else
    {
        double factor = *theta / (2.0 * sin_theta);
        w[0] = factor * (R[2][1] - R[1][2]);
        w[1] = factor * (R[0][2] - R[2][0]);
        w[2] = factor * (R[1][0] - R[0][1]);
    }
}
//---------------------------------------------------------------------
// Helper function: Compute the derivative of the log map for SO(3) (Jlog3).
// The output is a 3×3 matrix computed as:
//   Jlog3 = alpha * (w * w^T) + diag(diag_value) + 0.5 * hat(w)
// where alpha and diag_value use a small-angle threshold.
void Jlog3(double theta, const double w[3], double Jlog3_out[3][3])
{
    double st = sin(theta);
    double ct = cos(theta);
    double prec = 1e-3; // small-angle threshold
    double alpha, diag_value;
    if (fabs(theta) < prec) {
        alpha = 1.0/12.0 + (theta*theta)/720.0;
        diag_value = 0.5*(2 - theta*theta/6.0);
    } else {
        alpha = 1.0/(theta*theta) - (st/theta)/(2.0*(1-ct));
        diag_value = 0.5*(theta * (st/(1-ct)));
    }
    // Outer product of w: (w*w^T)
    double outer[3][3];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            outer[i][j] = w[i] * w[j];
        }
    }
    // Build Jlog3 = alpha*(w*w^T) + diag(diag_value)
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            Jlog3_out[i][j] = alpha * outer[i][j];
        }
        Jlog3_out[i][i] += diag_value;
    }
    // Add the skew-symmetric part: 0.5 * hat(w)
    // hat(w) = [  0    -w[2]  w[1] ]
    //          [ w[2]    0   -w[0] ]
    //          [-w[1]  w[0]    0   ]
    Jlog3_out[0][1] += -0.5 * w[2];
    Jlog3_out[0][2] +=  0.5 * w[1];
    Jlog3_out[1][0] +=  0.5 * w[2];
    Jlog3_out[1][2] += -0.5 * w[0];
    Jlog3_out[2][0] += -0.5 * w[1];
    Jlog3_out[2][1] +=  0.5 * w[0];
}

//---------------------------------------------------------------------
// Main function: Compute the derivative of the log map for SE(3) (Jlog6).
// It decomposes the 6×6 matrix into 3×3 blocks as follows:
//     [ A   B ]
//     [ 0   A ]
// where A = Jlog3 (the SO(3) part) and B is computed from the translation p.
void Jlog6(const inverseSE3 *T, double Jlog[6][6])
{
    // Extract rotation R and translation p.
    const double (*R)[3] = T->rotation;
    const double *p = T->translation;

    // Compute log3 of R: obtain angular vector w and its norm theta.
    double w[3];
    double theta;
    log3(R, w, &theta);

    // Compute the 3×3 Jlog3 block for rotation.
    double A[3][3];
    Jlog3(theta, w, A);

    // Let D = A (the bottom-right block).
    double D[3][3];
    memcpy(D, A, sizeof(D));

    // Precompute parameters for translation part.
    double t2 = theta * theta;
    double tinv = (fabs(theta) < 1e-8 ? 0.0 : 1.0/theta);
    double t2inv = (fabs(theta) < 1e-8 ? 0.0 : tinv * tinv);
    double st = sin(theta);
    double ct = cos(theta);
    double denom = (fabs(1-ct) < 1e-8 ? 1e-8 : (1-ct));
    double inv_2_2ct = 1.0/(2.0 * denom);
    double prec = 1e-3;
    double beta;
    if (fabs(theta) < prec) {
        beta = 1.0/12.0 + t2/720.0;
    } else {
        beta = t2inv - (st * tinv * inv_2_2ct);
    }
    double beta_dot_over_theta;
    if (fabs(theta) < prec) {
        beta_dot_over_theta = 1.0/360.0;
    } else {
        beta_dot_over_theta = -2.0/(t2*t2) + (1.0 + st*tinv) * t2inv * inv_2_2ct;
    }

    // Compute wᵀ * p.
    double wTp = w[0]*p[0] + w[1]*p[1] + w[2]*p[2];

    // Compute temporary vector: v3_tmp = (beta_dot_over_theta * wTp)*w - (t2*beta_dot_over_theta + 2*beta)*p.
    double v3_tmp[3];
    for (int i = 0; i < 3; i++){
        v3_tmp[i] = (beta_dot_over_theta * wTp) * w[i] - (t2 * beta_dot_over_theta + 2 * beta) * p[i];
    }

    // Compute the block C (bottom-left 3×3).
    // Start with outer product: C = v3_tmp * w^T.
    double C[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            C[i][j] = v3_tmp[i] * w[j];
        }
    }
    // Add beta * (w * p^T).
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            C[i][j] += beta * w[i] * p[j];
        }
    }
    // Add wTp * beta to the diagonal.
    for (int i = 0; i < 3; i++){
        C[i][i] += wTp * beta;
    }
    // Add the skew-symmetric matrix of (0.5*p) to C.
    double hat_half_p[3][3] = {
        { 0,         -0.5*p[2],  0.5*p[1] },
        { 0.5*p[2],   0,        -0.5*p[0] },
        { -0.5*p[1],  0.5*p[0],   0 }
    };
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            C[i][j] += hat_half_p[i][j];
        }
    }

    // Compute block B (top-right 3×3): B = C * A.
    double B[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                B[i][j] += C[i][k] * A[k][j];
            }
        }
    }

    // Assemble the final 6×6 Jlog matrix:
    // [ A    B ]
    // [ 0    D ]
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            Jlog[i][j]     = A[i][j];      // top-left
            Jlog[i][j+3]   = B[i][j];      // top-right
            Jlog[i+3][j]   = 0.0;          // bottom-left (set to zero)
            Jlog[i+3][j+3] = D[i][j];      // bottom-right (same as A)
        }
    }
}


// LDLT solver for a 6x6 symmetric matrix.
// Solves A*x = b for x.
void ldlt_solve_6(const Matrix6x6 *A, const double b[6], double x[6])
{
    double L[6][6] = {0};
    double D[6] = {0};
    int i, j, k;

    // Initialize L to identity.
    for (i = 0; i < 6; i++) {
        for (j = 0; j < 6; j++) {
            L[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Perform LDLᵀ decomposition.
    for (i = 0; i < 6; i++) {
        double sum = 0.0;
        for (k = 0; k < i; k++) {
            sum += L[i][k] * L[i][k] * D[k];
        }
        D[i] = A->data[i][i] - sum;
        // Check for nearly-zero pivots and regularize.
        if (fabs(D[i]) < REG_EPS) {
            D[i] = REG_EPS;
        }
        for (j = i + 1; j < 6; j++) {
            double sum2 = 0.0;
            for (k = 0; k < i; k++) {
                sum2 += L[j][k] * L[i][k] * D[k];
            }
            L[j][i] = (A->data[j][i] - sum2) / D[i];
        }
    }

    // Solve L * y = b (forward substitution).
    double y[6];
    for (i = 0; i < 6; i++) {
        double sum = 0.0;
        for (j = 0; j < i; j++) {
            sum += L[i][j] * y[j];
        }
        y[i] = b[i] - sum;
    }

    // Solve D * z = y (element-wise division).
    double z[6];
    for (i = 0; i < 6; i++) {
        z[i] = y[i] / D[i];
    }

    // Solve Lᵀ * x = z (backward substitution).
    for (i = 5; i >= 0; i--) {
        double sum = 0.0;
        for (j = i + 1; j < 6; j++) {
            sum += L[j][i] * x[j];
        }
        x[i] = z[i] - sum;
    }
}


// Wrap angle to the range [-pi, pi]
double wrapAngle(double angle) {
    while(angle > M_PI)  angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Integrate the joint configuration.
// q: current joint configuration (angles, cos, sin)
// v: joint velocity vector (update)
// dt: time step (DT)
// Returns: new joint configuration.
joint_position integrate(const joint_position *q, const struct Vectorxd *v) {
    joint_position new_q;
    for (int i = 0; i < NU; i++) {
        // Update the joint angle.
        new_q.position[i] = q->position[i] + v->data[i] * DT;
        // Optional: wrap the angle to [-pi, pi]
        new_q.position[i] = wrapAngle(new_q.position[i]);
        // Update cosine and sine.
        new_q.cos[i] = cos(new_q.position[i]);
        new_q.sin[i] = sin(new_q.position[i]);
    }
    return new_q;
}






#endif // INVERSE_KINEMATICS_HELPER_H