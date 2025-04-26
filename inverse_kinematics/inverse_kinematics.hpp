#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "inverse_kinematics_config.hpp"

#ifdef DOUBLE_VERSION
    //#include "forward_kinematics_indy7_double.h"
    //#include "forward_kinematics_ur5_double.h"
    #include "forward_kinematics_roarm_double.hpp"
    #endif
#ifdef FIXED32_VERSION
    #include "forward_kinematics_indy7_fixed32.h"
#endif

#include "inverse_kinematics_helper.hpp"


void inverse_kinematics_roarm(
    double goal_position[3], // 3D goal position
    double goal_rotation[3][3],  // 3x3 goal rotation matrix
    double* q_start, // joint angles to start, it should be current position usually
    double* q_out // output joint angles
) {
    
    // const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));
    inverseSE3 oMdes = {
        .translation = {goal_position[0], goal_position[1], goal_position[2]},
        .rotation = {
            {goal_rotation[0][0], goal_rotation[0][1], goal_rotation[0][2]},
            {goal_rotation[1][0], goal_rotation[1][1], goal_rotation[1][2]},
            {goal_rotation[2][0], goal_rotation[2][1], goal_rotation[2][2]}
        }
    };

    joint_position q;
    for (int i = 0; i < NU; i++) {
        q.position[i] = q_start[i];
        q.cos[i] = cos(q_start[i]);
        q.sin[i] = sin(q_start[i]);
    }
    
    // pinocchio::Data::Matrix6x J(6, model.nv); 
    Matrix6x J = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < NU; j++) {
            J.data[i][j] = 0.0;
        }
    }

    bool success = false;

    //Vector6d err;
    Vector6d err = {0};
    for (int i = 0; i < 6; i++) {
        err.data[i] = 0.0;
    }

    //Eigen::VectorXd v(model.nv);
    VectorXd v = {0};
    for (int i = 0; i < NU; i++) {
        v.data[i] = 0.0;
    }

    for(int i = 0;; i++){      
        SE3s fk = ForwardKinematics(
            q.cos[0], q.cos[1], q.cos[2], q.cos[3], q.cos[4],
            q.sin[0], q.sin[1], q.sin[2], q.sin[3], q.sin[4],
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0
        );



        // const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
        inverseSE3 iMd = actInv(oMdes, fk.SE3[NU - 1]);


        // err = pinocchio::log6(iMd).toVector(); // in joint frame
        log6_SE3(&iMd, err.data);


        double err_norm = 0;
        for (int k = 0; k < 6; k++){
            err_norm += err.data[k] * err.data[k];
        }
        err_norm = sqrt(err_norm);
        //printf("Error norm: %f\n", err_norm);
        if (err_norm < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }
        
        // pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
        computeJointJacobian(&fk, NU - 1, &J);


        //pinocchio::Data::Matrix6 Jlog;
        Matrix6x6 Jlog = {0};
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 6; l++) {
                Jlog.data[k][l] = 0.0;
            }
        }

        // pinocchio::Jlog6(iMd.inverse(), Jlog);
        inverseSE3 iMd_inv = inverse_SE3(&iMd);

        Jlog6(&iMd_inv, Jlog.data);


        // J = -Jlog * J;
        {
            int i, j, k;
            double newJ[6][NU];
            // Compute newJ = Jlog * J, then negate.
            for (i = 0; i < 6; i++) {
                for (j = 0; j < NU; j++) {
                    newJ[i][j] = 0.0;
                    for (k = 0; k < 6; k++) {
                        newJ[i][j] += Jlog.data[i][k] * J.data[k][j];
                    }
                    newJ[i][j] = -newJ[i][j];
                }
            }
            // Copy the result back into J.
            for (i = 0; i < 6; i++) {
                for (j = 0; j < NU; j++) {
                    J.data[i][j] = newJ[i][j];
                }
            }
        }

        // pinocchio::Data::Matrix6 JJt;
        Matrix6x6 JJt = {0};
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 6; l++) {
                JJt.data[k][l] = 0.0;
            }
        }

        // JJt.noalias() = J * J.transpose();
        // Compute JJt = J * J^T and store directly into the global JJt variable.
        for (int i = 0; i < 6; i++){
            for (int j = 0; j < 6; j++){
                JJt.data[i][j] = 0.0;
                for (int k = 0; k < NU; k++){
                    JJt.data[i][j] += J.data[i][k] * J.data[j][k];
                }
            }
        }

        // JJt.diagonal().array() += damp;
        for (int k = 0; k < 6; k++){
            JJt.data[k][k] += damp;
        }

        // v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        // Then, in your IK loop after computing JJt and adding damping to its diagonal,
        // solve for 'x' and compute v = -Jᵀ*x as follows:
        {
            double x[6];
            // Solve (JJt)x = err.data
            ldlt_solve_6(&JJt, err.data, x);

            // Compute v = -Jᵀ * x. (J is 6xNU; v is NU-dimensional.)
            for (int j = 0; j < NU; j++) {
                double sum = 0.0;
                for (int i = 0; i < 6; i++) {
                    sum += J.data[i][j] * x[i];
                }
                v.data[j] = -sum;
            }
        }


        // q = pinocchio::integrate(model, q, v * DT);
        q = integrate(&q, &v);

        // **** Joint Limit Projection Step ****
        // Ensure that each joint remains within its defined limits.
        //for (int i = 0; i < NU; i++) {
        //    if (q.position[i] < joint_lower[i]) {
        //        q.position[i] = joint_lower[i];
        //        q.cos[i] = cos(q.position[i]);
        //        q.sin[i] = sin(q.position[i]);
        //    }
        //    if (q.position[i] > joint_upper[i]) {
        //        q.position[i] = joint_upper[i];
        //        q.cos[i] = cos(q.position[i]);
        //        q.sin[i] = sin(q.position[i]);
        //    }
        //}
        // ****************************************

        if(i % 10 == 0){
            printf("Iteration %d err: %f \n", i, err_norm);
        }

    }

    if (success)
    {
        printf("Success!\n");
        //printf("Final q: ");
        //for (int k = 0; k < NU; k++){
        //    printf("%f ", q.position[k]);
        //}
    }
    else
    {
        printf("Failed to converge.");
        printf("Final q: ");
        for (int k = 0; k < NU; k++){
            printf("%f ", q.position[k]);
        }
        // return the initial q_start
        for (int k = 0; k < NU; k++){
            q.position[k] = q_start[k];
        }
    }


    // Copy the result into q_out
    for (int i = 0; i < NU; i++) {
        q_out[i] = q.position[i];
    }
    printf("q_out: ");
    for (int i = 0; i < NU; i++) {
        printf("%f ", q_out[i]);
    }
    printf("\n");



}



