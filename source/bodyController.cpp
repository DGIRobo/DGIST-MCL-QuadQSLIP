#include "bodyController.h"
#include <iostream>
using namespace std;

bodyController::bodyController() {
    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        // Pos PID
        Kp_pos[i] = 0.0;
        Kd_pos[i] = 0.0;

        error_pos[i] = 0.0;
        error_old_pos[i] = error_pos[i];
        error_dot_pos[i] = 0.0;
        error_dot_old_pos[i] = error_dot_pos[i];
        PID_output_pos[i] = 0;

        // Vel PID
        Kp_vel[i] = 0.0;
        Kd_vel[i] = 0.0;

        error_vel[i] = 0.0;
        error_old_vel[i] = error_vel[i];
        error_dot_vel[i] = 0.0;
        error_dot_old_vel[i] = error_dot_vel[i];
        PID_output_vel[i] = 0;
    }

    for (int i = 0; i < NUM_LEG; i++)
    {
        // Trunk DOB
        rhs_dob[i] = 0.0;
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob[i] = 0.0;
        lhs_dob_old[i] = lhs_dob[i];
        lhs_dob_LPF[i] = 0.0;
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];
        rhs_dob_LPF[i] = 0.0;
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        Dist_hat[i] = 0.0;
    }
};

bodyController::~bodyController(){};

Vector4d bodyController::Feedforward_vel(TrunkModel_* trunk_model, int flag)
{
    if (flag == 1) {
        Feedforward_output_vel = trunk_model->jacb_trunk_pos * trunk_model->vel_trunk_des;
    }
    else {
        for (int i = 0; i < NUM_LEG; i++) {
            Feedforward_output_vel[i] = 0;
        }
    }
    
    return Feedforward_output_vel;
};

void bodyController::pid_gain_pos(double kp, double kd, double cut_off_pos, int flag)
{
    if (flag == 1) {
        for (int i = 0; i < NDOF_TRUNK; i++)
        {
            Kp_pos[i] = kp;
            Kd_pos[i] = kd;
            cut_off_D_pos = cut_off_pos;
        }
    }
    else {
        for (int i = 0; i < NDOF_TRUNK; i++)
        {
            Kp_pos[i] = 0;
            Kd_pos[i] = 0;
            cut_off_D_pos = cut_off_pos;
        }
    }
};

Vector4d bodyController::PID_pos(TrunkModel_* trunk_model)
{
    for (int i = 0; i < NDOF_TRUNK; i++) // Error를 state 모델에 넣을 필요 있는지 생각해봐야함. error는 여기에 있어도 됨. //error들 update 해줘야함
    {
        error_pos[i] = trunk_model->pos_trunk_des[i] - trunk_model->pos_trunk_est[i];
        //error_pos[i] = trunk_model->pos_trunk_des[i] - trunk_model->pos_trunk_act[i];
        error_old_pos[i] = trunk_model->pos_trunk_des_old[i] - trunk_model->pos_trunk_est_old[i];
        //error_old_pos[i] = trunk_model->pos_trunk_des_old[i] - trunk_model->pos_trunk_act_old[i];
        
        error_dot_pos[i] = tustin_derivative(error_pos[i], error_old_pos[i], error_dot_old_pos[i], cut_off_D_pos);
        
        // DOB 끄면 PID만 사용해야하는데 state model에 넣지 않아도 되는지 생각해봐야함.
        PID_output_pos[i] = Kp_pos[i] * error_pos[i] + Kd_pos[i] * error_dot_pos[i]; // 이걸 return을 사용하면?
    }
    return PID_output_pos;
};

/*
Vector4d bodyController::DOB(TrunkModel_* trunk_model, double cut_off ,int flag)
{
    cut_off_dob = cut_off;
    lhs_dob = trunk_model->ctrl_input_RW_from_trunk_old;
    rhs_dob = trunk_model->r;

    if (flag == true)
    {
        for (int i = 0; i < NUM_LEG; i++)
        {
            lhs_dob_LPF[i] = lowpassfilter(lhs_dob[i], lhs_dob_old[i], lhs_dob_LPF_old[i], cut_off_dob); 
            rhs_dob_LPF[i] = lowpassfilter(rhs_dob[i], rhs_dob_old[i], rhs_dob_LPF_old[i], cut_off_dob); 

            Dist_hat[i] = lhs_dob_LPF[i] - rhs_dob_LPF[i];
            //printf("%f \n", Dist_hat[i]);
        }
    }
    else
    {
        for (int i = 0; i < NUM_LEG; i++)
            Dist_hat[i] = 0;
    }
    
    return Dist_hat;
};
*/

void bodyController::ctrl_update()
{
    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        //PID pos
        error_old_pos[i] = error_pos[i];
        error_dot_old_pos[i] = error_dot_pos[i];

        //PID vel
        error_old_vel[i] = error_vel[i];
        error_dot_old_vel[i] = error_dot_vel[i];
    }

    for (int i = 0; i < NUM_LEG; i++)
    {
        //Trunk DOB
        rhs_dob_old[i] = rhs_dob[i];
        lhs_dob_old[i] = lhs_dob[i];
        rhs_dob_LPF_old[i] = rhs_dob_LPF[i];
        lhs_dob_LPF_old[i] = lhs_dob_LPF[i];
    }
};