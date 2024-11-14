#ifndef __BODYCONTROLLER_H__
#define __BODYCONTROLLER_H__

#include "globVariable.h"
#include "bodyKinematics.h"
#include "trajectory.h"
#include <mujoco/mujoco.h>
#include "filter.h"
#include <vector>

using namespace std;

class bodyController
{

private:
    // Feedforward
    Vector4d Feedforward_output_vel;

    // Pos_PID
    Vector3d Kp_pos;
    Vector3d Kd_pos;
    double cut_off_D_pos;
    Vector4d PID_output_pos;
    Vector4d error_pos;
    Vector4d error_old_pos;
    Vector4d error_dot_pos;
    Vector4d error_dot_old_pos;

    // Vel_PID
    Vector3d Kp_vel;
    Vector3d Kd_vel;
    double cut_off_D_vel;
    Vector3d PID_output_vel;
    Vector3d error_vel;
    Vector3d error_old_vel;
    Vector3d error_dot_vel;
    Vector3d error_dot_old_vel;

    //DOB
    Vector4d rhs_dob;
    Vector4d rhs_dob_old;
    Vector4d lhs_dob;
    Vector4d lhs_dob_old;
    double lhs_dob_LPF[NDOF_TRUNK];
    double lhs_dob_LPF_old[NDOF_TRUNK];
    double rhs_dob_LPF[NDOF_TRUNK];
    double rhs_dob_LPF_old[NDOF_TRUNK];
    Vector4d Dist_hat;
    double cut_off_dob;

public:
    bodyController(); //생성자
    ~bodyController(); //소멸자
    
    Vector4d Feedforward_vel(TrunkModel_* trunk_model, int flag);
    void pid_gain_pos(double kp, double kd, double cut_off_pos, int flag);
    Vector4d PID_pos(TrunkModel_* trunk_model);
    // Vector4d DOB(TrunkModel_* trunk_model, double cut_off ,int flag);
    void ctrl_update(); // 이걸 사용할지에 대해서 생각해보기
};
#endif // __BODYCONTROLLER_H__

