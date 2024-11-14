#include "bodyKinematics.h"

bodyKinematics::bodyKinematics(){};

bodyKinematics::~bodyKinematics(){};

void bodyKinematics::state_update(TrunkModel_* trunk_model)
{
    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        // Trunk Sensor
        trunk_model->imu_accl_old[i] = trunk_model->imu_accl[i];
        trunk_model->imu_gyro_old[i] = trunk_model->imu_gyro[i];

        trunk_model->pos_trunk_act_old[i] = trunk_model->pos_trunk_act[i];
        trunk_model->pos_trunk_act_old[i + 3] = trunk_model->pos_trunk_act[i + 3];
        
        trunk_model->vel_trunk_act_old[i] = trunk_model->vel_trunk_act[i];
        trunk_model->vel_trunk_act_old[i + 3] = trunk_model->vel_trunk_act[i + 3];
    }

    for (int i = 0; i < NUM_LEG; i++)
    {
        // Trunk
        trunk_model->pos_trunk_des_old[i] = trunk_model->pos_trunk_des[i];
        trunk_model->pos_trunk_est_old[i] = trunk_model->pos_trunk_est[i];

        trunk_model->vel_trunk_des_old[i] = trunk_model->vel_trunk_des[i];
        trunk_model->vel_trunk_est_old[i] = trunk_model->vel_trunk_est[i];

        trunk_model->trunk_pos_ctrl_input_old[i] = trunk_model->trunk_pos_ctrl_input[i];
        trunk_model->trunk_vel_ctrl_input_old[i] = trunk_model->trunk_vel_ctrl_input[i];
    }
};

void bodyKinematics::model_param_cal(const mjModel* m, mjData* d, TrunkModel_* trunk_model)
{
    /* Trunk Parameter */
    m_hip = 2.5;
    m_trunk_front = 10.;
    m_trunk_rear = 18.;
    m_trunk = 4 * m_hip + m_trunk_front + m_trunk_rear;

    /* Trunk Dimension */
    width_trunk = 0.33;
    length_trunk = 0.64;
}; // param_model parameter

void bodyKinematics::sensor_measure(const mjModel* m, mjData* d, TrunkModel_* trunk_model)
{
    // sensor based estimation
    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        trunk_model->imu_accl[i] = d->sensordata[i];
        trunk_model->imu_gyro[i] = d->sensordata[i + 3];

        //trunk_model->vel_trunk_act[i] = tustin_integrate(trunk_model->imu_accl[i], trunk_model->imu_accl_old[i], trunk_model->vel_trunk_act_old[i]);
        //trunk_model->vel_trunk_act[i + 3] = trunk_model->imu_gyro[i];

        //trunk_model->pos_trunk_act[i] = tustin_integrate(trunk_model->vel_trunk_act[i], trunk_model->vel_trunk_act_old[i], trunk_model->pos_trunk_act_old[i]);
        //trunk_model->pos_trunk_act[i + 3] = tustin_integrate(trunk_model->vel_trunk_act[i + 3], trunk_model->vel_trunk_act_old[i + 3], trunk_model->pos_trunk_act_old[i + 3]);        
    }

    // getting simulation state
    Quaterniond qdot(d->qvel[3], d->qvel[4], d->qvel[5], d->qvel[6]);
    Quaterniond q(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
    Vector3d eulerdot = ToEulerAngles(qdot);
    Vector3d euler = ToEulerAngles(q);
    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        trunk_model->vel_trunk_act[i] = d->qvel[i];
        trunk_model->pos_trunk_act[i] = d->qpos[i];
        trunk_model->vel_trunk_act[i + 3] = eulerdot[i];
        trunk_model->pos_trunk_act[i + 3] = euler[i];
    }
};

void bodyKinematics::jacobianTrunk(TrunkModel_* trunk_model)
{
    // Forward Jacobian: Trunk Angle to radial of leg
    trunk_model->jacb_trunk_pos(0,0) = 1;
    trunk_model->jacb_trunk_pos(0,1) = width_trunk/2;
    trunk_model->jacb_trunk_pos(0,2) = - length_trunk/2;
    trunk_model->jacb_trunk_pos(0,3) = 0;

    trunk_model->jacb_trunk_pos(1,0) = 1;
    trunk_model->jacb_trunk_pos(1,1) = - width_trunk/2;
    trunk_model->jacb_trunk_pos(1,2) = - length_trunk/2;
    trunk_model->jacb_trunk_pos(1,3) = 0;

    trunk_model->jacb_trunk_pos(2,0) = 1;
    trunk_model->jacb_trunk_pos(2,1) = width_trunk/2;
    trunk_model->jacb_trunk_pos(2,2) = length_trunk/2;
    trunk_model->jacb_trunk_pos(2,3) = 0;

    trunk_model->jacb_trunk_pos(3,0) = 1;
    trunk_model->jacb_trunk_pos(3,1) = - width_trunk/2;
    trunk_model->jacb_trunk_pos(3,2) = length_trunk/2;
    trunk_model->jacb_trunk_pos(3,3) = 0;

    // Backward Jacobian: radial of leg to Trunk Angle
    trunk_model->jacb_trunk_pos_inv(0,0) = 0.25;
    trunk_model->jacb_trunk_pos_inv(0,1) = 0.25;
    trunk_model->jacb_trunk_pos_inv(0,2) = 0.25;
    trunk_model->jacb_trunk_pos_inv(0,3) = 0.25;

    trunk_model->jacb_trunk_pos_inv(1,0) = 1/(2*width_trunk);
    trunk_model->jacb_trunk_pos_inv(1,1) = - 1/(2*width_trunk);
    trunk_model->jacb_trunk_pos_inv(1,2) = 1/(2*width_trunk);
    trunk_model->jacb_trunk_pos_inv(1,3) = - 1/(2*width_trunk);

    trunk_model->jacb_trunk_pos_inv(2,0) = - 1/(2*length_trunk);
    trunk_model->jacb_trunk_pos_inv(2,1) = - 1/(2*length_trunk);
    trunk_model->jacb_trunk_pos_inv(2,2) = 1/(2*length_trunk);
    trunk_model->jacb_trunk_pos_inv(2,3) = 1/(2*length_trunk);

    trunk_model->jacb_trunk_pos_inv(3,0) = 1;
    trunk_model->jacb_trunk_pos_inv(3,1) = -1;
    trunk_model->jacb_trunk_pos_inv(3,2) = -1;
    trunk_model->jacb_trunk_pos_inv(3,3) = 1;

    // Velocity Jacobian
    trunk_model->jacb_trunk_vel[0][0] = trunk_model->jacb_trunk_pos(0,0);
    trunk_model->jacb_trunk_vel[0][1] = trunk_model->jacb_trunk_pos(0,1);
    trunk_model->jacb_trunk_vel[0][2] = trunk_model->jacb_trunk_pos(0,2);

    trunk_model->jacb_trunk_vel[1][0] = trunk_model->jacb_trunk_pos(0,0);
    trunk_model->jacb_trunk_vel[1][1] = trunk_model->jacb_trunk_pos(1,1);
    trunk_model->jacb_trunk_vel[1][2] = trunk_model->jacb_trunk_pos(2,2);

    trunk_model->jacb_trunk_vel[2][0] = trunk_model->jacb_trunk_pos(0,0);
    trunk_model->jacb_trunk_vel[2][1] = trunk_model->jacb_trunk_pos(1,1);
    trunk_model->jacb_trunk_vel[2][2] = trunk_model->jacb_trunk_pos(2,2);

    trunk_model->jacb_trunk_vel[3][0] = trunk_model->jacb_trunk_pos(0,0);
    trunk_model->jacb_trunk_vel[3][1] = trunk_model->jacb_trunk_pos(1,1);
    trunk_model->jacb_trunk_vel[3][2] = trunk_model->jacb_trunk_pos(2,2);
};

void bodyKinematics::bwdKinematics_cal(TrunkModel_* trunk_model, StateModel_* FL_model, StateModel_* FR_model, StateModel_* RL_model, StateModel_* RR_model)
{
    trunk_model->r[0] = FL_model->posRW[0];
    trunk_model->r[1] = FR_model->posRW[0];
    trunk_model->r[2] = RL_model->posRW[0];
    trunk_model->r[3] = RR_model->posRW[0];
    //printf("%f, %f, %f, %f \n", trunk_model->r[0], trunk_model->r[1], trunk_model->r[2], trunk_model->r[3]);

    trunk_model->pos_trunk_est = trunk_model->jacb_trunk_pos_inv * trunk_model->r;
    //printf("%f, %f, %f, %f \n", trunk_model->jacb_trunk_pos_inv(0,0), trunk_model->jacb_trunk_pos_inv(0,1), trunk_model->jacb_trunk_pos_inv(0,2), trunk_model->jacb_trunk_pos_inv(0,3));
    //printf("%f, %f, %f \n", trunk_model->pos_trunk_est[0], trunk_model->pos_trunk_est[1], trunk_model->pos_trunk_est[2]);

    trunk_model->rdot[0] = FL_model->velRW[0];
    trunk_model->rdot[1] = FR_model->velRW[0];
    trunk_model->rdot[2] = RL_model->velRW[0];
    trunk_model->rdot[3] = RR_model->velRW[0];

    trunk_model->vel_trunk_est = trunk_model->jacb_trunk_pos_inv * trunk_model->rdot;
};

void bodyKinematics::state_init(const mjModel* m, mjData* d, TrunkModel_* trunk_model)
{
    Quaterniond qdot(d->qvel[3], d->qvel[4], d->qvel[5], d->qvel[6]);
    Quaterniond q(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
    Vector3d eulerdot = ToEulerAngles(qdot);
    Vector3d euler = ToEulerAngles(q);

    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        // Trunk Sensor
        trunk_model->imu_accl[i] = 0;
        trunk_model->imu_accl_old[i] = trunk_model->imu_accl[i];
        trunk_model->imu_gyro[i] = 0;
        trunk_model->imu_gyro_old[i] = trunk_model->imu_gyro[i];

        trunk_model->pos_trunk_act[i] = d->qpos[i];
        trunk_model->pos_trunk_act_old[i] = trunk_model->pos_trunk_act[i];
        trunk_model->pos_trunk_act[i + 3] = euler[i];
        trunk_model->pos_trunk_act_old[i + 3] = trunk_model->pos_trunk_act[i + 3];
        
        trunk_model->vel_trunk_act[i] = d->qvel[i];
        trunk_model->vel_trunk_act_old[i] = trunk_model->vel_trunk_act[i];
        trunk_model->vel_trunk_act[i + 3] = eulerdot[i];
        trunk_model->vel_trunk_act_old[i + 3] = trunk_model->vel_trunk_act[i + 3];
    }

    for (int i = 0; i < NDOF_TRUNK; i++)
    {
        // Trunk
        trunk_model->pos_trunk_des[i] = trunk_model->pos_trunk_act[i];
        trunk_model->pos_trunk_des_old[i] = trunk_model->pos_trunk_des[i];
        trunk_model->pos_trunk_est[i] = 0;
        trunk_model->pos_trunk_est_old[i] = trunk_model->pos_trunk_est[i];

        trunk_model->vel_trunk_des[i] = trunk_model->vel_trunk_act[i];
        trunk_model->vel_trunk_des_old[i] = trunk_model->vel_trunk_des[i];
        trunk_model->vel_trunk_est[i] = 0;
        trunk_model->vel_trunk_est_old[i] = trunk_model->vel_trunk_est[i];

        trunk_model->trunk_pos_ctrl_input[i] = 0;
        trunk_model->trunk_pos_ctrl_input_old[i] = trunk_model->trunk_pos_ctrl_input[i];
        trunk_model->trunk_vel_ctrl_input[i] = 0;
        trunk_model->trunk_vel_ctrl_input_old[i] = trunk_model->trunk_vel_ctrl_input[i];
    }
    trunk_model->pos_trunk_des[0] = 0.3536;
    trunk_model->pos_trunk_des_old[0] = trunk_model->pos_trunk_des[0];

    trunk_model->pos_trunk_des[3] = 0;
    trunk_model->pos_trunk_des_old[3] = trunk_model->pos_trunk_des[3];
    trunk_model->pos_trunk_est[3] = 0;
    trunk_model->pos_trunk_est_old[3] = trunk_model->pos_trunk_est[3];

    trunk_model->vel_trunk_des[3] = 0;
    trunk_model->vel_trunk_des_old[3] = trunk_model->vel_trunk_des[3];
    trunk_model->vel_trunk_est[3] = 0;
    trunk_model->vel_trunk_est_old[3] = trunk_model->vel_trunk_est[3];

    trunk_model->trunk_pos_ctrl_input[3] = 0;
    trunk_model->trunk_pos_ctrl_input_old[3] = trunk_model->trunk_pos_ctrl_input[3];
    trunk_model->trunk_vel_ctrl_input[3] = 0;
    trunk_model->trunk_vel_ctrl_input_old[3] = trunk_model->trunk_vel_ctrl_input[3];
};

Vector3d bodyKinematics::ToEulerAngles(const Quaterniond& q) {
    Vector3d angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = atan2(sinr_cosp, cosr_cosp) + M_PI / 2;

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        angles[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = atan2(siny_cosp, cosy_cosp);
    return angles;
};