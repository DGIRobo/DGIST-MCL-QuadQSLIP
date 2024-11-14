#ifndef __BODYKINEMATICS_H__
#define __BODYKINEMATICS_H__

#include "globVariable.h"
#include "filter.h"
#include <mujoco/mujoco.h> //Caution Crash


class bodyKinematics
{

private:   
    /* Trunk Parameter */
    double m_hip;         // mass of hip torso
    double m_trunk_front; // mass of front trunk
    double m_trunk_rear;  // mass of rear trunk
    double m_trunk;       // total mass of trunk
    double m_total;       // total robot mass

    /* Trunk Dimension */
    double width_trunk;
    double length_trunk;
    double length_front;
    double length_rear;
    double height_trunk;

public:
    bodyKinematics(); //생성자
    ~bodyKinematics(); //소멸자
    
    void state_update(TrunkModel_* trunk_model);
    void model_param_cal(const mjModel* m, mjData* d, TrunkModel_* trunk_model);
    void sensor_measure(const mjModel* m, mjData* d, TrunkModel_* trunk_model);
    void jacobianTrunk(TrunkModel_* trunk_model);
    void bwdKinematics_cal(TrunkModel_* trunk_model, StateModel_* FL_model, StateModel_* FR_model, StateModel_* RL_model, StateModel_* RR_model);
    void state_init(const mjModel* m, mjData* d, TrunkModel_* trunk_model);
    Vector3d ToEulerAngles(const Quaterniond& q);
};

#endif // !__BODYKINEMATICS_H__

