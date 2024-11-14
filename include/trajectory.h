#ifndef __TRAJECTORY_H__
#define	__TRAJECTORY_H__

#include "globVariable.h"

class trajectory
{
    private:
        // Squat Parameter
        double squat_T_pause;
        double freq_squat;
        double squat_r0;
        double squat_rc;

        // Jumping Parameter
        double K_thrust;
        double zeta_thrust;
        double K_land;
        double zeta_land;

        double jump_r0;
        double jump_rc;
        double jump_rt;

        double jump_T_stand;
        double jump_T_crouch;
        double jump_T_pause;
        double jump_T_land;
        double jump_T_recover;
        double jump_qd_max;

        // Roll Parameter
        double roll_T_pause;
        double freq_roll;
        double roll_r0;
        double roll_rc;

        // Pitch Parameter
        double pitch_T_pause;
        double freq_pitch;
        double pitch_r0;
        double pitch_rc;

    public:
        trajectory();
        ~trajectory();
        // For trunk input
        void Squat(double t, TrunkModel_* trunk_model);
        void Roll(double t, TrunkModel_* trunk_model);
        void Pitch(double t, TrunkModel_* trunk_model);
        void Hold(TrunkModel_* trunk_model);
        /*
        // For direct leg input
        void Squat(double t,StateModel_* state_model);
        void Jumping(double t, StateModel_* state_model, int mode_admitt);
        void Hold(StateModel_* state_model);
        */


};



#endif // !__TRAJECTORY_H__
