#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> //for bool
//#include<unistd.h> //for usl eep
#include <math.h>
#include <iostream>
//#include <resource.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include "bodyController.h"
#include "bodyKinematics.h"
#include "controller.h"
#include "dataLogging.h"
#include "animation.h"
#include "kinematics.h"
#include "trajectory.h"

mjvFigure figPosRW;         // RW position tracking plot
mjvFigure figFOB;           // RWFOB GRF estimation plot
mjvFigure figTrunkState;    // Trunk state plot

double simEndtime = 10;	// Simulation End Time

/***************** State Parameter Class Declaration *****************/
StateModel_ state_Model_FL;
StateModel_ state_Model_FR;
StateModel_ state_Model_RL;
StateModel_ state_Model_RR;
TrunkModel_ state_Model_Trunck; // Newly Added for Jihwan

/***************** Controller Class Declaration *****************/
const int leg_FL_no = 0;
const int leg_FR_no = 3;
const int leg_RL_no = 6;
const int leg_RR_no = 9;

const int leg_FL_sensor_no = 18;
const int leg_FR_sensor_no = 22;
const int leg_RL_sensor_no = 26;
const int leg_RR_sensor_no = 30;

controller ctrl_FL; // other class is in main loop
controller ctrl_FR;
controller ctrl_RL;
controller ctrl_RR;
bodyController ctrl_Trunk; // Newly Added for Jihwan

kinematics kin_FL;
kinematics kin_FR;
kinematics kin_RL;
kinematics kin_RR;
bodyKinematics kin_Trunk; // Newly Added for Jihwan

// trajectory tra_FL;
// trajectory tra_FR;
// trajectory tra_RL;
// trajectory tra_RR;
trajectory tra_Trunk; // Newly Added for Jihwan

/***************** Controller Mode Setting *****************/
double drop_height_setting = 0.7;
string cmd_motion_type = "Pitch";    // flag for switching Robot Motion
int flag_trunk_ff = 1;      // flag for switching ON/OFF Trunk FF control
int flag_trunk_pid = 1;     // flag for switching ON/OFF Trunk PID control
int flag_leg_pos_pid = 1;       // flag for switching ON/OFF Leg POS PID control
int flag_leg_vel_pid = 1;       // flag for switching ON/OFF Leg VEL PID control
int flag_DOB = 1;           // flag for switching ON/OFF RWDOB
int flag_admitt = 1;        // flag for switching ON/OFF admittance control

/***************** Controller Parameter Setting *****************/
double sensor_cutoff = 150;

double trunk_p_gain = 900;
double trunk_d_gain = 3;
double trunk_pd_cutoff = 100;

double omega_n = 700;
double zeta = 20;
double k = 5000;

double leg_pos_p_gain = 500;
double leg_pos_d_gain = 2;
double leg_pos_pd_cutoff = 400;

double leg_vel_p_gain = 100;
double leg_vel_d_gain = 2;
double leg_vel_pd_cutoff = 100;

double leg_DOB_cutoff = 10;

double leg_FOB_cutoff = 10;

/***************** Main Controller *****************/
void mycontroller(const mjModel* m, mjData* d)
{
    double time_run = d->time;
    // Trunk Position Reference Input
    state_Model_Trunck.trunk_pos_ctrl_input = state_Model_Trunck.jacb_trunk_pos * state_Model_Trunck.pos_trunk_des;

    // Trunk pos input to each legs
    state_Model_FL.posRW_ref[0] = state_Model_Trunck.trunk_pos_ctrl_input[0];
    state_Model_FR.posRW_ref[0] = state_Model_Trunck.trunk_pos_ctrl_input[1];
    state_Model_RL.posRW_ref[0] = state_Model_Trunck.trunk_pos_ctrl_input[2];
    state_Model_RR.posRW_ref[0] = state_Model_Trunck.trunk_pos_ctrl_input[3];

    state_Model_FL.posRW_ref[1] = pi / 2;
    state_Model_FR.posRW_ref[1] = pi / 2;
    state_Model_RL.posRW_ref[1] = pi / 2;
    state_Model_RR.posRW_ref[1] = pi / 2;

    // Leg Force Observer
    ctrl_FL.FOBRW(&state_Model_FL, leg_FOB_cutoff); // Rotating Workspace Force Observer (RWFOB)
    ctrl_FR.FOBRW(&state_Model_FR, leg_FOB_cutoff);
    ctrl_RL.FOBRW(&state_Model_RL, leg_FOB_cutoff);
    ctrl_RR.FOBRW(&state_Model_RR, leg_FOB_cutoff);

    // Leg Admittance Control
    ctrl_FL.admittanceCtrl(&state_Model_FL, omega_n, zeta, k, flag_admitt); //parameter(omega_n,zeta,k)
    ctrl_FR.admittanceCtrl(&state_Model_FR, omega_n, zeta, k, flag_admitt);
    ctrl_RL.admittanceCtrl(&state_Model_RL, omega_n, zeta, k, flag_admitt);
    ctrl_RR.admittanceCtrl(&state_Model_RR, omega_n, zeta, k, flag_admitt);

    // Leg Position PID Gain Setting
    ctrl_FL.pid_gain_pos(leg_pos_p_gain, leg_pos_d_gain, leg_pos_pd_cutoff, flag_leg_pos_pid); //(kp,kd,freq)
    ctrl_FR.pid_gain_pos(leg_pos_p_gain, leg_pos_d_gain, leg_pos_pd_cutoff, flag_leg_pos_pid);
    ctrl_RL.pid_gain_pos(leg_pos_p_gain, leg_pos_d_gain, leg_pos_pd_cutoff, flag_leg_pos_pid);
    ctrl_RR.pid_gain_pos(leg_pos_p_gain, leg_pos_d_gain, leg_pos_pd_cutoff, flag_leg_pos_pid);

    // Leg Position PID Control
    state_Model_FL.velRW_ref = ctrl_FL.PID_pos(&state_Model_FL); // RW position feedback
    state_Model_FR.velRW_ref = ctrl_FR.PID_pos(&state_Model_FR);
    state_Model_RL.velRW_ref = ctrl_RL.PID_pos(&state_Model_RL);
    state_Model_RR.velRW_ref = ctrl_RR.PID_pos(&state_Model_RR);

    // Trunk Feedforward control
    state_Model_Trunck.trunk_vel_ctrl_input = ctrl_Trunk.Feedforward_vel(&state_Model_Trunck, flag_trunk_ff);

    // Trunk PID Gain Setting
    ctrl_Trunk.pid_gain_pos(trunk_p_gain, trunk_d_gain, trunk_pd_cutoff, flag_trunk_pid);

    // Trunk PID control
    state_Model_Trunck.trunk_vel_ctrl_input = state_Model_Trunck.trunk_vel_ctrl_input + state_Model_Trunck.jacb_trunk_pos * ctrl_Trunk.PID_pos(&state_Model_Trunck);

    // Trunk vel input to each legs
    state_Model_FL.velRW_ref[0] = state_Model_FL.velRW_ref[0] + state_Model_Trunck.trunk_vel_ctrl_input[0];
    state_Model_FR.velRW_ref[0] = state_Model_FR.velRW_ref[0] + state_Model_Trunck.trunk_vel_ctrl_input[1];
    state_Model_RL.velRW_ref[0] = state_Model_RL.velRW_ref[0] + state_Model_Trunck.trunk_vel_ctrl_input[2];
    state_Model_RR.velRW_ref[0] = state_Model_RR.velRW_ref[0] + state_Model_Trunck.trunk_vel_ctrl_input[3];

    // Leg Velocity PID Gain Setting
    ctrl_FL.pid_gain_vel(leg_vel_p_gain, leg_vel_d_gain, leg_vel_pd_cutoff, flag_leg_vel_pid); //(kp,kd,freq)
    ctrl_FR.pid_gain_vel(leg_vel_p_gain, leg_vel_d_gain, leg_vel_pd_cutoff, flag_leg_vel_pid);
    ctrl_RL.pid_gain_vel(leg_vel_p_gain, leg_vel_d_gain, leg_vel_pd_cutoff, flag_leg_vel_pid);
    ctrl_RR.pid_gain_vel(leg_vel_p_gain, leg_vel_d_gain, leg_vel_pd_cutoff, flag_leg_vel_pid);

    // Leg Velocity PID Control
    state_Model_FL.tau_bi = state_Model_FL.jacbRW_trans * ctrl_FL.PID_vel(&state_Model_FL); // RW position feedback
    state_Model_FR.tau_bi = state_Model_FR.jacbRW_trans * ctrl_FR.PID_vel(&state_Model_FR);
    state_Model_RL.tau_bi = state_Model_RL.jacbRW_trans * ctrl_RL.PID_vel(&state_Model_RL);
    state_Model_RR.tau_bi = state_Model_RR.jacbRW_trans * ctrl_RR.PID_vel(&state_Model_RR);

    // Leg RWDOB control
    state_Model_FL.tau_bi = state_Model_FL.tau_bi + ctrl_FL.DOBRW(&state_Model_FL, leg_DOB_cutoff, flag_DOB);
    state_Model_FR.tau_bi = state_Model_FR.tau_bi + ctrl_FR.DOBRW(&state_Model_FR, leg_DOB_cutoff, flag_DOB);
    state_Model_RL.tau_bi = state_Model_RL.tau_bi + ctrl_RL.DOBRW(&state_Model_RL, leg_DOB_cutoff, flag_DOB);
    state_Model_RR.tau_bi = state_Model_RR.tau_bi + ctrl_RR.DOBRW(&state_Model_RR, leg_DOB_cutoff, flag_DOB);

   // Torque input Biarticular
    d->ctrl[0] = 5000*(0-d->qpos[7]); //FLHAA
    d->ctrl[1] = state_Model_FL.tau_bi[0] + state_Model_FL.tau_bi[1] ;
    d->ctrl[2] = state_Model_FL.tau_bi[1];

    d->ctrl[3] = 5000*(0-d->qpos[10]); //FRHAA
    d->ctrl[4] = state_Model_FR.tau_bi[0] + state_Model_FR.tau_bi[1] ;
    d->ctrl[5] = state_Model_FR.tau_bi[1];

    d->ctrl[6] = 5000*(0-d->qpos[13]); //RLHAA
    d->ctrl[7] = state_Model_RL.tau_bi[0] + state_Model_RL.tau_bi[1] ;
    d->ctrl[8] = state_Model_RL.tau_bi[1];

    d->ctrl[9] = 5000*(0-d->qpos[16]); //FLHAA
    d->ctrl[10] = state_Model_RR.tau_bi[0] + state_Model_RR.tau_bi[1] ;
    d->ctrl[11] = state_Model_RR.tau_bi[1];

    // Data Logging
    if (loop_index % data_frequency == 0) {
        save_data_leg(m, d, &state_Model_FL, ctrl_FL.Data_Return(ctrl_FL), fid_FL, leg_FL_sensor_no);
        save_data_leg(m, d, &state_Model_FR, ctrl_FR.Data_Return(ctrl_FR), fid_FR, leg_FR_sensor_no);
        save_data_leg(m, d, &state_Model_RL, ctrl_RL.Data_Return(ctrl_RL), fid_RL, leg_RL_sensor_no);
        save_data_leg(m, d, &state_Model_RR, ctrl_RR.Data_Return(ctrl_RR), fid_RR, leg_RR_sensor_no);
        save_data_trunk(m, d, &state_Model_Trunck, fid_Trunk);
    }
    loop_index += 1;
}


/***************** Main Function *****************/
int main(int argc, const char** argv)
{
    // activate software
    mj_activate("mjkey.txt");

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if (!m)
        mju_error_s("Load model error: %s", error);


    // make data
    d = mj_makeData(m);

    // Initialize GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-88.95, -17.5, 1.8, 0.04, 0.000000, 0.27};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    fid_FL = fopen(datapath_FL, "w");
    fid_FR = fopen(datapath_FR, "w");
    fid_RL = fopen(datapath_RL, "w");
    fid_RR = fopen(datapath_RR, "w");
    fid_Trunk = fopen(datapath_Trunk, "w");

    init_save_data_leg(fid_FL);
    init_save_data_leg(fid_FR);
    init_save_data_leg(fid_RL);
    init_save_data_leg (fid_RR);
    init_save_data_trunk(fid_Trunk);


    // Initialization

    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = drop_height_setting;   // qpos[0,1,2] : trunk pos
                        // qpos[3,4,5.6] : trunk orientation quaternian
                        // default: 0.3536 m

    d->qpos[3] = -0.73;
    d->qpos[4] = 0.73;
    d->qpos[5] = 0;
    d->qpos[6] = 0;

    d->qpos[7] = 0; //FLHAA         //d->ctrl[0] FLHAA
    d->qpos[8] = pi/6; //FLHIP       //d->ctrl[1] FLHIP
    d->qpos[9] = pi/3; //FLKNEE        //d->ctrl[2] FLKNEE
    d->qpos[10] = 0; //FRHAA        //d->ctrl[3] FRHAA
    d->qpos[11] = pi/6; //FRHIP        //d->ctrl[4] FRHIP
    d->qpos[12] = pi/3; //FRKNEE       //d->ctrl[5] FRKNEE
    d->qpos[13] = 0; //RLHAA        //d->ctrl[6] RLHAA
    d->qpos[14] = pi/6; //RLHIP        //d->ctrl[7] RLHIP
    d->qpos[15] = pi/3; //RLKNEE       //d->ctrl[8] RLKNEE
    d->qpos[16] = 0; //RRHAA        //d->ctrl[9] RRHAA
    d->qpos[17] = pi/6; //RRHIP        //d->ctrl[10] RRHIP
    d->qpos[18] = pi/3; //RRKNEE       //d->ctrl[11] RRKNEE


    kin_FL.model_param_cal(m, d, &state_Model_FL); // state init is before. Caution Error.
    kin_FR.model_param_cal(m, d, &state_Model_FR);
    kin_RL.model_param_cal(m, d, &state_Model_RL);
    kin_RR.model_param_cal(m, d, &state_Model_RR);
    kin_Trunk.model_param_cal(m, d, &state_Model_Trunck);

    kin_FL.state_init(m,d, &state_Model_FL);
    kin_FR.state_init(m,d, &state_Model_FR);
    kin_RL.state_init(m,d, &state_Model_RL);
    kin_RR.state_init(m,d, &state_Model_RR);
    kin_Trunk.state_init(m,d, &state_Model_Trunck);

    // custom controller
    mjcb_control = mycontroller;


    /***************** Simulation Loop *****************/
    // use the first while condition if you want to simulate for a period.
    int i = 0;
    while (!glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        // Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        //printf(" %f  %f \n", d->ctrl[0], d->ctrl[1]);
        state_Model_FL.time = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            kin_FL.sensor_measure(m, d, &state_Model_FL, sensor_cutoff, leg_FL_no); // get joint sensor data & calculate biarticular angles
            kin_FR.sensor_measure(m, d, &state_Model_FR, sensor_cutoff, leg_FR_no);
            kin_RL.sensor_measure(m, d, &state_Model_RL, sensor_cutoff, leg_RL_no);
            kin_RR.sensor_measure(m, d, &state_Model_RR, sensor_cutoff, leg_RR_no);
            kin_Trunk.sensor_measure(m, d, &state_Model_Trunck);

            kin_FL.model_param_cal(m, d,&state_Model_FL); // calculate model parameters
            kin_FR.model_param_cal(m, d,&state_Model_FR);
            kin_RL.model_param_cal(m, d,&state_Model_RL);
            kin_RR.model_param_cal(m, d,&state_Model_RR);
            kin_Trunk.model_param_cal(m, d,&state_Model_Trunck);

            kin_FL.jacobianRW(&state_Model_FL);
            kin_FR.jacobianRW(&state_Model_FR);
            kin_RL.jacobianRW(&state_Model_RL);
            kin_RR.jacobianRW(&state_Model_RR);            // calculate RW Jacobian
            kin_Trunk.jacobianTrunk(&state_Model_Trunck);

            if (d->time < 10)
            {
                //printf("ref: %f \n", state_Model_FL.posRW_ref[1]);
                //printf(" qddot_bi_tustin(0) = %f ,%f ", state_Model_FL.qddot_bi_tustin[0],state_Model_FL.qddot_bi_tustin[1]);
                //printf(" qddot_bi(1) = %f ,%f \n", state_Model_FL.qddot_bi[0],state_Model_FL.qddot_bi[1]);

            }

            kin_FL.fwdKinematics_cal(&state_Model_FL);     // calculate RW Kinematics
            kin_FR.fwdKinematics_cal(&state_Model_FR);
            kin_RL.fwdKinematics_cal(&state_Model_RL);
            kin_RR.fwdKinematics_cal(&state_Model_RR);
            kin_Trunk.bwdKinematics_cal(&state_Model_Trunck, &state_Model_FL, &state_Model_FR, &state_Model_RL, &state_Model_RR);

            /* Trajectory Generation */
            if (cmd_motion_type == "Squat")   // Squat
            {
                tra_Trunk.Squat(d->time, &state_Model_Trunck);
                //tra_FL.Squat(d->time, &state_Model_FL);
                //tra_FR.Squat(d->time, &state_Model_FR);
                //tra_RL.Squat(d->time, &state_Model_RL);
                //tra_RR.Squat(d->time, &state_Model_RR);
            }
            else if (cmd_motion_type == "Roll") // Roll
            {
                tra_Trunk.Roll(d->time, &state_Model_Trunck);
            }
            else if (cmd_motion_type == "Pitch") // Roll
            {
                tra_Trunk.Pitch(d->time, &state_Model_Trunck);
            }
            else // Hold stance
            {
                tra_Trunk.Hold(&state_Model_Trunck);
                //tra_FL.Hold(&state_Model_FL);
                //tra_FR.Hold(&state_Model_FR);
                //tra_RL.Hold(&state_Model_RL);
                //tra_RR.Hold(&state_Model_RR);
            }
            mj_step(m, d);

            kin_FL.state_update(&state_Model_FL);
            kin_FR.state_update(&state_Model_FR);
            kin_RL.state_update(&state_Model_RL);
            kin_RR.state_update(&state_Model_RR);
            kin_Trunk.state_update(&state_Model_Trunck);

            ctrl_FL.ctrl_update();
            ctrl_FR.ctrl_update();
            ctrl_RL.ctrl_update();
            ctrl_RR.ctrl_update();
            //ctrl_Trunk.ctrl_update();
        }

        if (d->time >= simEndtime) {
            fclose(fid_FL);
            fclose(fid_FR);
            fclose(fid_RL);
            fclose(fid_RR);
            fclose(fid_Trunk);

        }
        //printf("%f \n", state_Model_FL.deltaPos[0]);
        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        //opt.frame = mjFRAME_WORLD;
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = 0;
        //cam.lookat[2] = 0;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 1;
}
