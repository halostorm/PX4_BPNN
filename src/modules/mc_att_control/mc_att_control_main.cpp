/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <uORB/topics/vehicle_gps_position.h>

#include<string>
#include<time.h>
/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

///////////////////////////////////////////////////////////////////////
#define sample_number  50 //样本数量
#define error_min  0.01 //误差小于该值则停止学习
#define error_max  0.1 //误差大于该值则启动学习

int train_count = 0; //样本计算次数
int begin_count = 10000; //初始控制次数
bool neure_flag_enabled = false; //flag of using NN;由遥控器选择
bool unstable_flag_enabled = false; //flag of using NN；误差不稳定则学习，由error_min、error_max决定
double param[12] = { 0.0 };
double param1[3] = { 0.0 };

FILE *fd = NULL; //输出pid神经元参数变化；
FILE *fe = NULL; //输出 pid error变化；
//static uint64_t gps_time_sec = 0;

bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle,
        void *buffer);

//////////////////////////
class MulticopterAttitudeControl {
public:
    /**
     * Constructor
     */
    MulticopterAttitudeControl();

    /**
     * Destructor, also kills the main task
     */
    ~MulticopterAttitudeControl();

    /**
     * Start the multicopter attitude control task.
     *
     * @return		OK on success.
     */
    int start();

private:

    bool _task_should_exit; /**< if true, task_main() should exit */
    int _control_task; /**< task handle */

    int _ctrl_state_sub; /**< control state subscription */
    int _v_att_sp_sub; /**< vehicle attitude setpoint subscription */
    int _v_rates_sp_sub; /**< vehicle rates setpoint subscription */
    int _v_control_mode_sub; /**< vehicle control mode subscription */
    int _params_sub; /**< parameter updates subscription */
    int _manual_control_sp_sub; /**< manual control setpoint subscription */
    int _armed_sub; /**< arming status subscription */
    int _vehicle_status_sub; /**< vehicle status subscription */
    int _motor_limits_sub; /**< motor limits subscription */

    orb_advert_t _v_rates_sp_pub; /**< rate setpoint publication */
    orb_advert_t _actuators_0_pub; /**< attitude actuator controls publication */
    orb_advert_t _controller_status_pub; /**< controller status publication */

    orb_id_t _rates_sp_id; /**< pointer to correct rates setpoint uORB metadata structure */
    orb_id_t _actuators_id; /**< pointer to correct actuator controls0 uORB metadata structure */

    bool _actuators_0_circuit_breaker_enabled; /**< circuit breaker to suppress output */

    struct control_state_s _ctrl_state; /**< control state */
    struct vehicle_attitude_setpoint_s _v_att_sp; /**< vehicle attitude setpoint */
    struct vehicle_rates_setpoint_s _v_rates_sp; /**< vehicle rates setpoint */
    struct manual_control_setpoint_s _manual_control_sp; /**< manual control setpoint */
    struct vehicle_control_mode_s _v_control_mode; /**< vehicle control mode */
    struct actuator_controls_s _actuators; /**< actuator controls */
    struct actuator_armed_s _armed; /**< actuator arming status */
    struct vehicle_status_s _vehicle_status; /**< vehicle status */
    struct multirotor_motor_limits_s _motor_limits; /**< motor limits */
    struct mc_att_ctrl_status_s _controller_status; /**< controller status */

    perf_counter_t _loop_perf; /**< loop performance counter */
    perf_counter_t _controller_latency_perf;

    math::Vector<3> _rates_prev; /**< angular rates on previous step */
    math::Vector<3> _rates_sp_prev; /**< previous rates setpoint */
    math::Vector<3> _rates_sp; /**< angular rates setpoint */
    math::Vector<3> _rates_int; /**< angular rates integral error */
    float _thrust_sp; /**< thrust setpoint */
    math::Vector<3> _att_control; /**< attitude control vector */

    math::Matrix<3, 3> _I; /**< identity matrix */

    struct {
        param_t roll_p;
        param_t roll_rate_p;
        param_t roll_rate_i;
        param_t roll_rate_d;
        param_t roll_rate_ff;
        param_t pitch_p;
        param_t pitch_rate_p;
        param_t pitch_rate_i;
        param_t pitch_rate_d;
        param_t pitch_rate_ff;
        param_t tpa_breakpoint;
        param_t tpa_slope;
        param_t yaw_p;
        param_t yaw_rate_p;
        param_t yaw_rate_i;
        param_t yaw_rate_d;
        param_t yaw_rate_ff;
        param_t yaw_ff;
        param_t roll_rate_max;
        param_t pitch_rate_max;
        param_t yaw_rate_max;
        param_t yaw_auto_max;

        param_t acro_roll_max;
        param_t acro_pitch_max;
        param_t acro_yaw_max;
        param_t rattitude_thres;

        param_t vtol_type;
        param_t roll_tc;
        param_t pitch_tc;
        param_t vtol_opt_recovery_enabled;
        param_t vtol_wv_yaw_rate_scale;

    } _params_handles; /**< handles for interesting parameters */

    struct {
        math::Vector<3> att_p; /**< P gain for angular error */
        math::Vector<3> rate_p; /**< P gain for angular rate error */
        math::Vector<3> rate_i; /**< I gain for angular rate error */
        math::Vector<3> rate_d; /**< D gain for angular rate error */
        math::Vector<3> rate_ff; /**< Feedforward gain for desired rates */
        float yaw_ff; /**< yaw control feed-forward */

        float tpa_breakpoint; /**< Throttle PID Attenuation breakpoint */
        float tpa_slope; /**< Throttle PID Attenuation slope */

        float roll_rate_max;
        float pitch_rate_max;
        float yaw_rate_max;
        float yaw_auto_max;
        math::Vector<3> mc_rate_max; /**< attitude rate limits in stabilized modes */
        math::Vector<3> auto_rate_max; /**< attitude rate limits in auto modes */
        math::Vector<3> acro_rate_max; /**< max attitude rates in acro mode */
        float rattitude_thres;
        int vtol_type; /**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
        bool vtol_opt_recovery_enabled;
        float vtol_wv_yaw_rate_scale; /**< Scale value [0, 1] for yaw rate setpoint  */
    } _params;

    struct {
        //BNN_Part_params///////////////////////////////////////////////////////////////////////////////////////////////////////

        math::Vector<3> u1_1, u1_2; //输入层
        math::Vector<3> x1_1, x1_2;

        math::Vector<3> u2_1, u2_2, u2_3; //隐藏层
        math::Vector<3> x2_1, x2_2, x2_3;

        math::Vector<3> u2_1_prev, u2_2_prev, u2_3_prev;
        math::Vector<3> x2_1_prev, x2_2_prev, x2_3_prev;

        math::Vector<3> u3; //输出层
        math::Vector<3> x3;

        math::Vector<3> w1_1, w1_2, w1_3; //in-hide权值
        math::Vector<3> w2_1, w2_2, w2_3;

        math::Vector<3> deta_w1_1, deta_w1_2, deta_w1_3; //in-hide权值改变量
        math::Vector<3> deta_w2_1, deta_w2_2, deta_w2_3;

        math::Vector<3> w21, w22, w23; //hide-out权值，相当于PID参数

        math::Vector<3> deta_w21, deta_w22, deta_w23;

        math::Vector<3> v, r, y, y_;
        math::Vector<3> v_prev, r_prev, y_prev, y_prev_;

        math::Vector<3> deta1;
        math::Vector<3> deta2_1, deta2_2, deta2_3;

        math::Vector<3> Error;

        float study_speed_h_o1, study_speed_h_o2, study_speed_h_o3;

        float study_speed_i_h11, study_speed_i_h12, study_speed_i_h13,
                study_speed_i_h21, study_speed_i_h22, study_speed_i_h23;
        //END///////////////////////////////////
    } n_n; //神经元算法参数

    TailsitterRecovery *_ts_opt_recovery; /**< Computes optimal rates for tailsitter recovery */

    /**
     * Update our local parameter cache.
     */
    int parameters_update();

    /**
     * Check for parameter update and handle it.
     */
    void parameter_update_poll();

    /**
     * Check for changes in vehicle control mode.
     */
    void vehicle_control_mode_poll();

    /**
     * Check for changes in manual inputs.
     */
    void vehicle_manual_poll();

    /**
     * Check for attitude setpoint updates.
     */
    void vehicle_attitude_setpoint_poll();

    /**
     * Check for rates setpoint updates.
     */
    void vehicle_rates_setpoint_poll();

    /**
     * Check for arming status updates.
     */
    void arming_status_poll();

    /**
     * Attitude controller.
     */
    void control_attitude(float dt);

    /**
     * Attitude rates controller.
     */
    void control_attitude_rates(float dt);

    void control_attitude_rates_1(float dt);

    /**
     * Check for vehicle status updates.
     */
    void vehicle_status_poll();

    /**
     * Check for vehicle motor limits status.
     */
    void vehicle_motor_limits_poll();

    /**
     * Shim for calling task_main from task_create.
     */
    static void task_main_trampoline(int argc, char *argv[]);

    /**
     * Main attitude control task.
     */
    void task_main();

    math::Vector<3> Limit(math::Vector<3> x);

    float sign(float x);
    int compare_vector(float x, float y); //如果x与y出现相等的元素，返回0
};

namespace mc_att_control {

MulticopterAttitudeControl *g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

        _task_should_exit(false), _control_task(-1),

        /* subscriptions */
        _ctrl_state_sub(-1), _v_att_sp_sub(-1), _v_control_mode_sub(-1), _params_sub(
                -1), _manual_control_sp_sub(-1), _armed_sub(-1), _vehicle_status_sub(
                -1),

        /* publications */
        _v_rates_sp_pub(nullptr), _actuators_0_pub(nullptr), _controller_status_pub(
                nullptr), _rates_sp_id(0), _actuators_id(0),

        _actuators_0_circuit_breaker_enabled(false),

        /* performance counters */
        _loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")), _controller_latency_perf(
                perf_alloc_once(PC_ELAPSED, "ctrl_latency")), _ts_opt_recovery(
                nullptr)

{
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    memset(&_v_att_sp, 0, sizeof(_v_att_sp));
    memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
    memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
    memset(&_v_control_mode, 0, sizeof(_v_control_mode));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_armed, 0, sizeof(_armed));
    memset(&_vehicle_status, 0, sizeof(_vehicle_status));
    memset(&_motor_limits, 0, sizeof(_motor_limits));
    memset(&_controller_status, 0, sizeof(_controller_status));
    memset(&n_n, 0, sizeof(n_n));
    _vehicle_status.is_rotary_wing = true;

    //NN变量初始化
    //in-hide权值初始化

    n_n.w1_1(0) = 1.0;
    n_n.w1_1(1) = 1.0;
    n_n.w1_1(2) = 1.0;
    n_n.w1_2(0) = 1.0;
    n_n.w1_2(1) = 1.0;
    n_n.w1_2(2) = 1.0;
    n_n.w1_3(0) = 1.0;
    n_n.w1_3(1) = 1.0;
    n_n.w1_3(2) = 1.0;
    n_n.w2_1(0) = -1.0;
    n_n.w2_1(1) = -1.0;
    n_n.w2_1(2) = -1.0;
    n_n.w2_2(0) = -1.0;
    n_n.w2_2(1) = -1.0;
    n_n.w2_2(2) = -1.0;
    n_n.w2_3(0) = -1.0;
    n_n.w2_3(1) = -1.0;
    n_n.w2_3(2) = -1.0;

    /*
     n_n.u2_1_prev.zero();
     n_n.u2_2_prev.zero();
     n_n.u2_3_prev.zero();
     n_n.x2_1_prev.zero();
     n_n.x2_2_prev.zero();
     n_n.x2_3_prev.zero();
     */
    ///END
    _params.att_p.zero();
    _params.rate_p.zero();
    _params.rate_i.zero();
    _params.rate_d.zero();
    _params.rate_ff.zero();
    _params.yaw_ff = 0.0f;
    _params.roll_rate_max = 0.0f;
    _params.pitch_rate_max = 0.0f;
    _params.yaw_rate_max = 0.0f;
    _params.mc_rate_max.zero();
    _params.auto_rate_max.zero();
    _params.acro_rate_max.zero();
    _params.rattitude_thres = 1.0f;
    _params.vtol_opt_recovery_enabled = false;
    _params.vtol_wv_yaw_rate_scale = 1.0f;

    _rates_prev.zero();
    _rates_sp.zero();
    _rates_sp_prev.zero();
    _rates_int.zero();
    _thrust_sp = 0.0f;
    _att_control.zero();

    _I.identity();

    _params_handles.roll_p = param_find("MC_ROLL_P");
    _params_handles.roll_rate_p = param_find("MC_ROLLRATE_P");
    _params_handles.roll_rate_i = param_find("MC_ROLLRATE_I");
    _params_handles.roll_rate_d = param_find("MC_ROLLRATE_D");
    _params_handles.roll_rate_ff = param_find("MC_ROLLRATE_FF");
    _params_handles.pitch_p = param_find("MC_PITCH_P");
    _params_handles.pitch_rate_p = param_find("MC_PITCHRATE_P");
    _params_handles.pitch_rate_i = param_find("MC_PITCHRATE_I");
    _params_handles.pitch_rate_d = param_find("MC_PITCHRATE_D");
    _params_handles.pitch_rate_ff = param_find("MC_PITCHRATE_FF");
    _params_handles.tpa_breakpoint = param_find("MC_TPA_BREAK");
    _params_handles.tpa_slope = param_find("MC_TPA_SLOPE");
    _params_handles.yaw_p = param_find("MC_YAW_P");
    _params_handles.yaw_rate_p = param_find("MC_YAWRATE_P");
    _params_handles.yaw_rate_i = param_find("MC_YAWRATE_I");
    _params_handles.yaw_rate_d = param_find("MC_YAWRATE_D");
    _params_handles.yaw_rate_ff = param_find("MC_YAWRATE_FF");
    _params_handles.yaw_ff = param_find("MC_YAW_FF");
    _params_handles.roll_rate_max = param_find("MC_ROLLRATE_MAX");
    _params_handles.pitch_rate_max = param_find("MC_PITCHRATE_MAX");
    _params_handles.yaw_rate_max = param_find("MC_YAWRATE_MAX");
    _params_handles.yaw_auto_max = param_find("MC_YAWRAUTO_MAX");
    _params_handles.acro_roll_max = param_find("MC_ACRO_R_MAX");
    _params_handles.acro_pitch_max = param_find("MC_ACRO_P_MAX");
    _params_handles.acro_yaw_max = param_find("MC_ACRO_Y_MAX");
    _params_handles.rattitude_thres = param_find("MC_RATT_TH");
    _params_handles.vtol_type = param_find("VT_TYPE");
    _params_handles.roll_tc = param_find("MC_ROLL_TC");
    _params_handles.pitch_tc = param_find("MC_PITCH_TC");
    _params_handles.vtol_opt_recovery_enabled = param_find("VT_OPT_RECOV_EN");
    _params_handles.vtol_wv_yaw_rate_scale = param_find("VT_WV_YAWR_SCL");

    /* fetch initial parameter values */
    parameters_update();

    if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
        // the vehicle is a tailsitter, use optimal recovery control strategy
        _ts_opt_recovery = new TailsitterRecovery();
    }

}

MulticopterAttitudeControl::~MulticopterAttitudeControl() {
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    if (_ts_opt_recovery != nullptr) {
        delete _ts_opt_recovery;
    }

    mc_att_control::g_control = nullptr;
}

math::Vector<3> MulticopterAttitudeControl::Limit(math::Vector<3> x) {
    for (int i = 0; i < 3; i++) {
        if (x(i) > 1)
            x(i) = 1;
        else if (x(i) < -1)
            x(i) = -1;
        else
            continue;
    }
    return x;

}

float MulticopterAttitudeControl::sign(float x) {

    //x = (2.0/(1+exp(-x))) -1;
    if (x > 1)
        x = 1;
    else if (x < -1)
        x = -1;
    return x;
}

int MulticopterAttitudeControl::compare_vector(float x, float y) {

    if ((double)fabs(x - y) < 1e-6)
        return 0;
    return 1;
}

int MulticopterAttitudeControl::parameters_update() {
    float v;

    float roll_tc, pitch_tc;

    param_get(_params_handles.roll_tc, &roll_tc);
    param_get(_params_handles.pitch_tc, &pitch_tc);

    /* roll gains */
    param_get(_params_handles.roll_p, &v);
    _params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
    param_get(_params_handles.roll_rate_p, &v);
    _params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
    param_get(_params_handles.roll_rate_i, &v);
    _params.rate_i(0) = v;
    param_get(_params_handles.roll_rate_d, &v);
    _params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
    param_get(_params_handles.roll_rate_ff, &v);
    _params.rate_ff(0) = v;

    /* pitch gains */
    param_get(_params_handles.pitch_p, &v);
    _params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
    param_get(_params_handles.pitch_rate_p, &v);
    _params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
    param_get(_params_handles.pitch_rate_i, &v);
    _params.rate_i(1) = v;
    param_get(_params_handles.pitch_rate_d, &v);
    _params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
    param_get(_params_handles.pitch_rate_ff, &v);
    _params.rate_ff(1) = v;

    param_get(_params_handles.tpa_breakpoint, &v);
    _params.tpa_breakpoint = v;
    param_get(_params_handles.tpa_slope, &v);
    _params.tpa_slope = v;

    /* yaw gains */
    param_get(_params_handles.yaw_p, &v);
    _params.att_p(2) = v;
    param_get(_params_handles.yaw_rate_p, &v);
    _params.rate_p(2) = v;
    param_get(_params_handles.yaw_rate_i, &v);
    _params.rate_i(2) = v;
    param_get(_params_handles.yaw_rate_d, &v);
    _params.rate_d(2) = v;
    param_get(_params_handles.yaw_rate_ff, &v);
    _params.rate_ff(2) = v;

    param_get(_params_handles.yaw_ff, &_params.yaw_ff);

    /* angular rate limits */
    param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
    _params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
    param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
    _params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
    param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
    _params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

    /* auto angular rate limits */
    param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
    _params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
    param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
    _params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
    param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
    _params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

    /* manual rate control scale and auto mode roll/pitch rate limits */
    param_get(_params_handles.acro_roll_max, &v);
    _params.acro_rate_max(0) = math::radians(v);
    param_get(_params_handles.acro_pitch_max, &v);
    _params.acro_rate_max(1) = math::radians(v);
    param_get(_params_handles.acro_yaw_max, &v);
    _params.acro_rate_max(2) = math::radians(v);

    /* stick deflection needed in rattitude mode to control rates not angles */
    param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

    param_get(_params_handles.vtol_type, &_params.vtol_type);

    int tmp;
    param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
    _params.vtol_opt_recovery_enabled = (bool) tmp;

    param_get(_params_handles.vtol_wv_yaw_rate_scale,
            &_params.vtol_wv_yaw_rate_scale);

    _actuators_0_circuit_breaker_enabled = circuit_breaker_enabled(
            "CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

    return OK;
}

void MulticopterAttitudeControl::parameter_update_poll() {
    bool updated;

    /* Check if parameters have changed */
    orb_check(_params_sub, &updated);

    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
        parameters_update();
    }
}

void MulticopterAttitudeControl::vehicle_control_mode_poll() {
    bool updated;

    /* Check if vehicle control mode has changed */
    orb_check(_v_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub,
                &_v_control_mode);
    }
}

void MulticopterAttitudeControl::vehicle_manual_poll() {
    bool updated;

    /* get pilots inputs */
    orb_check(_manual_control_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub,
                &_manual_control_sp);
    }
}

void MulticopterAttitudeControl::vehicle_attitude_setpoint_poll() {
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_att_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
    }
}

void MulticopterAttitudeControl::vehicle_rates_setpoint_poll() {
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_rates_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
    }
}

void MulticopterAttitudeControl::arming_status_poll() {
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_armed_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
    }
}

void MulticopterAttitudeControl::vehicle_status_poll() {
    /* check if there is new status information */
    bool vehicle_status_updated;
    orb_check(_vehicle_status_sub, &vehicle_status_updated);

    if (vehicle_status_updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

        /* set correct uORB ID, depending on if vehicle is VTOL or not */
        if (!_rates_sp_id) {
            if (_vehicle_status.is_vtol) {
                _rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
                _actuators_id = ORB_ID(actuator_controls_virtual_mc);

            } else {
                _rates_sp_id = ORB_ID(vehicle_rates_setpoint);
                _actuators_id = ORB_ID(actuator_controls_0);
            }
        }
    }
    if (_vehicle_status.nav_state == 15) {
        neure_flag_enabled = true;
    } else if (_vehicle_status.nav_state == 0) {
        neure_flag_enabled = false;
    }
}

void MulticopterAttitudeControl::vehicle_motor_limits_poll() {
    /* check if there is a new message */
    bool updated;
    orb_check(_motor_limits_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub,
                &_motor_limits);
    }
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void MulticopterAttitudeControl::control_attitude(float dt) {
    vehicle_attitude_setpoint_poll();

    _thrust_sp = _v_att_sp.thrust;

    /* construct attitude setpoint rotation matrix */
    math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2],
            _v_att_sp.q_d[3]);
    math::Matrix<3, 3> R_sp = q_sp.to_dcm();

    /* get current rotation matrix from control state quaternions */
    math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2],
            _ctrl_state.q[3]);
    math::Matrix<3, 3> R = q_att.to_dcm();

    /* all input data is ready, run controller itself */

    /* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
    math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
    math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

    /* axis and sin(angle) of desired rotation */
    math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

    /* calculate angle error */
    float e_R_z_sin = e_R.length();
    float e_R_z_cos = R_z * R_sp_z;

    /* calculate weight for yaw control */
    float yaw_w = R_sp(2, 2) * R_sp(2, 2);

    /* calculate rotation matrix after roll/pitch only rotation */
    math::Matrix<3, 3> R_rp;

    if (e_R_z_sin > 0.0f) {
        /* get axis-angle representation */
        float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
        math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

        e_R = e_R_z_axis * e_R_z_angle;

        /* cross product matrix for e_R_axis */
        math::Matrix<3, 3> e_R_cp;
        e_R_cp.zero();
        e_R_cp(0, 1) = -e_R_z_axis(2);
        e_R_cp(0, 2) = e_R_z_axis(1);
        e_R_cp(1, 0) = e_R_z_axis(2);
        e_R_cp(1, 2) = -e_R_z_axis(0);
        e_R_cp(2, 0) = -e_R_z_axis(1);
        e_R_cp(2, 1) = e_R_z_axis(0);

        /* rotation matrix for roll/pitch only rotation */
        R_rp = R
                * (_I + e_R_cp * e_R_z_sin
                        + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

    } else {
        /* zero roll/pitch rotation */
        R_rp = R;
    }

    /* R_rp and R_sp has the same Z axis, calculate yaw error */
    math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
    math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
    e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

    if (e_R_z_cos < 0.0f) {
        /* for large thrust vector rotations use another rotation method:
         * calculate angle and axis for R -> R_sp rotation directly */
        math::Quaternion q_error;
        q_error.from_dcm(R.transposed() * R_sp);
        math::Vector<3> e_R_d =
                q_error(0) >= 0.0f ?
                        q_error.imag() * 2.0f : -q_error.imag() * 2.0f;

        /* use fusion of Z axis based rotation and direct rotation */
        float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
        e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
    }

    /* calculate angular rates setpoint */
    _rates_sp = _params.att_p.emult(e_R);

    /* limit rates */
    for (int i = 0; i < 3; i++) {
        if ((_v_control_mode.flag_control_velocity_enabled
                || _v_control_mode.flag_control_auto_enabled)
                && !_v_control_mode.flag_control_manual_enabled) {
            _rates_sp(i) = math::constrain(_rates_sp(i),
                    -_params.auto_rate_max(i), _params.auto_rate_max(i));

        } else {
            _rates_sp(i) = math::constrain(_rates_sp(i),
                    -_params.mc_rate_max(i), _params.mc_rate_max(i));
        }
    }

    /* feed forward yaw setpoint rate */
    _rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

    /* weather-vane mode, dampen yaw rate */
    if ((_v_control_mode.flag_control_velocity_enabled
            || _v_control_mode.flag_control_auto_enabled)
            && _v_att_sp.disable_mc_yaw_control == true
            && !_v_control_mode.flag_control_manual_enabled) {
        float wv_yaw_rate_max = _params.auto_rate_max(2)
                * _params.vtol_wv_yaw_rate_scale;
        _rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max,
                wv_yaw_rate_max);
        // prevent integrator winding up in weathervane mode
        _rates_int(2) = 0.0f;
    }
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
/*
 void MulticopterAttitudeControl::control_attitude_rates(float dt) {
 // reset integral if disarmed
 if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
 _rates_int.zero();
 }

 //current body angular rates
 math::Vector<3> rates;
 rates(0) = _ctrl_state.roll_rate;
 rates(1) = _ctrl_state.pitch_rate;
 rates(2) = _ctrl_state.yaw_rate;

 //throttle pid attenuation factor
 float tpa = fmaxf(0.0f,
 fminf(1.0f,
 1.0f
 - _params.tpa_slope
 * (fabsf(_v_rates_sp.thrust)
 - _params.tpa_breakpoint)));

 //angular rates error
 math::Vector<3> rates_err = _rates_sp - rates;

 _att_control = _params.rate_p.emult(rates_err * tpa)
 + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int
 + _params.rate_ff.emult(_rates_sp);

 _rates_sp_prev = _rates_sp;
 _rates_prev = rates;

 //update integral only if not saturated on low limit and if motor commands are not saturated
 if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit
 && !_motor_limits.upper_limit) {
 for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
 if (fabsf(_att_control(i)) < _thrust_sp) {
 float rate_i = _rates_int(i)
 + _params.rate_i(i) * rates_err(i) * dt;

 if (PX4_ISFINITE(rate_i) && rate_i > -RATES_I_LIMIT
 && rate_i < RATES_I_LIMIT
 && _att_control(i) > -RATES_I_LIMIT
 && _att_control(i) < RATES_I_LIMIT &&
 //if the axis is the yaw axis, do not update the integral if the limit is hit
 !((i == AXIS_INDEX_YAW) && _motor_limits.yaw)) {
 _rates_int(i) = rate_i;
 }
 }
 }
 }
 }
 */
void MulticopterAttitudeControl::control_attitude_rates_1(float dt) {

    struct vehicle_gps_position_s buf_gps_pos;
    memset(&buf_gps_pos, 0, sizeof(buf_gps_pos));

    /* reset integral if disarmed */
    if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
        _rates_int.zero();
    }

    /* current body angular rates */
    math::Vector<3> rates;
    rates(0) = _ctrl_state.roll_rate;
    rates(1) = _ctrl_state.pitch_rate;
    rates(2) = _ctrl_state.yaw_rate;

    /* throttle pid attenuation factor */
    /*
     float tpa = fmaxf(0.0f,
     fminf(1.0f,
     1.0f
     - _params.tpa_slope
     * (fabsf(_v_rates_sp.thrust)
     - _params.tpa_breakpoint)));
     */
    /* angular rates error */
    math::Vector<3> rates_err = _rates_sp - rates;
    /*
     param1[0] = rates_err(0) * rates_err(0);
     param1[1] = rates_err(1) * rates_err(1);
     param1[2] = rates_err(2) * rates_err(2);
     for (int i = 0; i < 3; i++) {
     fprintf(fe, "%.4f\t", param1[i]);
     }
     fprintf(fe, "\n\r");
     */
    /////////////////////////////nn begin
    if ((begin_count-- < 0) && (neure_flag_enabled == true)) { //
        begin_count = -1;
        if (train_count == 0) //学习参数重新初始化
                {
            n_n.deta_w21(1) = 0;
            n_n.deta_w22(1) = 0;
            n_n.deta_w23(1) = 0;
            n_n.deta_w1_1(1) = 0;
            n_n.deta_w1_2(1) = 0;
            n_n.deta_w1_3(1) = 0;
            n_n.deta_w2_1(1) = 0;
            n_n.deta_w2_2(1) = 0;
            n_n.deta_w2_3(1) = 0;

            n_n.deta2_1(1) = 0;
            n_n.deta2_2(1) = 0;
            n_n.deta2_3(1) = 0;
            n_n.deta1(1) = 0;

            n_n.Error(1) = 0;

            train_count = 1;
        } else {
            if (train_count == sample_number + 1) { //计算最终改变量
                n_n.deta_w21(1) = n_n.deta_w21(1) * (-1) / sample_number;
                n_n.deta_w22(1) = n_n.deta_w21(1) * (-1) / sample_number;
                n_n.deta_w23(1) = n_n.deta_w21(1) * (-1) / sample_number;

                n_n.deta_w1_1(1) = n_n.deta_w1_1(1) * (-1) / sample_number;
                n_n.deta_w1_2(1) = n_n.deta_w1_2(1) * (-1) / sample_number;
                n_n.deta_w1_3(1) = n_n.deta_w1_3(1) * (-1) / sample_number;
                n_n.deta_w2_1(1) = n_n.deta_w2_1(1) * (-1) / sample_number;
                n_n.deta_w2_2(1) = n_n.deta_w2_2(1) * (-1) / sample_number;
                n_n.deta_w2_3(1) = n_n.deta_w2_3(1) * (-1) / sample_number;

                n_n.Error(1) = n_n.Error(1) / sample_number;
                if ((double)n_n.Error(1) < error_min) {
                    unstable_flag_enabled = false;
                } else if ((double)n_n.Error(1) > error_max) {
                    unstable_flag_enabled = true;
                }
                if (unstable_flag_enabled == true) {
                    //学习速率初始化
                    n_n.study_speed_h_o1 = 0.01;

                    n_n.study_speed_h_o2 = 0.01;

                    n_n.study_speed_h_o3 = 0.01;

                    n_n.study_speed_i_h11 = 0.01;

                    n_n.study_speed_i_h12 = 0.01;

                    n_n.study_speed_i_h13 = 0.01;

                    n_n.study_speed_i_h21 = 0.01;

                    n_n.study_speed_i_h22 = 0.01;

                    n_n.study_speed_i_h23 = 0.01;

                    while ((n_n.study_speed_h_o1
                            > ((n_n.Error(1))
                                    / (n_n.deta_w21(1) * n_n.deta_w21(1)) / 100))
                            && (compare_vector(n_n.deta_w21(1), 0))) {
                        n_n.study_speed_h_o1 = n_n.study_speed_h_o1 / 10;
                    }
                    while ((n_n.study_speed_h_o2
                            > ((n_n.Error(1))
                                    / (n_n.deta_w22(1) * n_n.deta_w22(1)) / 100))
                            && (compare_vector(n_n.deta_w22(1), 0))) {
                        n_n.study_speed_h_o2 = n_n.study_speed_h_o2 / 10;
                    }
                    while ((n_n.study_speed_h_o3
                            > ((n_n.Error(1))
                                    / (n_n.deta_w23(1) * n_n.deta_w23(1)) / 100))
                            && (compare_vector(n_n.deta_w23(1), 0))) {
                        n_n.study_speed_h_o3 = n_n.study_speed_h_o3 / 10;
                    }

                    while ((n_n.study_speed_i_h11
                            > ((n_n.Error(1))
                                    / (n_n.deta_w1_1(1) * n_n.deta_w1_1(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w1_1(1), 0))) {
                        n_n.study_speed_i_h11 = n_n.study_speed_i_h11 / 10;

                    }
                    while ((n_n.study_speed_i_h12
                            > ((n_n.Error(1))
                                    / (n_n.deta_w1_2(1) * n_n.deta_w1_2(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w1_2(1), 0))) {
                        n_n.study_speed_i_h12 = n_n.study_speed_i_h12 / 10;

                    }
                    while ((n_n.study_speed_i_h13
                            > ((n_n.Error(1))
                                    / (n_n.deta_w1_3(1) * n_n.deta_w1_3(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w1_3(1), 0))) {
                        n_n.study_speed_i_h13 = n_n.study_speed_i_h13 / 10;

                    }
                    while ((n_n.study_speed_i_h21
                            > ((n_n.Error(1))
                                    / (n_n.deta_w2_1(1) * n_n.deta_w2_1(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w2_1(1), 0))) {
                        n_n.study_speed_i_h21 = n_n.study_speed_i_h21 / 10;

                    }
                    while ((n_n.study_speed_i_h22
                            > ((n_n.Error(1))
                                    / (n_n.deta_w2_2(1) * n_n.deta_w2_2(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w2_2(1), 0))) {
                        n_n.study_speed_i_h22 = n_n.study_speed_i_h22 / 10;

                    }
                    while ((n_n.study_speed_i_h23
                            > ((n_n.Error(1))
                                    / (n_n.deta_w2_3(1) * n_n.deta_w2_3(1))
                                    / 100))
                            && (compare_vector(n_n.deta_w23(1), 0))) {
                        n_n.study_speed_i_h23 = n_n.study_speed_i_h23 / 10;

                    }

                    n_n.w21(1) = n_n.w21(1)
                            - n_n.study_speed_h_o1 * (n_n.deta_w21(1));
                    n_n.w22(1) = n_n.w22(1)
                            - n_n.study_speed_h_o2 * (n_n.deta_w22(1));
                    n_n.w23(1) = n_n.w23(1)
                            - n_n.study_speed_h_o3 * (n_n.deta_w23(1));

                    n_n.w1_1(1) = n_n.w1_1(1)
                            - n_n.study_speed_i_h11 * (n_n.deta_w1_1(1));
                    n_n.w1_2(1) = n_n.w1_2(1)
                            - n_n.study_speed_i_h12 * (n_n.deta_w1_2(1));
                    n_n.w1_3(1) = n_n.w1_3(1)
                            - n_n.study_speed_i_h13 * (n_n.deta_w1_3(1));
                    n_n.w2_1(1) = n_n.w2_1(1)
                            - n_n.study_speed_i_h21 * (n_n.deta_w2_1(1));
                    n_n.w2_2(1) = n_n.w2_2(1)
                            - n_n.study_speed_i_h22 * (n_n.deta_w2_2(1));
                    n_n.w2_3(1) = n_n.w2_3(1)
                            - n_n.study_speed_i_h23 * (n_n.deta_w2_3(1));

                    ///////////////输出记录更新的PID参数
                    param[0] = n_n.w21(1);
                    param[1] = n_n.w22(1);
                    param[2] = n_n.w23(1);
                    param[3] = n_n.w1_1(1);
                    param[4] = n_n.w1_2(1);
                    param[5] = n_n.w1_3(1);
                    param[6] = n_n.w2_1(1);
                    param[7] = n_n.w2_2(1);
                    param[8] = n_n.w2_3(1);
                    param[9] = n_n.Error(1);
                    param[10] = n_n.study_speed_h_o1;
                    param[11] = n_n.deta_w21(1);

                    fd = fopen("/fs/microsd/pidchange.txt", "a+");
                    //fe = fopen("/fs/microsd/errorchange.txt", "a+");
                    //////获取当前时刻时间
                    time_t timeSec = time(NULL); //获取1970.1.1至当前秒数time_t
                    struct tm *timeinfo = localtime(&timeSec); //创建TimeData，并转化成当地时间
                    fprintf(fd, "%d-%d-%d  %d:%d:%d\t",
                            timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                            timeinfo->tm_mday, timeinfo->tm_hour + 8,
                            timeinfo->tm_min, timeinfo->tm_sec);
                    for (int i = 0; i < 12; i++) {
                        fprintf(fd, "%.4f\t", param[i]);
                    }
                    fprintf(fd, "\n\r");
                    fclose(fd);
                    train_count = 0;
                }
            } else if (compare_vector(n_n.v(1), n_n.v_prev(1))
                    && compare_vector(n_n.u2_1(1), n_n.u2_1_prev(1))
                    && compare_vector(n_n.u2_2(1), n_n.u2_2_prev(1))
                    && compare_vector(n_n.u2_3(1), n_n.u2_3_prev(1))) {
                //第K+1次参数更新
                n_n.y_ = rates;
                n_n.y_prev_ = n_n.y;

                n_n.deta1(1) = ((n_n.r(1) - n_n.y(1))
                        * (sign(
                                (n_n.y_(1) - n_n.y_prev_(1))
                                        / (n_n.v(1) - n_n.v_prev(1))))) * 2; //in-hide deta

                n_n.deta2_1(1) = (n_n.deta1(1))
                        * (sign(
                                (n_n.x2_1(1) - n_n.x2_1_prev(1))
                                        / (n_n.u2_1(1) - n_n.u2_1_prev(1))))
                        * (n_n.w21(1));
                n_n.deta2_2(1) = (n_n.deta1(1))
                        * (sign(
                                (n_n.x2_2(1) - n_n.x2_2_prev(1))
                                        / (n_n.u2_2(1) - n_n.u2_2_prev(1))))
                        * (n_n.w22(1));
                n_n.deta2_3(1) = (n_n.deta1(1))
                        * (sign(
                                (n_n.x2_3(1) - n_n.x2_3_prev(1))
                                        / (n_n.u2_3(1) - n_n.u2_3_prev(1))))
                        * (n_n.w23(1));

                //hide-out
                n_n.deta_w21(1) += (n_n.deta1(1)) * (n_n.x2_1(1));
                n_n.deta_w22(1) += (n_n.deta1(1)) * (n_n.x2_2(1));
                n_n.deta_w23(1) += (n_n.deta1(1)) * (n_n.x2_3(1));
                //in-hide
                n_n.deta_w1_1(1) += n_n.deta2_1(1) * (n_n.x1_1(1));
                n_n.deta_w1_2(1) += n_n.deta2_2(1) * (n_n.x1_1(1));
                n_n.deta_w1_3(1) += n_n.deta2_3(1) * (n_n.x1_1(1));

                n_n.deta_w2_1(1) += n_n.deta2_1(1) * (n_n.x1_2(1));
                n_n.deta_w2_2(1) += n_n.deta2_2(1) * (n_n.x1_2(1));
                n_n.deta_w2_3(1) += n_n.deta2_3(1) * (n_n.x1_2(1));

                n_n.Error(1) += (n_n.r(1) - n_n.y(1)) * (n_n.r(1) - n_n.y(1));

                train_count++;
            }
        }

    }
    //第K-1次参数记录///////////////////////
    n_n.x2_1_prev = n_n.x2_1;
    n_n.x2_2_prev = n_n.x2_2;

    n_n.u2_1_prev = n_n.u2_1;
    n_n.u2_2_prev = n_n.u2_2;

    n_n.u2_3_prev = n_n.u2_3;
    n_n.x2_3_prev = n_n.x2_3;

    n_n.v_prev = n_n.v;
    n_n.y_prev = n_n.y;
    //end
    //第K次参数更新/////////////////////////////
    n_n.r = _rates_sp; //更新r，记录输入target为输入层单元1的in
    n_n.y = rates; //更新y，记录输入measure为输入层单元2的in
    //输入层赋值,计算
    n_n.u1_1 = _rates_sp;
    n_n.u1_2 = rates;
    n_n.x1_1 = Limit(n_n.u1_1);
    n_n.x1_2 = Limit(n_n.u1_2);
    //隐藏层赋值,计算
    n_n.u2_1 = n_n.w1_1.emult(n_n.x1_1) + n_n.w2_1.emult(n_n.x1_2);
    n_n.u2_2 = n_n.w1_2.emult(n_n.x1_1) + n_n.w2_2.emult(n_n.x1_2);
    n_n.u2_3 = n_n.w1_3.emult(n_n.x1_1) + n_n.w2_3.emult(n_n.x1_2);

    n_n.x2_1 = Limit(n_n.u2_1); //隐藏层计算 比例元
    n_n.x2_2 = Limit((n_n.u2_2 - n_n.u2_2_prev) / dt); //隐藏层计算 微分元

    ////上次神经元参数记录2

    _att_control = n_n.w21.emult(n_n.x2_1) + n_n.w22.emult(n_n.x2_2)
            + n_n.w23.emult(n_n.x2_3) + _params.rate_ff.emult(_rates_sp);
    //输出层计算
    n_n.u3 = _att_control;
    n_n.x3 = Limit(n_n.u3);

    _att_control = n_n.x3; //记录神经网络输出，用_att_control标记

    n_n.v = _att_control; //更新v
    //   _rates_sp_prev = _rates_sp;
    //  _rates_prev = rates;

    /* update integral only if not saturated on low limit and if motor commands are not saturated */
    if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit
            && !_motor_limits.upper_limit) {
        for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
            if (fabsf(_att_control(i)) < _thrust_sp) {
                float rate_i = n_n.x2_3_prev(i) + n_n.u2_3(i) * dt; //隐藏层计算 积分元

                if (PX4_ISFINITE(rate_i * n_n.w23(i))
                        && (rate_i * n_n.w23(i)) > -RATES_I_LIMIT
                        && (rate_i * n_n.w23(i)) < RATES_I_LIMIT
                        && _att_control(i) > -RATES_I_LIMIT
                        && _att_control(i) < RATES_I_LIMIT &&
                        /* if the axis is the yaw axis, do not update the integral if the limit is hit */
                        !((i == AXIS_INDEX_YAW) && _motor_limits.yaw)) {
                    n_n.u2_3(i) = rate_i; //隐藏层计算 积分元
                }
            }
        }
        n_n.x2_3 = Limit(n_n.u2_3); //隐藏层计算 积分元
    }
    //end

}
void MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[]) {

    mc_att_control::g_control->task_main();

}

void MulticopterAttitudeControl::task_main() {

    /*
     * do subscriptions
     */
    _v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _params_sub = orb_subscribe(ORB_ID(parameter_update));
    _manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));

    /* initialize parameters cache */
    parameters_update();
    /////////////hide-out初始权值赋值为PID初始参数
    n_n.w21 = _params.rate_p;
    n_n.w22 = _params.rate_i;
    n_n.w23 = _params.rate_d;
    /* wakeup source: vehicle attitude */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = _ctrl_state_sub;
    fds[0].events = POLLIN;
    while (!_task_should_exit) {

        /* wait for up to 100ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("mc att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf);

        /* run controller on attitude changes */
        if (fds[0].revents & POLLIN) {
            static uint64_t last_run = 0;
            float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
            last_run = hrt_absolute_time();

            /* guard against too small (< 2ms) and too large (> 20ms) dt's */
            if (dt < 0.002f) {
                dt = 0.002f;

            } else if (dt > 0.02f) {
                dt = 0.02f;
            }

            /* copy attitude and control state topics */
            orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

            /* check for updates in other topics */
            parameter_update_poll();
            vehicle_control_mode_poll();
            arming_status_poll();
            vehicle_manual_poll();
            vehicle_status_poll();
            vehicle_motor_limits_poll();

            /* Check if we are in rattitude mode and the pilot is above the threshold on pitch
             * or roll (yaw can rotate 360 in normal att control).  If both are true don't
             * even bother running the attitude controllers */
            if (_v_control_mode.flag_control_rattitude_enabled) {
                if (fabsf(_manual_control_sp.y) > _params.rattitude_thres
                        || fabsf(_manual_control_sp.x)
                                > _params.rattitude_thres) {
                    _v_control_mode.flag_control_attitude_enabled = false;
                }
            }

            if (_v_control_mode.flag_control_attitude_enabled) {

                if (_ts_opt_recovery == nullptr) {
                    // the  tailsitter recovery instance has not been created, thus, the vehicle
                    // is not a tailsitter, do normal attitude control
                    control_attitude(dt);

                } else {
                    vehicle_attitude_setpoint_poll();
                    _thrust_sp = _v_att_sp.thrust;
                    math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1],
                            _ctrl_state.q[2], _ctrl_state.q[3]);
                    math::Quaternion q_sp(&_v_att_sp.q_d[0]);
                    _ts_opt_recovery->setAttGains(_params.att_p,
                            _params.yaw_ff);
                    _ts_opt_recovery->calcOptimalRates(q, q_sp,
                            _v_att_sp.yaw_sp_move_rate, _rates_sp);

                    /* limit rates */
                    for (int i = 0; i < 3; i++) {
                        _rates_sp(i) = math::constrain(_rates_sp(i),
                                -_params.mc_rate_max(i),
                                _params.mc_rate_max(i));
                    }
                }

                /* publish attitude rates setpoint */
                _v_rates_sp.roll = _rates_sp(0);
                _v_rates_sp.pitch = _rates_sp(1);
                _v_rates_sp.yaw = _rates_sp(2);
                _v_rates_sp.thrust = _thrust_sp;
                _v_rates_sp.timestamp = hrt_absolute_time();

                if (_v_rates_sp_pub != nullptr) {
                    orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

                } else if (_rates_sp_id) {
                    _v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
                }

                //}

            } else {
                /* attitude controller disabled, poll rates setpoint topic */
                if (_v_control_mode.flag_control_manual_enabled) {
                    /* manual rates control - ACRO mode */
                    _rates_sp = math::Vector<3>(_manual_control_sp.y,
                            -_manual_control_sp.x, _manual_control_sp.r).emult(
                            _params.acro_rate_max);
                    _thrust_sp = math::min(_manual_control_sp.z,
                            MANUAL_THROTTLE_MAX_MULTICOPTER);

                    /* publish attitude rates setpoint */
                    _v_rates_sp.roll = _rates_sp(0);
                    _v_rates_sp.pitch = _rates_sp(1);
                    _v_rates_sp.yaw = _rates_sp(2);
                    _v_rates_sp.thrust = _thrust_sp;
                    _v_rates_sp.timestamp = hrt_absolute_time();

                    if (_v_rates_sp_pub != nullptr) {
                        orb_publish(_rates_sp_id, _v_rates_sp_pub,
                                &_v_rates_sp);

                    } else if (_rates_sp_id) {
                        _v_rates_sp_pub = orb_advertise(_rates_sp_id,
                                &_v_rates_sp);
                    }

                } else {
                    /* attitude controller disabled, poll rates setpoint topic */
                    vehicle_rates_setpoint_poll();
                    _rates_sp(0) = _v_rates_sp.roll;
                    _rates_sp(1) = _v_rates_sp.pitch;
                    _rates_sp(2) = _v_rates_sp.yaw;
                    _thrust_sp = _v_rates_sp.thrust;
                }
            }

            if (_v_control_mode.flag_control_rates_enabled) {

                control_attitude_rates_1(dt);
                /* publish actuator controls */
                _actuators.control[0] =
                        (PX4_ISFINITE(_att_control(0))) ?
                                _att_control(0) : 0.0f;
                _actuators.control[1] =
                        (PX4_ISFINITE(_att_control(1))) ?
                                _att_control(1) : 0.0f;
                _actuators.control[2] =
                        (PX4_ISFINITE(_att_control(2))) ?
                                _att_control(2) : 0.0f;
                _actuators.control[3] =
                        (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
                _actuators.control[7] = _v_att_sp.landing_gear;
                _actuators.timestamp = hrt_absolute_time();
                _actuators.timestamp_sample = _ctrl_state.timestamp;

                _controller_status.roll_rate_integ = _rates_int(0);
                _controller_status.pitch_rate_integ = _rates_int(1);
                _controller_status.yaw_rate_integ = _rates_int(2);
                _controller_status.timestamp = hrt_absolute_time();

                if (!_actuators_0_circuit_breaker_enabled) {
                    if (_actuators_0_pub != nullptr) {

                        orb_publish(_actuators_id, _actuators_0_pub,
                                &_actuators);
                        perf_end(_controller_latency_perf);

                    } else if (_actuators_id) {
                        _actuators_0_pub = orb_advertise(_actuators_id,
                                &_actuators);
                    }

                }

                /* publish controller status */
                if (_controller_status_pub != nullptr) {
                    orb_publish(ORB_ID(mc_att_ctrl_status),
                            _controller_status_pub, &_controller_status);

                } else {
                    _controller_status_pub = orb_advertise(
                            ORB_ID(mc_att_ctrl_status), &_controller_status);
                }
            }
        }

        perf_end(_loop_perf);
    }

    _control_task = -1;
    return;
}

int MulticopterAttitudeControl::start() {
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("mc_att_control", SCHED_DEFAULT,
            SCHED_PRIORITY_MAX - 5, 1500,
            (px4_main_t) &MulticopterAttitudeControl::task_main_trampoline,
            nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

int mc_att_control_main(int argc, char *argv[]) {
    if (argc < 2) {
        warnx("usage: mc_att_control {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (mc_att_control::g_control != nullptr) {
            warnx("already running");
            return 1;
        }

        mc_att_control::g_control = new MulticopterAttitudeControl;

        if (mc_att_control::g_control == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != mc_att_control::g_control->start()) {
            delete mc_att_control::g_control;
            mc_att_control::g_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (mc_att_control::g_control == nullptr) {
            warnx("not running");
            return 1;
        }

        delete mc_att_control::g_control;
        mc_att_control::g_control = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (mc_att_control::g_control) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}
