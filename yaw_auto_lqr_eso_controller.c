/**
 ******************************************************************************
 * @file    yaw_auto_lqr_eso_controller.c
 * @school  南京理工大学（NJUST）
 * @author  戴子轩
 * @email   daizeming24@gmail.com
 * @date    2026/5/16
 * @brief   Standalone yaw auto-aim LQR-ESO controller matching the math summary.
 ******************************************************************************
 */

#include "yaw_auto_lqr_eso_controller.h"

#include <math.h>
#include <string.h>

#define YAW_AUTO_LQR_ESO_MIN_J       (1.0e-6f)
#define YAW_AUTO_LQR_ESO_MIN_DT_S    (1.0e-5f)
#define YAW_AUTO_LQR_ESO_MAX_DT_S    (0.05f)
#define YAW_AUTO_LQR_ESO_DEFAULT_DT_S (0.001f)

static float YawAutoAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float YawAutoClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float YawAutoClampAbs(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    return YawAutoClamp(value, -limit, limit);
}

static float YawAutoDeadband(float value, float deadband)
{
    const float abs_value = YawAutoAbs(value);

    if (deadband <= 0.0f) {
        return value;
    }
    if (abs_value <= deadband) {
        return 0.0f;
    }
    return (value > 0.0f) ? (value - deadband) : (value + deadband);
}

static float YawAutoSanitizeDt(float dt_s)
{
    if (dt_s < YAW_AUTO_LQR_ESO_MIN_DT_S || dt_s > YAW_AUTO_LQR_ESO_MAX_DT_S) {
        return YAW_AUTO_LQR_ESO_DEFAULT_DT_S;
    }
    return dt_s;
}

static uint8_t YawAutoGatePass(float value, float limit)
{
    if (limit <= 0.0f) {
        return 1U;
    }
    return (YawAutoAbs(value) <= limit) ? 1U : 0U;
}

static void YawAutoUpdateEsoGains(YawAutoLqrEso_t *ctrl,
                                  const YawAutoLqrEsoConfig_t *cfg)
{
    float w0;

    if (ctrl == NULL || cfg == NULL || cfg->j_kg_m2 <= YAW_AUTO_LQR_ESO_MIN_J ||
        cfg->eso_enable == 0U) {
        if (ctrl != NULL) {
            ctrl->eso.b0 = 0.0f;
            ctrl->eso.w0 = 0.0f;
            ctrl->eso.beta1 = 0.0f;
            ctrl->eso.beta2 = 0.0f;
            ctrl->eso.beta3 = 0.0f;
        }
        return;
    }

    w0 = (cfg->eso_bandwidth_rad_s > 0.0f) ? cfg->eso_bandwidth_rad_s : 0.0f;
    ctrl->eso.b0 = 1.0f / cfg->j_kg_m2;
    ctrl->eso.w0 = w0;
    ctrl->eso.beta1 = 3.0f * w0;
    ctrl->eso.beta2 = 3.0f * w0 * w0;
    ctrl->eso.beta3 = w0 * w0 * w0;
}

static void YawAutoUpdateEso(YawAutoLqrEso_t *ctrl,
                             const YawAutoLqrEsoConfig_t *cfg,
                             float theta_meas_rad,
                             float u_nm,
                             float dt_s)
{
    float e;
    float z1_dot;
    float z2_dot;
    float z3_dot;
    float damping;

    if (ctrl == NULL || cfg == NULL || cfg->eso_enable == 0U ||
        cfg->j_kg_m2 <= YAW_AUTO_LQR_ESO_MIN_J || ctrl->eso.w0 <= 0.0f) {
        return;
    }

    damping = cfg->b_nms_rad / cfg->j_kg_m2;
    e = theta_meas_rad - ctrl->eso.z1;
    z1_dot = ctrl->eso.z2 + ctrl->eso.beta1 * e;
    z2_dot = -damping * ctrl->eso.z2 + ctrl->eso.b0 * u_nm +
             ctrl->eso.z3 + ctrl->eso.beta2 * e;
    z3_dot = ctrl->eso.beta3 * e;

    ctrl->eso.z1 += dt_s * z1_dot;
    ctrl->eso.z2 += dt_s * z2_dot;
    ctrl->eso.z3 += dt_s * z3_dot;
}

static float YawAutoCoulombComp(const YawAutoLqrEsoConfig_t *cfg, float omega_ref_rad_s)
{
    if (cfg == NULL || cfg->coulomb_smooth_rad_s <= 1.0e-6f ||
        cfg->tau_coulomb_nm == 0.0f) {
        return 0.0f;
    }
    return cfg->tau_coulomb_nm * tanhf(omega_ref_rad_s / cfg->coulomb_smooth_rad_s);
}

static float YawAutoCalcEsoComp(const YawAutoLqrEso_t *ctrl,
                                const YawAutoLqrEsoConfig_t *cfg)
{
    float tau_eso;

    if (ctrl == NULL || cfg == NULL || cfg->eso_enable == 0U ||
        ctrl->eso.b0 <= 1.0e-6f || cfg->eso_comp_gain == 0.0f) {
        return 0.0f;
    }

    tau_eso = -cfg->eso_comp_gain * ctrl->eso.z3 / ctrl->eso.b0;
    return YawAutoClampAbs(tau_eso, cfg->eso_comp_limit_nm);
}

void YawAutoLqrEso_Init(YawAutoLqrEso_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
}

void YawAutoLqrEso_Reset(YawAutoLqrEso_t *ctrl, float theta_rad, float omega_rad_s)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->eso.z1 = theta_rad;
    ctrl->eso.z2 = omega_rad_s;
    ctrl->eso.z3 = 0.0f;
    ctrl->theta_integral_rad_s = 0.0f;
    ctrl->tau_bias_nm = 0.0f;
    ctrl->tau_meas_filt_nm = 0.0f;
    ctrl->tau_cmd_last_nm = 0.0f;
    ctrl->feedback_ready = 0U;
}

uint8_t YawAutoLqrEso_GetStateSpace(const YawAutoLqrEsoConfig_t *cfg,
                                    YawAutoLqrEsoStateSpace_t *model)
{
    float inv_j;

    if (cfg == NULL || model == NULL || cfg->j_kg_m2 <= YAW_AUTO_LQR_ESO_MIN_J) {
        return 0U;
    }

    memset(model, 0, sizeof(*model));
    inv_j = 1.0f / cfg->j_kg_m2;

    model->a[0][0] = 0.0f;
    model->a[0][1] = 1.0f;
    model->a[1][0] = 0.0f;
    model->a[1][1] = -cfg->b_nms_rad * inv_j;

    model->b[0][0] = 0.0f;
    model->b[0][1] = 0.0f;
    model->b[1][0] = inv_j;
    model->b[1][1] = inv_j;

    model->c[0] = 1.0f;
    model->c[1] = 0.0f;
    return 1U;
}

void YawAutoLqrEso_Calc(YawAutoLqrEso_t *ctrl,
                        const YawAutoLqrEsoConfig_t *cfg,
                        const YawAutoLqrEsoFeedback_t *feedback,
                        const YawAutoLqrEsoReference_t *ref,
                        float dt_s,
                        YawAutoLqrEsoOutput_t *output)
{
    YawAutoLqrEsoOutput_t out;
    float e_theta;
    float e_omega;
    float tau_without_bias;
    float tau_cmd;
    float tau_meas_alpha;
    float tau_track_err;

    memset(&out, 0, sizeof(out));
    if (ctrl == NULL || cfg == NULL || feedback == NULL || ref == NULL) {
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    dt_s = YawAutoSanitizeDt(dt_s);
    YawAutoUpdateEsoGains(ctrl, cfg);

    if (feedback->feedback_ok == 0U) {
        YawAutoLqrEso_Reset(ctrl, feedback->theta_rad, feedback->omega_rad_s);
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    if (ctrl->feedback_ready == 0U) {
        ctrl->feedback_ready = 1U;
        ctrl->tau_meas_filt_nm = feedback->tau_meas_nm;
        ctrl->eso.z1 = feedback->theta_rad;
        ctrl->eso.z2 = feedback->omega_rad_s;
        ctrl->eso.z3 = 0.0f;
        out.theta_ref_rad = ref->theta_rad;
        out.omega_ref_rad_s = ref->omega_rad_s;
        out.alpha_ref_rad_s2 = ref->alpha_rad_s2;
        if (output != NULL) {
            *output = out;
        }
        return;
    }

    tau_meas_alpha = YawAutoClamp(cfg->tau_meas_lpf_alpha, 0.0f, 1.0f);
    ctrl->tau_meas_filt_nm +=
        tau_meas_alpha * (feedback->tau_meas_nm - ctrl->tau_meas_filt_nm);

    YawAutoUpdateEso(ctrl, cfg, feedback->theta_rad, ctrl->tau_cmd_last_nm, dt_s);

    e_theta = YawAutoDeadband(feedback->theta_rad - ref->theta_rad,
                              cfg->theta_deadband_rad);
    e_omega = feedback->omega_rad_s - ref->omega_rad_s;

    ctrl->theta_integral_rad_s += e_theta * dt_s;
    ctrl->theta_integral_rad_s =
        YawAutoClampAbs(ctrl->theta_integral_rad_s,
                        cfg->theta_integral_limit_rad_s);

    out.theta_ref_rad = ref->theta_rad;
    out.omega_ref_rad_s = ref->omega_rad_s;
    out.alpha_ref_rad_s2 = ref->alpha_rad_s2;
    out.e_theta_rad = e_theta;
    out.e_omega_rad_s = e_omega;
    out.tau_ff_alpha_nm = cfg->j_kg_m2 * ref->alpha_rad_s2;
    out.tau_ff_viscous_nm = cfg->b_nms_rad * ref->omega_rad_s;
    out.tau_ff_coulomb_nm = YawAutoCoulombComp(cfg, ref->omega_rad_s);
    out.tau_ff_nm = out.tau_ff_alpha_nm +
                    out.tau_ff_viscous_nm +
                    out.tau_ff_coulomb_nm;
    out.tau_i_nm = -cfg->k_i * ctrl->theta_integral_rad_s;
    out.tau_lqr_nm = out.tau_ff_nm -
                     cfg->k_theta * e_theta -
                     cfg->k_omega * e_omega +
                     out.tau_i_nm;
    out.tau_eso_raw_nm = YawAutoCalcEsoComp(ctrl, cfg);
    if (cfg->eso_comp_enable != 0U &&
        YawAutoGatePass(feedback->omega_rad_s, cfg->eso_omega_gate_rad_s) != 0U &&
        YawAutoGatePass(ref->alpha_rad_s2, cfg->eso_alpha_gate_rad_s2) != 0U) {
        out.eso_comp_active = 1U;
        out.tau_eso_active_nm = out.tau_eso_raw_nm;
    }

    tau_without_bias = out.tau_lqr_nm + out.tau_eso_active_nm;
    tau_track_err = tau_without_bias - ctrl->tau_meas_filt_nm;
    ctrl->tau_bias_nm += cfg->tau_bias_ki * tau_track_err * dt_s;
    ctrl->tau_bias_nm =
        YawAutoClampAbs(ctrl->tau_bias_nm, cfg->tau_bias_limit_nm);

    out.tau_bias_nm = ctrl->tau_bias_nm;
    out.tau_pre_limit_nm = tau_without_bias + ctrl->tau_bias_nm;
    tau_cmd = out.tau_pre_limit_nm;

    if (cfg->torque_soft_limit_nm > 0.0f) {
        const float limited_tau = YawAutoClampAbs(tau_cmd,
                                                  cfg->torque_soft_limit_nm);
        if (limited_tau != tau_cmd) {
            out.soft_limit_active = 1U;
        }
        tau_cmd = limited_tau;
    }

    if (cfg->torque_min_nm < cfg->torque_max_nm) {
        const float limited_tau = YawAutoClamp(tau_cmd,
                                               cfg->torque_min_nm,
                                               cfg->torque_max_nm);
        if (limited_tau != tau_cmd) {
            out.hard_limit_active = 1U;
        }
        tau_cmd = limited_tau;
    }

    out.tau_cmd_before_slew_nm = tau_cmd;
    if (cfg->torque_slew_enable != 0U &&
        cfg->torque_slew_rate_nm_s > 0.0f) {
        const float max_delta = cfg->torque_slew_rate_nm_s * dt_s;
        const float limited_tau =
            YawAutoClamp(tau_cmd,
                         ctrl->tau_cmd_last_nm - max_delta,
                         ctrl->tau_cmd_last_nm + max_delta);
        if (limited_tau != tau_cmd) {
            out.slew_limit_active = 1U;
        }
        tau_cmd = limited_tau;
    }

    out.tau_cmd_nm = tau_cmd;
    ctrl->tau_cmd_last_nm = tau_cmd;
    if (output != NULL) {
        *output = out;
    }
}
