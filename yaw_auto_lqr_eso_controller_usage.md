# Yaw 自瞄 LQR-ESO 控制器使用说明

本文说明 `yaw_auto_lqr_eso_controller.c/.h` 的使用方法。该代码是
`单轴云台最优控制器设计.pdf` 的配套整理版，默认只描述 yaw 自瞄控制器，不包含包间外推、视觉包锁存或恒加速度外推逻辑。

相关文件：

- `yaw_auto_lqr_eso_controller.h`
- `yaw_auto_lqr_eso_controller.c`
- `单轴云台最优控制器设计.pdf`
- `yaw_auto_lqr_eso_controller_matlab.m`

## 1. 模块定位

该控制器输入连续时间参考量：

- `theta_rad`：参考角位置，单位 `rad`
- `omega_rad_s`：参考角速度，单位 `rad/s`
- `alpha_rad_s2`：参考角加速度，单位 `rad/s^2`

每个控制周期直接使用上述参考量计算控制力矩，不在模块内部判断视觉包是否更新，也不对两包之间的轨迹做外推。

内部计算链路为：

1. 更新三阶 ESO 观测器。
2. 计算跟踪误差 `e_theta = theta - theta_ref`、`e_omega = omega - omega_ref`。
3. 计算前馈力矩 `J * alpha_ref + B * omega_ref + tau_coulomb`。
4. 计算 LQR 反馈力矩。
5. 可选叠加 ESO 扰动补偿。
6. 叠加偏置补偿。
7. 经过软限幅、硬限幅和力矩斜率限制。

## 2. 接入方式

如果只是作为文档配套代码查看，不需要接入工程构建。

如果要接入固件工程，需要把 `.c/.h` 放入参与编译的源码目录，或在 CMake/Makefile 中显式加入：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    path/to/yaw_auto_lqr_eso_controller.c
)
```

调用方包含头文件：

```c
#include "yaw_auto_lqr_eso_controller.h"
```

## 3. 基本对象

控制器对象保存 ESO、积分、偏置和上一周期力矩：

```c
static YawAutoLqrEso_t yaw_ctrl;
```

配置对象保存模型参数、增益、限幅和开关：

```c
static YawAutoLqrEsoConfig_t yaw_cfg = {
    .j_kg_m2 = J_VALUE,
    .b_nms_rad = B_VALUE,
    .k_theta = K_THETA_VALUE,
    .k_omega = K_OMEGA_VALUE,
    .k_i = KI_VALUE,
    .theta_integral_limit_rad_s = INTEGRAL_LIMIT_VALUE,

    .tau_coulomb_nm = TAU_COULOMB_VALUE,
    .coulomb_smooth_rad_s = COULOMB_SMOOTH_VALUE,

    .eso_bandwidth_rad_s = ESO_W0_VALUE,
    .eso_comp_gain = ESO_COMP_GAIN_VALUE,
    .eso_comp_limit_nm = ESO_COMP_LIMIT_VALUE,
    .eso_omega_gate_rad_s = ESO_OMEGA_GATE_VALUE,
    .eso_alpha_gate_rad_s2 = ESO_ALPHA_GATE_VALUE,

    .tau_bias_ki = TAU_BIAS_KI_VALUE,
    .tau_bias_limit_nm = TAU_BIAS_LIMIT_VALUE,
    .tau_meas_lpf_alpha = TAU_MEAS_LPF_ALPHA_VALUE,
    .theta_deadband_rad = THETA_DEADBAND_VALUE,

    .torque_soft_limit_nm = TORQUE_SOFT_LIMIT_VALUE,
    .torque_min_nm = TORQUE_MIN_VALUE,
    .torque_max_nm = TORQUE_MAX_VALUE,
    .torque_slew_rate_nm_s = TORQUE_SLEW_RATE_VALUE,

    .eso_enable = 1U,
    .eso_comp_enable = 1U,
    .torque_slew_enable = 1U,
};
```

其中 `J_VALUE`、`B_VALUE`、`K_THETA_VALUE` 等应由固件参数、在线调参或 MATLAB 调参脚本给出。

## 4. 初始化

系统启动时初始化控制器：

```c
YawAutoLqrEso_Init(&yaw_ctrl);
```

如果电机反馈失效、控制模式切换或需要清空控制器历史状态，调用：

```c
YawAutoLqrEso_Reset(&yaw_ctrl, theta_meas_rad, omega_meas_rad_s);
```

`Reset` 会清空 ESO 扰动、积分、偏置和上一周期力矩，并用当前测量角位置/角速度初始化观测器状态。

## 5. 周期调用

每个控制周期构造反馈量和连续时间参考量：

```c
YawAutoLqrEsoFeedback_t feedback = {
    .theta_rad = theta_meas_rad,
    .omega_rad_s = omega_meas_rad_s,
    .tau_meas_nm = tau_meas_nm,
    .feedback_ok = feedback_ok,
};

YawAutoLqrEsoReference_t ref = {
    .theta_rad = theta_ref_rad,
    .omega_rad_s = omega_ref_rad_s,
    .alpha_rad_s2 = alpha_ref_rad_s2,
};

YawAutoLqrEsoOutput_t out;

YawAutoLqrEso_Calc(&yaw_ctrl, &yaw_cfg, &feedback, &ref, dt_s, &out);
```

然后把 `out.tau_cmd_nm` 发送给 yaw 电机：

```c
DMMotorSetTorque(yaw_motor, out.tau_cmd_nm);
```

第一次反馈有效时，控制器只同步测量状态并输出零力矩；下一周期开始正常闭环计算。

## 6. 输出字段

常用输出字段如下：

| 字段 | 含义 |
| --- | --- |
| `tau_cmd_nm` | 最终发送给电机的命令力矩 |
| `tau_lqr_nm` | 前馈加 LQR 反馈后的力矩 |
| `tau_ff_nm` | 总前馈力矩 |
| `tau_ff_alpha_nm` | 角加速度前馈项 |
| `tau_ff_viscous_nm` | 粘性阻尼前馈项 |
| `tau_ff_coulomb_nm` | 库仑摩擦补偿项 |
| `tau_eso_raw_nm` | ESO 估计出的原始补偿力矩 |
| `tau_eso_active_nm` | 实际接入控制的 ESO 补偿力矩 |
| `tau_bias_nm` | 偏置补偿力矩 |
| `e_theta_rad` | 角位置误差 |
| `e_omega_rad_s` | 角速度误差 |
| `soft_limit_active` | 软限幅是否触发 |
| `hard_limit_active` | 硬限幅是否触发 |
| `slew_limit_active` | 斜率限制是否触发 |
| `eso_comp_active` | ESO 补偿是否实际接入 |

## 7. 状态空间矩阵

如需在代码中得到与文档一致的状态空间矩阵，可调用：

```c
YawAutoLqrEsoStateSpace_t model;
uint8_t ok = YawAutoLqrEso_GetStateSpace(&yaw_cfg, &model);
```

返回成功后：

- `model.a` 对应状态矩阵 `A`
- `model.b` 对应输入矩阵 `B`
- `model.c` 对应输出矩阵 `C`

这里输入向量按 `[tau, d]` 排列，即控制力矩和等效总扰动共同作为输入。

## 8. 注意事项

- 所有角度输入使用弧度制，不使用角度制。
- 参考量必须由上游直接给出连续时间 `theta_ref`、`omega_ref`、`alpha_ref`。
- 模块内部不做角度 wrap；如果 yaw 跨越 `±pi` 或 `0/2pi` 边界，应由调用方先处理到最近等效角。
- `j_kg_m2` 必须大于零，否则状态空间和 ESO 输入增益无效。
- `eso_comp_enable = 0U` 时 ESO 仍可观测，但补偿不接入力矩。
- `eso_omega_gate_rad_s` 和 `eso_alpha_gate_rad_s2` 小于等于零时，对应门限视为不限制。
- `torque_min_nm < torque_max_nm` 时硬限幅生效。
- `torque_slew_enable = 1U` 且 `torque_slew_rate_nm_s > 0` 时斜率限制生效。

