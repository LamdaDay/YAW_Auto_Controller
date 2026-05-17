%% yaw_auto_lqr_tune.m
% Tune self-aim yaw LQR feedback gains from QR weights.
%
% Firmware path:
%   modules/lqr_eso/lqr_eso.c, YawLQRESO_Calc()
%
% Self-aim mode is active when input->auto_aim_mode != 0. In that mode the
% firmware uses:
%   e_theta = theta_meas - theta_ref_planned
%   e_omega = omega_meas - omega_ref_planned
%   tau_ff  = J_auto * alpha_ref + B_auto * omega_ref + coulomb_ff
%   tau_lqr = tau_ff - Ktheta * e_theta - Komega * e_omega + tau_lqi
%
% This script tunes only the feedback matrix:
%   K(1) -> g_lqr_auto_k_theta  (also LQR_AUTO_K_THETA_DEFAULT)
%   K(2) -> g_lqr_auto_k_omega  (also LQR_AUTO_K_OMEGA_DEFAULT)
%
% The current firmware K = (12.0, 3.4) was derived from live Ozone tuning.
% This script also solves continuous LQR (Control System Toolbox `lqr`), so
% the printed K can be pasted DIRECTLY into the firmware without any
% discrete-vs-continuous mismatch.
%
% Workflow:
%   1. Edit Q / R below.
%   2. Run.
%   3. Read the printed K matrix.
%   4. Copy the printed g_lqr_auto_k_* values into Ozone live-watch for
%      on-the-fly tuning, or copy the macro block into lqr_eso.c for a
%      permanent default change.
%
% Feedforward, Coulomb compensation, observer-only ESO, LQI, bias and output
% limits are not included in the Riccati solve. They are shown here only as
% firmware context.

clear; clc; close all;

%% Firmware-synchronized self-aim yaw model and limits
J = 0.039;                % g_yaw_auto_j / YAW_AUTO_J_DEFAULT, kg*m^2
Bv = 0.30;                % g_yaw_auto_b / YAW_AUTO_B_DEFAULT, N*m*s/rad
Ts = 0.001;               % YAW_TS_DEFAULT, s
torqueLimit = 7.0;        % g_ctrl_t_limit / DM torque limit, N*m
torqueSlewRate = 7000.0;  % g_ctrl_auto_torque_slew_rate, N*m/s

currentKTheta = 12.0;     % LQR_AUTO_K_THETA_DEFAULT
currentKOmega = 3.4;      % LQR_AUTO_K_OMEGA_DEFAULT

%% Tunable QR matrices
% State is x = [theta_error_rad; omega_error_rad_s].
% Increase Q(1,1) for tighter angle tracking.
% Increase Q(2,2) for more damping / less overshoot.
% Increase R(1,1) for softer torque and less limit_active.
Q = [330.0, 0.0;
       0.0, 10.0];
R = 1.0;

%% Manual override
% Keep false when you want QR changes to affect the printed K.
useManualTargetGain = false;
targetKTheta = currentKTheta;
targetKOmega = currentKOmega;

%% Offline sanity-step settings
stepRefDeg = 5.0;
simTimeS = 1.0;
settleBandRatio = 0.02;
enablePlot = false;

%% Plant model: J * theta_ddot + B * theta_dot = tau
A = [0, 1;
     0, -Bv / J];
Bu = [0;
      1 / J];

if rank([Bu, A * Bu]) < size(A, 1)
    error('Self-aim yaw plant is not controllable. Check J and Bv.');
end

validateQrMatrices(Q, R);
Q = 0.5 * (Q + Q.');

if exist('ss', 'file') ~= 2 || exist('c2d', 'file') ~= 2 || exist('lqr', 'file') ~= 2
    error(['Control System Toolbox functions ss(), c2d() and lqr() are required ', ...
           'for this firmware-matched tuning script.']);
end

sysC = ss(A, Bu, eye(2), zeros(2, 1));
sysD = c2d(sysC, Ts, 'zoh');
Ad = sysD.A;
Bd = sysD.B;

%% Continuous LQR gain (matches firmware design path)
% Firmware uses gains derived from continuous-time CARE, sampled at 1 ms.
% Solving with `lqr` (continuous) makes the printed K directly drop-in.
if useManualTargetGain
    K = [targetKTheta, targetKOmega];
    P = NaN(2);
    gainSource = 'manual targetKTheta/targetKOmega';
else
    [K, P, ~] = lqr(A, Bu, Q, R);
    gainSource = 'continuous CARE (firmware-matched)';
end
polesC = eig(A - Bu * K);

Ktheta = K(1);
Komega = K(2);
poleInfo = estimatePoleMetrics(polesC);

%% Saturated step simulation without FF/ESO/bias
t = 0:Ts:simTimeS;
thetaRef = deg2rad(stepRefDeg);
x = zeros(2, numel(t));
u = zeros(1, numel(t));
uRaw = zeros(1, numel(t));
torqueClampActive = false(1, numel(t));
slewActive = false(1, numel(t));
uLast = 0.0;

for k = 1:numel(t)-1
    e = x(:, k) - [thetaRef; 0];
    uRaw(k) = -K * e;

    uLimited = clampScalar(uRaw(k), -torqueLimit, torqueLimit);
    torqueClampActive(k) = (abs(uLimited - uRaw(k)) > 1e-12);

    maxDeltaU = torqueSlewRate * Ts;
    uSlew = clampScalar(uLimited, uLast - maxDeltaU, uLast + maxDeltaU);
    slewActive(k) = (abs(uSlew - uLimited) > 1e-12);

    u(k) = uSlew;
    uLast = u(k);
    x(:, k+1) = Ad * x(:, k) + Bd * u(k);
end
u(end) = u(end-1);
uRaw(end) = uRaw(end-1);
torqueClampActive(end) = torqueClampActive(end-1);
slewActive(end) = slewActive(end-1);

thetaDeg = rad2deg(x(1, :));
omegaRadS = x(2, :);
thetaErrDeg = thetaDeg - stepRefDeg;
settleBandDeg = settleBandRatio * max(abs(stepRefDeg), eps);
settleTime = findSettlingTime(t, thetaErrDeg, settleBandDeg);
stepDir = sign(stepRefDeg);
if stepDir == 0
    stepDir = 1;
end
overshootDeg = max(0, max(stepDir * thetaDeg) - abs(stepRefDeg));
finalErrorDeg = thetaErrDeg(end);
maxTorque = max(abs(u));

%% Print results for Ozone live-watch tuning
fprintf('================ Self-aim yaw QR-LQR tuning ================\n');
fprintf('Firmware model: J*theta_ddot + B*theta_dot = tau\n');
fprintf('J_auto          = %.9g kg*m^2\n', J);
fprintf('B_auto          = %.9g N*m*s/rad\n', Bv);
fprintf('Ts              = %.9g s\n', Ts);
fprintf('torqueLimit     = %.9g N*m\n', torqueLimit);
fprintf('torqueSlewRate  = %.9g N*m/s\n', torqueSlewRate);

fprintf('\nQR matrices used for this run:\n');
fprintf('Q =\n'); disp(Q);
fprintf('R =\n'); disp(R);
fprintf('K source = %s\n', gainSource);

fprintf('\nContinuous plant A/Bu:\n');
fprintf('A =\n'); disp(A);
fprintf('Bu =\n'); disp(Bu);
fprintf('Discrete plant Ad/Bd:\n');
fprintf('Ad =\n'); disp(Ad);
fprintf('Bd =\n'); disp(Bd);

fprintf('\nCurrent firmware self-aim K:\n');
fprintf('g_lqr_auto_k_theta = %.9gf;\n', currentKTheta);
fprintf('g_lqr_auto_k_omega = %.9gf;\n', currentKOmega);

fprintf('\nRecommended K matrix from current Q/R:\n');
fprintf('K = [%.9g, %.9g]\n', Ktheta, Komega);
fprintf('tau_fb = -K * [e_theta; e_omega]\n');

fprintf('\nOzone copy block:\n');
fprintf('g_lqr_auto_k_theta = %.9gf;\n', Ktheta);
fprintf('g_lqr_auto_k_omega = %.9gf;\n', Komega);

fprintf('\nFirmware default macro block:\n');
fprintf('#define LQR_AUTO_K_THETA_DEFAULT            %.9gf\n', Ktheta);
fprintf('#define LQR_AUTO_K_OMEGA_DEFAULT            %.9gf\n', Komega);

fprintf('\nGain ratio vs current default:\n');
fprintf('Ktheta ratio = %.6f\n', Ktheta / currentKTheta);
fprintf('Komega ratio = %.6f\n', Komega / currentKOmega);

fprintf('\nClosed-loop continuous poles:\n'); disp(polesC.');
fprintf('estimated wn   = %.9g rad/s\n', poleInfo.wn);
fprintf('estimated zeta = %.9g\n', poleInfo.zeta);
fprintf('Continuous Riccati P:\n'); disp(P);

fprintf('\nFirmware context not solved by QR:\n');
fprintf('tau_ff_alpha uses g_yaw_auto_j * alpha_ref and has its own cap.\n');
fprintf('tau_ff_viscous uses g_yaw_auto_b * omega_ref.\n');
fprintf('Coulomb FF, observer-only ESO diagnostics, LQI, bias and final clamps remain firmware-side.\n');

fprintf('\nSanity step: %.3f deg, %.3f s\n', stepRefDeg, simTimeS);
fprintf('max torque             = %.6f N*m (%.2f%% of limit)\n', ...
        maxTorque, 100.0 * maxTorque / torqueLimit);
fprintf('torque clamp samples   = %d / %d\n', nnz(torqueClampActive), numel(torqueClampActive));
fprintf('torque slew samples    = %d / %d\n', nnz(slewActive), numel(slewActive));
fprintf('overshoot              = %.6f deg\n', overshootDeg);
fprintf('final error            = %.6f deg\n', finalErrorDeg);
fprintf('settle band            = +/- %.6f deg\n', settleBandDeg);
fprintf('settle time            = %.6f s\n', settleTime);

%% Optional plots
if enablePlot
    figure('Name', 'Self-aim yaw QR-LQR saturated step');

    subplot(3, 1, 1);
    plot(t, thetaDeg, 'LineWidth', 1.2); hold on;
    yline(stepRefDeg, '--', 'LineWidth', 1.0);
    yline(stepRefDeg + settleBandDeg, ':');
    yline(stepRefDeg - settleBandDeg, ':');
    grid on;
    ylabel('theta (deg)');
    title('Self-aim yaw feedback-only step check');
    legend('theta', 'ref', '2% band', 'Location', 'best');

    subplot(3, 1, 2);
    plot(t, omegaRadS, 'LineWidth', 1.2);
    grid on;
    ylabel('omega (rad/s)');

    subplot(3, 1, 3);
    plot(t, u, 'LineWidth', 1.2); hold on;
    plot(t, uRaw, ':', 'LineWidth', 1.0);
    yline(torqueLimit, '--r');
    yline(-torqueLimit, '--r');
    grid on;
    ylabel('tau (N*m)');
    xlabel('time (s)');
    legend('limited tau', 'raw tau', 'limit', 'Location', 'best');
end

function y = clampScalar(x, lower, upper)
    y = min(max(x, lower), upper);
end

function validateQrMatrices(Q, R)
    if ~isequal(size(Q), [2, 2])
        error('Q must be a 2x2 matrix for x = [theta_error; omega_error].');
    end
    if ~isscalar(R)
        error('R must be a scalar / 1x1 matrix for yaw torque input.');
    end
    if any(~isfinite(Q(:))) || ~isfinite(R)
        error('Q and R must contain finite numeric values.');
    end
    if max(max(abs(Q - Q.'))) > 1e-9
        error('Q must be symmetric. Use Q = 0.5 * (Q + Q.'') if needed.');
    end
    if min(eig(Q)) < -1e-9
        error('Q must be positive semidefinite.');
    end
    if R <= 0.0
        error('R must be positive.');
    end
end

function settleTime = findSettlingTime(t, err, band)
    settleTime = NaN;
    inside = abs(err) <= band;
    for idx = 1:numel(t)
        if all(inside(idx:end))
            settleTime = t(idx);
            return;
        end
    end
end

function info = estimatePoleMetrics(polesC)
    finitePoles = polesC(isfinite(polesC));
    info = struct('wn', NaN, 'zeta', NaN);
    if isempty(finitePoles)
        return;
    end

    complexIdx = find(abs(imag(finitePoles)) > 1e-9, 1, 'first');
    if ~isempty(complexIdx)
        p = finitePoles(complexIdx);
        sigma = -real(p);
        wd = abs(imag(p));
        info.wn = sqrt(sigma^2 + wd^2);
        info.zeta = sigma / max(info.wn, eps);
        return;
    end

    [~, idx] = max(real(finitePoles));
    dominantPole = finitePoles(idx);
    info.wn = max(-real(dominantPole), 0.0);
    info.zeta = 1.0;
end
