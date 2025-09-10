%% ===== Realistic PID vs Open-loop Simulation (Matching ESP32) =====
clear; clc; close all;

%% --- Motor & simulation parameters (MATCHING ESP32) ---
PWMmax = 4095; PWMmin = 0;
targetRPM = 300;        % Desired motor speed
tau = 0.25;             % Motor time constant [s] - more realistic for brushed DC
dt = 0.05;              % Control period - MATCHING ESP32 (50ms)
Tsim = 20;              % Total simulation time
time = 0:dt:Tsim;

% Motor gain: calibrated so ~3000 PWM gives 300 RPM at steady state
K_m = 0.1;              % RPM per PWM unit

%% --- PID parameters (MATCHING ESP32 EXACTLY) ---
Kp = 2.5;      % proportional
Ki = 8.0;      % integral  
Kd = 0.01;     % derivative
DERIV_ALPHA = 0.7;     % derivative filter
INTEG_MAX = 5000;
INTEG_MIN = -5000;

%% --- Realistic open-loop PWM calculation ---
% Calculate what PWM would theoretically give target RPM at steady state
PWM_theoretical = targetRPM / K_m;  % = 3000 for 300 RPM
% But in reality, you might guess wrong, so add some error
PWM_OL = PWM_theoretical * 0.8;     % 20% underestimate (common mistake)
RPM_OL = zeros(size(time));

%% --- Preallocate arrays for PID ---
RPM = zeros(size(time));
PWM = zeros(size(time));
error_history = zeros(size(time));
integ_history = zeros(size(time));
deriv_history = zeros(size(time));

% Initialize PID state variables (matching ESP32)
integ = 0; 
derivFiltered = 0;
rpmPrev = 0;

%% --- Simulation loop (MATCHING ESP32 LOGIC EXACTLY) ---
for k = 2:length(time)
    % PID error calculation
    error = targetRPM - RPM(k-1);
    error_history(k) = error;
    
    % Derivative on measurement (matching ESP32)
    derivRaw = -(RPM(k-1) - rpmPrev)/dt;
    derivFiltered = DERIV_ALPHA*derivFiltered + (1-DERIV_ALPHA)*derivRaw;
    deriv_history(k) = derivFiltered;
    
    % Integrator anti-windup (matching ESP32 logic exactly)
    integCandidate = integ + error*dt;
    if integCandidate > INTEG_MAX
        integCandidate = INTEG_MAX;
    end
    if integCandidate < INTEG_MIN
        integCandidate = INTEG_MIN;
    end
    
    % Check if control output would be within bounds (ESP32 logic)
    u_unclamped = Kp*error + Ki*integCandidate + Kd*derivFiltered;
    if u_unclamped >= 0.0 && u_unclamped <= PWMmax
        integ = integCandidate;  % Commit integrator only if output not saturated
    end
    integ_history(k) = integ;
    
    % Final control output with clamping (matching ESP32)
    u = Kp*error + Ki*integ + Kd*derivFiltered;
    if u < 0.0
        u = 0.0;
    end
    if u > PWMmax
        u = PWMmax;
    end
    PWM(k) = u;
    
    % Motor dynamics (first-order system)
    RPM(k) = RPM(k-1) + dt*(K_m*PWM(k) - RPM(k-1))/tau;
    
    % Update previous RPM for derivative calculation
    rpmPrev = RPM(k-1);
    
    % Open-loop response for comparison
    RPM_OL(k) = RPM_OL(k-1) + dt*(K_m*PWM_OL - RPM_OL(k-1))/tau;
end

%% --- Enhanced Plotting ---
figure('Position',[100 100 1200 900]);

% Main response comparison
subplot(2,2,1);
plot(time, RPM, 'b-', 'LineWidth', 2.5); hold on;
plot(time, RPM_OL, 'r--', 'LineWidth', 2);
yline(targetRPM, 'k--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('RPM'); grid on;
legend('PID Response', 'Open-loop (80% of ideal)', 'Target RPM', 'Location', 'best');
title('PID vs Open-loop Motor Response');
ylim([0 max(400, max(RPM)*1.1)]);

% PWM comparison
subplot(2,2,2);
plot(time, PWM, 'm-', 'LineWidth', 2); hold on;
yline(PWM_OL, 'r--', 'LineWidth', 2);
yline(PWM_theoretical, 'g:', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('PWM Duty'); grid on;
legend('PID PWM', 'Open-loop PWM', 'Ideal PWM', 'Location', 'best');
title('PWM Control Signals');
ylim([0 PWMmax]);

% Error over time
subplot(2,2,3);
plot(time, error_history, 'g-', 'LineWidth', 2); hold on;
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Error [RPM]'); grid on;
title('Control Error Over Time');

% PID component contributions
subplot(2,2,4);
plot(time, Kp*error_history, 'r-', 'LineWidth', 1.5); hold on;
plot(time, Ki*integ_history, 'b-', 'LineWidth', 1.5);
plot(time, Kd*deriv_history, 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Control Contribution'); grid on;
legend('P term', 'I term', 'D term', 'Location', 'best');
title('PID Component Analysis');

%% --- Performance metrics ---
[peakRPM, peakIdx] = max(RPM);
overshoot = max(0, (peakRPM - targetRPM)/targetRPM*100);

% Settling time (within 2% of target)
settleThreshold = 0.02 * targetRPM;
settleIdx = find(abs(RPM(round(end/2):end) - targetRPM) <= settleThreshold, 1, 'first');
if ~isempty(settleIdx)
    settlingTime = time(settleIdx + round(length(time)/2) - 1);
else
    settlingTime = NaN;
    fprintf('Warning: System did not settle within 2%% of target\n');
end

% Rise time (10% to 90% of final value)
finalValue = mean(RPM(end-20:end));
riseStart = find(RPM >= 0.1*finalValue, 1, 'first');
riseEnd = find(RPM >= 0.9*finalValue, 1, 'first');
if ~isempty(riseStart) && ~isempty(riseEnd)
    riseTime = time(riseEnd) - time(riseStart);
else
    riseTime = NaN;
end

% Steady-state errors
steadyStateError_PID = targetRPM - mean(RPM(end-20:end));
steadyStateError_OL = targetRPM - mean(RPM_OL(end-20:end));

% Control effort (average PWM in steady state)
steadyStatePWM = mean(PWM(end-20:end));

%% --- Detailed Results ---
fprintf('\n=== SIMULATION RESULTS ===\n');
fprintf('Target RPM: %.1f\n', targetRPM);
fprintf('Control Period: %.0f ms (matching ESP32)\n', dt*1000);
fprintf('Motor Time Constant: %.2f s\n', tau);
fprintf('Motor Gain: %.3f RPM/PWM\n', K_m);

fprintf('\n--- PID Performance ---\n');
fprintf('Final RPM: %.2f (%.1f%% of target)\n', RPM(end), RPM(end)/targetRPM*100);
fprintf('Overshoot: %.2f%%\n', overshoot);
fprintf('Rise Time: %.2f s\n', riseTime);
fprintf('Settling Time (2%%): %.2f s\n', settlingTime);
fprintf('Steady-state Error: %.2f RPM (%.2f%%)\n', steadyStateError_PID, abs(steadyStateError_PID)/targetRPM*100);
fprintf('Steady-state PWM: %.0f (%.1f%% of max)\n', steadyStatePWM, steadyStatePWM/PWMmax*100);
fprintf('Max PWM reached: %.0f\n', max(PWM));

fprintf('\n--- Open-loop Performance ---\n');
fprintf('Open-loop PWM: %.0f (%.1f%% of theoretical)\n', PWM_OL, PWM_OL/PWM_theoretical*100);
fprintf('Final RPM: %.2f (%.1f%% of target)\n', RPM_OL(end), RPM_OL(end)/targetRPM*100);
fprintf('Steady-state Error: %.2f RPM (%.2f%%)\n', steadyStateError_OL, abs(steadyStateError_OL)/targetRPM*100);

fprintf('\n--- PID Gain Analysis ---\n');
fprintf('Kp = %.2f, Ki = %.2f, Kd = %.4f\n', Kp, Ki, Kd);
fprintf('Final integrator value: %.1f\n', integ);
if max(PWM) >= 0.98*PWMmax
    fprintf('WARNING: PWM saturated - consider reducing gains\n');
end
if overshoot > 20
    fprintf('WARNING: High overshoot - consider reducing Kp or Ki\n');
end
if abs(steadyStateError_PID) > 0.05*targetRPM
    fprintf('WARNING: Large steady-state error - consider increasing Ki\n');
end

%% --- Stability margin estimate ---
% Rough estimate based on maximum control effort
controlEffortMargin = (PWMmax - max(PWM)) / PWMmax * 100;
fprintf('\nControl effort margin: %.1f%% (headroom before saturation)\n', controlEffortMargin);