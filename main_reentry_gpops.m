% main_reentry_gpops.m
clear; clc; close all;

%% --- 0) 상수 및 auxdata 설정 ---
auxdata.mu      = 3.986e14;     % [m^3/s^2] 지구 중력상수*질량
auxdata.Re      = 6378e3;       % [m] 지구 반지름
auxdata.rho0    = 1.225;        % [kg/m^3] 해수면 밀도
auxdata.H       = 8400;         % [m] 대기 스케일 높이
auxdata.mass    = 1200;         % [kg] 재진입체 질량
auxdata.Sref    = 3.0;          % [m^2] 기준 면적
auxdata.qmax    = 4e6;          % [W/m^2] 허용 최대 heat rate
auxdata.qdynMax = 320000;       % [Pa] 허용 최대 동압
auxdata.omegaE  = 7.2921159e-5; % [rad/s] 지구 자전 각속도

% Booster thrust and mass settings
auxdata.Thrust1 = 398000;       % [N]
auxdata.Thrust2 = 97000;        % [N]
auxdata.m10     = 21500;        % Stage 1 initial mass
auxdata.m20     = 5200;         % Stage 2 initial mass
auxdata.Isp1    = 237;          % [s] Stage 1 specific impulse
auxdata.Isp2    = 250;          % [s] Stage 2 specific impulse
auxdata.g0      = 9.80665;      % [m/s^2]

% Phase 경계 시간들
auxdata.t1_split = 5;           % [s]
auxdata.t1_f     = 80;          % [s]

% H0 Cost Function 관련 파라미터 (Phase 4 시작 고도 최소화에 사용)
auxdata.hCostWeight = 1.0;
auxdata.hCostScale  = 100e3;    % [m]

% 형상 타입 (1=waverider, 2=wing-body, 3=lifting-body)
auxdata.shapeType = 1;

%% --- 발사점 (위도/경도) 설정 ---
lat0_deg = 37.5;    % [deg]
lon0_deg = 127.0;   % [deg]
lat0     = deg2rad(lat0_deg);   % phi0
lon0     = deg2rad(lon0_deg);   % theta0

%% --- 목표 downrange 거리 (great-circle 거리 기준) ---
downrange_targets_km = [1500, 2000, 2500];   % [km]  
downrange_targets_m  = downrange_targets_km * 1e3;
numTargets           = numel(downrange_targets_m);

% 타겟 리스트: "위도는 발사 위도와 동일", 경도는 각 downrange에 맞게 계산
targetList = struct([]);
for i = 1:numTargets
    L     = downrange_targets_m(i);  % [m]
    sigma = L / auxdata.Re;          % 중앙각 [rad]

    phi0   = lat0;
    theta0 = lon0;

    % great-circle 거리 sigma, 최종 위도 = phi0를 만족하도록 Δtheta 계산
    cosSigma      = cos(sigma);
    sphi0         = sin(phi0);
    cphi0         = cos(phi0);
    % cos(sigma) = sin^2(phi0) + cos^2(phi0)*cos(Δtheta)
    cosDeltaTheta = (cosSigma - sphi0^2) / (cphi0^2);
    cosDeltaTheta = max(min(cosDeltaTheta, 1.0), -1.0);
    deltaTheta    = acos(cosDeltaTheta);

    targetList(i).phi   = phi0;               % 위도 고정
    targetList(i).theta = theta0 + deltaTheta;% 동쪽 방향으로 이동
end

%% --- 1) 상태/제어 정의 ---
% 상태: x = [r, theta, phi, V, gamma, psi]
% 제어: u = [alpha, sigma]

%% --- 2) 시간 구간 및 공통 bounds 정의 ---
t1_0     = 0;       % Phase 1 시작
t1_split = 5;       % Phase 1 종료
t1_f     = 80;      % Phase 2 종료

t3_f_nom = 160;     % Phase 3 nominal 종료 (guess용)
t3_f_min = 140;     % Phase 3 종료시간 하한 (자유도)
t3_f_max = 180;     % Phase 3 종료시간 상한

tf_max   = 2000;    % 최종 시간 upper bound

% 초기/최종 반경
h0 = 0;  hf = 0;
r0 = auxdata.Re + h0;
rf = auxdata.Re + hf;

V0 = 0;
Vf = 720;           % 최종 속도 목표 (reentryEndpoint에서 rf, Vf, gammaf 고정)

gamma0 = deg2rad(90);   % 수직 발사
gammaf = deg2rad(-80);  % 최종 FPA
psi0   = deg2rad(90);   % 동쪽 향

% 상태 bounds
rMin = auxdata.Re;        rMax = auxdata.Re + 120e3;
thetaMin = -pi;           thetaMax =  pi;
phiMin   = -pi/2;         phiMax   =  pi/2;
Vmin = 0;                 Vmax = 4000;
gammaMin = -pi/2;         gammaMax =  pi/2;
psiMin = 0;               psiMax = deg2rad(100);

% 제어 bounds
alphaMin_boost = deg2rad(-20);
alphaMax_boost = deg2rad(20);

switch auxdata.shapeType
    case 1  % Waverider
        alphaMin_4 = deg2rad(-10);
        alphaMax_4 = deg2rad(50);
    case 2  % Wing-body
        alphaMin_4 = deg2rad(-10);
        alphaMax_4 = deg2rad(50);
    case 3  % Lifting-body
        alphaMin_4 = deg2rad(0);
        alphaMax_4 = deg2rad(30);
    otherwise
        alphaMin_4 = deg2rad(-10);
        alphaMax_4 = deg2rad(50);
end

sigmaMin_all = deg2rad(-90);
sigmaMax_all = deg2rad( 90);

qdotMin = 0;                  qdotMax = auxdata.qmax;
qdynMin = 0;                  qdynMax = auxdata.qdynMax;

%% --- 상단부에서 고정하고 싶은 boost AOA 설정 ---
alpha_phase2_fixed = deg2rad(5);   % Phase 2 고정 AOA (원하면 조절)
alpha_phase3_fixed = deg2rad(3);   % Phase 3 고정 AOA (원하면 조절)

%% --- 2-1) PHASE 1: 0 ~ 5 s (완전 고정) ---
bounds.phase(1).initialtime.lower = t1_0;
bounds.phase(1).initialtime.upper = t1_0;
bounds.phase(1).finaltime.lower   = t1_split;
bounds.phase(1).finaltime.upper   = t1_split;

bounds.phase(1).initialstate.lower = [r0, lon0, lat0, V0, gamma0, psi0];
bounds.phase(1).initialstate.upper = [r0, lon0, lat0, V0, gamma0, psi0];

bounds.phase(1).state.lower = [rMin, thetaMin, phiMin, Vmin, gammaMin, psiMin];
bounds.phase(1).state.upper = [rMax, thetaMax, phiMax, Vmax, gammaMax, psiMax];

bounds.phase(1).finalstate.lower = bounds.phase(1).state.lower;
bounds.phase(1).finalstate.upper = bounds.phase(1).state.upper;

bounds.phase(1).control.lower = [0, 0];   % AOA=0, bank=0
bounds.phase(1).control.upper = [0, 0];

%% --- 2-2) PHASE 2: 5 ~ 80 s (시간 고정, AOA 상수) ---
bounds.phase(2).initialtime.lower = t1_split;
bounds.phase(2).initialtime.upper = t1_split;
bounds.phase(2).finaltime.lower   = t1_f;
bounds.phase(2).finaltime.upper   = t1_f;

bounds.phase(2).initialstate.lower = [rMin, thetaMin, phiMin, Vmin, gammaMin, psiMin];
bounds.phase(2).initialstate.upper = [rMax, thetaMax, phiMax, Vmax, gammaMax, psiMax];

bounds.phase(2).state.lower = bounds.phase(2).initialstate.lower;
bounds.phase(2).state.upper = bounds.phase(2).initialstate.upper;

bounds.phase(2).finalstate.lower = bounds.phase(2).state.lower;
bounds.phase(2).finalstate.upper = bounds.phase(2).state.upper;

bounds.phase(2).control.lower = [alpha_phase2_fixed, 0];
bounds.phase(2).control.upper = [alpha_phase2_fixed, 0];

%% --- 2-3) PHASE 3: 80 ~ t3_f (final time만 자유, AOA 상수) ---
bounds.phase(3).initialtime.lower = t1_f;
bounds.phase(3).initialtime.upper = t1_f;

bounds.phase(3).finaltime.lower = 140;     % 예: 140초 ~ 220초 사이
bounds.phase(3).finaltime.upper = 220;

%bounds.phase(3).finaltime.lower   = t3_f_min;   % 자유 DOF
%bounds.phase(3).finaltime.upper   = t3_f_max;

bounds.phase(3).initialstate.lower = [rMin, thetaMin, phiMin, Vmin, gammaMin, psiMin];
bounds.phase(3).initialstate.upper = [rMax, thetaMax, phiMax, Vmax, gammaMax, psiMax];

bounds.phase(3).state.lower = bounds.phase(3).initialstate.lower;
bounds.phase(3).state.upper = bounds.phase(3).initialstate.upper;

bounds.phase(3).finalstate.lower = bounds.phase(3).state.lower;
bounds.phase(3).finalstate.upper = bounds.phase(3).state.upper;

% 🔹 AoA 상수, bank=0
bounds.phase(3).control.lower = [alpha_phase3_fixed, 0];
bounds.phase(3).control.upper = [alpha_phase3_fixed, 0];

%% --- 2-4) PHASE 4: 160 ~ tf (재진입/글라이드 최적화) ---
%bounds.phase(4).initialtime.lower = t3_f_min;
%bounds.phase(4).initialtime.upper = t3_f_max;
bounds.phase(4).initialtime.lower = 0;   % free
bounds.phase(4).initialtime.upper = tf_max;


bounds.phase(4).finaltime.lower   = t3_f_min;
bounds.phase(4).finaltime.upper   = tf_max;

bounds.phase(4).initialstate.lower = [rMin, thetaMin, phiMin, Vmin, gammaMin, psiMin];
bounds.phase(4).initialstate.upper = [rMax, thetaMax, phiMax, Vmax, gammaMax, psiMax];

bounds.phase(4).state.lower = bounds.phase(4).initialstate.lower;
bounds.phase(4).state.upper = bounds.phase(4).initialstate.upper;

% finalstate는 각 타겟마다 루프 안에서 설정

bounds.phase(4).control.lower = [alphaMin_4, sigmaMin_all];
bounds.phase(4).control.upper = [alphaMax_4, sigmaMax_all];

bounds.phase(4).path.lower     = [qdotMin, qdynMin];
bounds.phase(4).path.upper     = [qdotMax, qdynMax];
bounds.phase(4).integral.lower = 0;
bounds.phase(4).integral.upper = 1e10;

%% --- 3) 초기 guess 설정 ---

% Phase 1 guess
tGuess1 = [t1_0; t1_split];
rGuess1     = [r0; r0 + 5e3];
thetaGuess1 = [lon0; lon0];
phiGuess1   = [lat0; lat0];
VGuess1     = [V0; 500];
gammaGuess1 = [deg2rad(90); deg2rad(90)];
psiGuess1   = [psi0; psi0];

stateGuess1   = [rGuess1, thetaGuess1, phiGuess1, VGuess1, gammaGuess1, psiGuess1];
controlGuess1 = [zeros(2,1), zeros(2,1)];

guess.phase(1).time    = tGuess1;
guess.phase(1).state   = stateGuess1;
guess.phase(1).control = controlGuess1;

% Phase 2 guess
tGuess2 = [t1_split; t1_f];
rGuess2     = [r0 + 5e3; r0 + 40e3];
thetaGuess2 = [thetaGuess1(end); thetaGuess1(end) + 100e3/auxdata.Re];
phiGuess2   = [lat0; lat0];
VGuess2     = [500; 2500];
gammaGuess2 = [deg2rad(80); deg2rad(80)];
psiGuess2   = [psi0; psi0];

stateGuess2  = [rGuess2, thetaGuess2, phiGuess2, VGuess2, gammaGuess2, psiGuess2];
alphaGuess2  = [alpha_phase2_fixed; alpha_phase2_fixed];
sigmaGuess2  = [0; 0];

guess.phase(2).time    = tGuess2;
guess.phase(2).state   = stateGuess2;
guess.phase(2).control = [alphaGuess2, sigmaGuess2];

% Phase 3 guess
tGuess3 = [t1_f; t3_f_nom];
rGuess3     = [r0 + 40e3; r0 + 60e3];
thetaGuess3 = [thetaGuess2(end); thetaGuess2(end) + 400e3/auxdata.Re];
phiGuess3   = [lat0; lat0];
VGuess3     = [2500; 3700];
gammaGuess3 = [deg2rad(80); deg2rad(5)];
psiGuess3   = [psi0; psi0];

stateGuess3 = [rGuess3, thetaGuess3, phiGuess3, VGuess3, gammaGuess3, psiGuess3];
alphaGuess3 = [alpha_phase3_fixed; alpha_phase3_fixed];
sigmaGuess3 = [0; 0];

guess.phase(3).time    = tGuess3;
guess.phase(3).state   = stateGuess3;
guess.phase(3).control = [alphaGuess3, sigmaGuess3];

% Phase 4 guess
tGuess4 = [guess.phase(3).time(end); 1700];
rGuess4     = [r0 + 60e3; rf];
thetaGuess4 = [thetaGuess3(end); thetaGuess3(end) + 2000e3/auxdata.Re];
phiGuess4   = [lat0; lat0];
VGuess4     = [3700; Vf];
gammaGuess4 = [deg2rad(5); gammaf];
psiGuess4   = [psi0; psi0];

stateGuess4  = [rGuess4, thetaGuess4, phiGuess4, VGuess4, gammaGuess4, psiGuess4];
alphaGuess4  = linspace(deg2rad(15), deg2rad(20), 2).';
sigmaGuess4  = zeros(2,1);

guess.phase(4).time     = tGuess4;
guess.phase(4).state    = stateGuess4;
guess.phase(4).control  = [alphaGuess4, sigmaGuess4];
guess.phase(4).integral = 0;

%% --- 4) GPOPS 설정 ---
setup.name = 'ReentryCapsule_4Phase_MultiDownrange';

setup.functions.continuous = @reentryContinuous;
setup.functions.endpoint   = @reentryEndpoint;

setup.mesh.method       = 'hp-PattersonRao';
setup.mesh.tolerance    = 1e-3;
setup.mesh.maxiteration = 15;

setup.scales.method = 'automatic-bounds';

setup.nlp.solver                = 'ipopt';
setup.nlp.options.tolerance     = 1e-6;
setup.nlp.options.maxiterations = 5000;

% Event group: [rf, Vf, gammaf] equalities (reentryEndpoint에서 event1(1:3))
bounds.eventgroup(1).lower = [0 0 0];
bounds.eventgroup(1).upper = [0 0 0];

% Phase 1→2, 2→3, 3→4 연속성 (x=[r,theta,phi,V,gamma,psi])
bounds.eventgroup(2).lower = zeros(1,6);
bounds.eventgroup(2).upper = zeros(1,6);
bounds.eventgroup(3).lower = zeros(1,6);
bounds.eventgroup(3).upper = zeros(1,6);
bounds.eventgroup(4).lower = zeros(1,6);
bounds.eventgroup(4).upper = zeros(1,6);

setup.bounds  = bounds;
setup.guess   = guess;
setup.auxdata = auxdata;

setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.derivatives.dependencies    = 'sparseNaN';

%% --- 5) 타겟별로 gpops2 실행 ---
solutions = cell(numTargets,1);

for i = 1:numTargets
    thetaT = targetList(i).theta;
    phiT   = targetList(i).phi;

    % Phase 4 final state: r=Re, theta=thetaT, phi=phiT, V=Vf, gamma=-80deg
    bounds.phase(4).finalstate.lower = [rf, thetaT, phiT, Vf, deg2rad(-80), psiMin];
    bounds.phase(4).finalstate.upper = [rf, thetaT, phiT, Vf, deg2rad(-80), psiMax];

    setup.bounds  = bounds;
    setup.auxdata = auxdata;

    fprintf('\n=== Target %d: downrange = %.0f km, lat = %.2f deg ===\n', ...
        i, downrange_targets_km(i), lat0_deg);
    fprintf('thetaT (deg) = %.3f\n', rad2deg(thetaT));

    output       = gpops2(setup);
    sol          = output.result.solution;
    solutions{i} = sol;

    xf = sol.phase(4).state(end,:);
    rf_sol     = xf(1);
    thetaf_sol = xf(2);
    phif_sol   = xf(3);
    Vf_sol     = xf(4);
    gammaf_sol = xf(5);

    % 실제 downrange (great-circle) 계산
    phi0   = lat0;
    theta0 = lon0;
    R      = auxdata.Re;

    cosSigma = sin(phi0)*sin(phif_sol) + cos(phi0)*cos(phif_sol)*cos(thetaf_sol-theta0);
    cosSigma = max(min(cosSigma,1),-1);
    sigma    = acos(cosSigma);
    downrange_km_real = R * sigma / 1000;

    fprintf('  rf     = %.2f m\n', rf_sol);
    fprintf('  Vf     = %.2f m/s\n', Vf_sol);
    fprintf('  gammaf = %.3f deg\n', rad2deg(gammaf_sol));
    fprintf('  Final lat = %.3f deg, lon = %.3f deg\n', rad2deg(phif_sol), rad2deg(thetaf_sol));
    fprintf('  Great-circle downrange = %.2f km\n', downrange_km_real);

    %% --- Phase 4 결과를 CSV로 저장 (time / altitude / range / crossrange / heat-rate) ---
    t4  = sol.phase(4).time;
    x4  = sol.phase(4).state;
    r4  = x4(:,1);
    th4 = x4(:,2);
    ph4 = x4(:,3);
    V4  = x4(:,4);

    h4 = r4 - auxdata.Re;   % [m]

    % density
    rho4 = auxdata.rho0 .* exp(-h4./auxdata.H);

    % heat-rate
    rho_sqrt = sqrt(rho4);
    V3       = V4.^3.05;
    switch auxdata.shapeType
        case 1
            C_WR = 1.1813e-3;
            qdot4 = C_WR .* rho_sqrt .* V3;
        case {2,3}
            C_FIN = 9.12e-4;
            AF = deg2rad(45);
            factor = (1 - 0.18*sin(AF)^2)*cos(AF);
            qdot4 = C_FIN .* rho_sqrt .* V3 .* factor;
    end

    % downrange (great-circle) vs time
    phi0   = lat0;
    theta0 = lon0;
    cosSigma_t = sin(phi0).*sin(ph4) + cos(phi0).*cos(ph4).*cos(th4-theta0);
    cosSigma_t = max(min(cosSigma_t,1),-1);
    sigma_t    = acos(cosSigma_t);
    range_t    = R * sigma_t;   % [m]

    % crossrange ≈ R * |phi - phi0|
    cross_t = R * abs(ph4 - phi0); % [m]

    T = table( ...
        t4, ...
        h4/1000, ...          % [km]
        range_t/1000, ...     % [km]
        cross_t/1000, ...     % [km]
        qdot4, ...            % [W/m^2]
        'VariableNames', {'time_s','altitude_km','range_km','crossrange_km','heatRate_W_m2'} ...
    );

    fname = sprintf('traj_downrange_%dkm.csv', round(downrange_targets_km(i)));
    writetable(T, fname);
    fprintf('  -> CSV saved: %s\n', fname);
end

%% --- 6) 플롯: Phase 4 기준 Heat-rate / Altitude / Range / Crossrange ---

numCases    = numTargets;
qdot_all    = cell(numCases,1);
time_all4   = cell(numCases,1);
h_all4      = cell(numCases,1);
range_all4  = cell(numCases,1);
cross_all4  = cell(numCases,1);

phi0   = lat0;
theta0 = lon0;
R      = auxdata.Re;

for i = 1:numCases
    sol = solutions{i};
    t4  = sol.phase(4).time;
    x4  = sol.phase(4).state;
    r4  = x4(:,1);
    th4 = x4(:,2);
    ph4 = x4(:,3);
    V4  = x4(:,4);

    h4 = r4 - auxdata.Re;
    rho4 = auxdata.rho0 .* exp(-h4./auxdata.H);
    rho_sqrt = sqrt(rho4);
    V3 = V4.^3.05;

    switch auxdata.shapeType
        case 1
            C_WR = 1.1813e-3;
            qdot = C_WR .* rho_sqrt .* V3;
        case {2,3}
            C_FIN = 9.12e-4;
            AF    = deg2rad(45);
            factor = (1 - 0.18*sin(AF)^2)*cos(AF);
            qdot  = C_FIN .* rho_sqrt .* V3 .* factor;
    end

    cosSigma_t = sin(phi0).*sin(ph4) + cos(phi0).*cos(ph4).*cos(th4-theta0);
    cosSigma_t = max(min(cosSigma_t,1),-1);
    sigma_t    = acos(cosSigma_t);
    range_t    = R * sigma_t;         % [m]
    cross_t    = R * abs(ph4 - phi0); % [m]

    time_all4{i}   = t4;
    h_all4{i}      = h4/1000;
    qdot_all{i}    = qdot;
    range_all4{i}  = range_t/1000;
    cross_all4{i}  = cross_t/1000;
end

labels = arrayfun(@(L)sprintf('%.0f km',L), downrange_targets_km,'UniformOutput',false);

% Heat-rate
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all4{i}, qdot_all{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Heat-rate q̇ [W/m^2]');
legend(labels,'Location','best');
title('Heat-rate vs Time (Phase 4)');

% Altitude
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all4{i}, h_all4{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Altitude [km]');
legend(labels,'Location','best');
title('Altitude vs Time (Phase 4)');

% Range
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all4{i}, range_all4{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Range from launch [km]');
legend(labels,'Location','best');
title('Range vs Time (Phase 4)');

% Crossrange
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all4{i}, cross_all4{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Crossrange [km]');
legend(labels,'Location','best');
title('Crossrange vs Time (Phase 4)');

%% --- 7) Phase 1~4 전체 궤적 Altitude/Range 플롯 ---

time_all_traj     = cell(numCases,1);
altitude_all_traj = cell(numCases,1);
range_all_traj    = cell(numCases,1);

for i = 1:numCases
    sol = solutions{i};

    t_all     = [];
    r_all     = [];
    theta_all = [];
    phi_all   = [];

    for p = 1:4
        t_p = sol.phase(p).time;
        x_p = sol.phase(p).state;
        t_all     = [t_all;     t_p];
        r_all     = [r_all;     x_p(:,1)];
        theta_all = [theta_all; x_p(:,2)];
        phi_all   = [phi_all;   x_p(:,3)];
    end

    h_all = (r_all - auxdata.Re)/1000;   % [km]

    phi0   = lat0;
    theta0 = lon0;
    cosSigma = sin(phi0).*sin(phi_all) + cos(phi0).*cos(phi_all).*cos(theta_all-theta0);
    cosSigma = max(min(cosSigma,1),-1);
    sigma_all = acos(cosSigma);
    range_all_km = auxdata.Re * sigma_all / 1000;

    time_all_traj{i}     = t_all;
    altitude_all_traj{i} = h_all;
    range_all_traj{i}    = range_all_km;
end

% Altitude vs Time (Phase 1~4)
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all_traj{i}, altitude_all_traj{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Altitude [km]');
legend(labels,'Location','best');
title('Altitude vs Time (Phase 1–4)');

% Range vs Time (Phase 1~4)
figure; hold on; grid on;
for i = 1:numCases
    plot(time_all_traj{i}, range_all_traj{i}, 'LineWidth', 2);
end
xlabel('Time [s]');
ylabel('Great-circle Range [km]');
legend(labels,'Location','best');
title('Range vs Time (Phase 1–4)');
