function output = reentryContinuous(input)
% reentryContinuous.m
%
% 4-Phase Dynamics
%
% Phase 1 : Stage-1 boost first 5 sec (AOA=0, sigma=0)
% Phase 2 : Stage-1 boost remainder
% Phase 3 : Stage-2 boost
% Phase 4 : Free-flight + Reentry (aero + heat-rate)
%
% state = [r, theta, phi, V, gamma, psi]
% control = [alpha, sigma]

aux = input.auxdata;

% auxdata에서 추출 (for 루프 밖에서 정의)
t1_split = aux.t1_split;
t1_f     = aux.t1_f;

omegaE = aux.omegaE;

for p = 1:length(input.phase)
    x = input.phase(p).state;
    u = input.phase(p).control;
    t = input.phase(p).time; 

    r     = x(:,1);
    theta   = x(:,2);
    phi = x(:,3);
    V     = x(:,4);
    gamma = x(:,5);
    psi   = x(:,6);
    alpha = u(:,1);   % AOA
    sigma = u(:,2);   % bank angle

    h   = r - aux.Re;
    rho = aux.rho0 .* exp(-h./aux.H);     % atmospheric density
    g   = aux.mu ./ r.^2;                 % gravity

    %% ============================================================
    % PHASE-SPECIFIC DYNAMICS
    % ============================================================

    %% ============================
    % Phase 1 : Stage 1 boost (0~5s)
    % ============================
    if p == 1
        T = aux.Thrust1;
        m = aux.m10 + (-T/(aux.Isp1*aux.g0)).*t;
        L = 0;  D = 0;

        %% ============================
        % Phase 2 : Stage-1 boost remainder (5~80s)
        % ============================
    elseif p == 2
        T = aux.Thrust1;
        dt = t - t1_split;
        m = aux.m10 + (-T/(aux.Isp1*aux.g0)).*dt;
        L = 0;  D = 0;

        %% ============================
        % Phase 3 : Stage-2 boost (80~160s)
        % ============================
    elseif p == 3
        T = aux.Thrust2;
        dt = t - t1_f;
        m = aux.m20 + (-T/(aux.Isp2*aux.g0)).*dt;
        L = 0; D = 0;

        %% ============================
        % Phase 4 : Reentry/Glide
        % ============================
    elseif p == 4
        T = 0;
        m = aux.mass;        

        [CL, CD] = aeroCoeff(alpha, aux.shapeType);
        qdyn = 0.5 .* rho .* V.^2;
        L = qdyn .* aux.Sref .* CL;
        D = qdyn .* aux.Sref .* CD;
    end

    epsilon = 1e-6;

    V_stable        = V + epsilon;
    cosGamma        = cos(gamma);
    cosGamma_stable = cosGamma + epsilon;

    % rdot
    rdot = V .* sin(gamma);

    % thetadot (longitude)
    thetadot = V .* cos(gamma) .* sin(psi) ./ (r .* cos(phi));

    % phidot (latitude)
    phidot = V .* cos(gamma) .* cos(psi) ./ r;

    % Vdot (speed)
    Vdot = (T./m).*cos(alpha) - D./m - g.*sin(gamma) ...
        + omegaE.^2 .* r .* cos(phi) .* ...
        ( sin(gamma).*cos(phi) - cos(gamma).*sin(phi).*cos(psi) );

    % gammadot (flight-path or pitch angle)
    gammadot = ...
        ( ( (T./m).*sin(alpha) + L./m ).*cos(sigma) ...
        - ( g - V.^2 ./ r ) .* cos(gamma))./V_stable ...
        + 2*omegaE.*cos(phi).*sin(psi) ...
        + ((omegaE.^2 .* r .* cos(phi)) .* ...
        ( cos(gamma).*cos(phi) + sin(gamma).*sin(phi).*cos(psi) ))./V_stable;

    % psidot (heading angle)
    psidot = ...
        ( (T./m).*sin(alpha) + L./m ) .* (sin(sigma)./cosGamma_stable) ./ V_stable ...
        + (V.*cos(gamma)./r) .* sin(psi).*tan(phi) ...
        - 2*omegaE .* ( (sin(gamma)./cosGamma_stable).*cos(phi).*cos(psi) - sin(phi) ) ...
        + (omegaE.^2 .* r ./ (V_stable.*cosGamma_stable)) .* sin(phi).*cos(phi).*sin(psi);
    
    % === Reordered dynamics: [r, theta, phi, V, gamma, psi]
    dynamics = [rdot, thetadot, phidot, Vdot, gammadot, psidot];


    %% ============================================================
    % PATH & INTEGRAND
    % ============================================================
    if p == 4
        rho_sqrt = sqrt(rho);

        switch aux.shapeType
            case 1   % Waverider
                C_WR = 1.1813e-3;
                % Rn = aux.Rn;    % 0.006 m
                qdot = C_WR .* rho_sqrt .* V.^3.05;
            case {2,3}  % Wing-body / Lifting-body
                C_FIN = 9.12e-4;            % Table 2 constant
                AF = deg2rad(45);           % sweep angle
                factor = (1 - 0.18 * sin(AF).^2) * cos(AF);
                qdot = C_FIN .* rho_sqrt .* V^3 .* factor;
            otherwise
                error('Unknown shapeType for heat-rate computation.');
        end
        qdyn_val = 0.5 .* rho .* V.^2;
        phaseout(p).path = [qdot, qdyn_val];
        phaseout(p).integrand = qdot;
    else
        % No path constraints for phases 1~3
        % phaseout(p).path = zeros(size(r,1),2);
        qdot_dummy = zeros(size(r));
        qdyn_dummy = zeros(size(r));
        phaseout(p).path = [qdot_dummy, qdyn_dummy];
        phaseout(p).integrand = zeros(size(r,1),1);
    end

    %% Store dynamics
    phaseout(p).dynamics = dynamics;

end

output = phaseout;

end

%% ============================================================
% Aerodynamic Coefficients
% ============================================================
function [CL, CD] = aeroCoeff(alpha, shapeType)
switch shapeType
    case 1  % Waverider
        CL = -0.03 + 0.75 .* alpha;
        CD = 0.012 - 0.01 .* alpha + 0.6 .* alpha.^2;
    case 2  % Wing-body
        CL = -0.034 + 0.93 .* alpha;
        CD = 0.037 - 0.01 .* alpha + 0.736 .* alpha.^2 + 0.937 .* alpha.^3;
    case 3  % Lifting-body (HL-20)
        CL = -0.06 + 0.8 .* alpha;
        CD = 0.075 - 0.47 .* alpha + 2.7 .* alpha.^2 - 0.68 .* alpha.^3;
end
end