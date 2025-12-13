function output = reentryEndpoint(input)

aux = input.auxdata;

%% ============================================================
% PHASE 4 FINAL STATE (제약 조건)
% ============================================================
xf = input.phase(4).finalstate;

rf     = xf(1);
Vf     = xf(4);
gammaf = xf(5);

%% Phase 4 terminal requirements
rf_des    = aux.Re;
Vf_des    = 720;
gamma_des = deg2rad(-80);

event1(1) = rf     - rf_des;
event1(2) = Vf     - Vf_des;
event1(3) = gammaf - gamma_des;

%% ============================================================
% PHASE CONTINUITY (state 연속성)
% ============================================================

% Phase 1 -> 2
event2 = input.phase(1).finalstate - input.phase(2).initialstate;

% Phase 2 -> 3
event3 = input.phase(2).finalstate - input.phase(3).initialstate;

% Phase 3 -> 4
event4 = input.phase(3).finalstate - input.phase(4).initialstate;

%% ============================================================
% OBJECTIVE FUNCTION : Phase 3 종료 시간 t3_f 최소화
% ============================================================
t3_f = input.phase(3).finaltime;
J = t3_f;        % free-time optimization

%% ============================================================
% OUTPUT
% ============================================================
output.objective = J;
output.eventgroup(1).event = event1;
output.eventgroup(2).event = event2;
output.eventgroup(3).event = event3;
output.eventgroup(4).event = event4;

end
