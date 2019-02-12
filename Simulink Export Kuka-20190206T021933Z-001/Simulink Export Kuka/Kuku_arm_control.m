%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Developed by Rishi Khajuriwala
% For RBE 594 Capstone Project
% Bedside Rehabilitation robotic system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;


%% Linearizing the Plant
% The robot arm dynamics are nonlinear. To understand whether the arm can be 
% controlled with one set of PI gains, linearize the plant at various points
% (snapshot times) along the trajectory of interest. Here "plant" refers to
% the dynamics between the control signals (outputs of PID blocks) and the
% measurement signals (output of "KUKA LBR arm model" block).

SnapshotTimes = 0:1:5;
% Plant is from PID outputs to Robot Arm outputs
LinIOs = [...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_1_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_2_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_3_PID',1,'openinput'),...        ,.
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_4_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_5_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_6_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/Controller/Joint_7_PID',1,'openinput'),...
   linio('KUKA_ARM_ASSEMBLY/KUKA LBR arm model',1,'output')];
LinOpt = linearizeOptions('SampleTime',0);  % seek continuous-time model
G = linearize('KUKA_ARM_ASSEMBLY',LinIOs,SnapshotTimes,LinOpt);

size(G)

%%
% Plot the gap between the linearized models at t=0,1,2,3,4 seconds and the final
% model at t=5 seconds.

G5 = G(:,:,end);  % t=5
G5.SamplingGrid = [];
sigma(G5,G(:,:,2:5)-G5,{1e-3,1e3}), grid
title('Variation of linearized dynamics along trajectory')
legend('Linearization at t=5 s','Absolute variation',...
       'location','SouthWest')

%%
% While the dynamics vary significantly at low and high frequency,
% the variation drops to less than 10% near 10 rad/s, which is roughly 
% the desired control bandwidth. Small plant variations near the 
% target gain crossover frequency suggest that we can control the arm 
% with a single set of PI gains and need not resort to gain scheduling.

%% Tuning the PI Controllers with LOOPTUNE
% With |looptune|, you can directly tune all six PI loops to achieve
% the desired response time with minimal loop interaction and
% adequate MIMO stability margins. The controller is tuned in continuous time
% and automatically discretized when writing the PI gains
% back to Simulink. Use the |slTuner| interface to specify
% which blocks must be tuned and to locate the plant/controller boundary.

% Linearize the plant at t=3s
tLinearize = 3;

% Create slTuner interface
TunedBlocks = {'Joint_1_PID','Joint_2_PID','Joint_3_PID',...
               'Joint_4_PID','Joint_5_PID','Joint_6_PID','Joint_7_PID'}; 
ST0 = slTuner('KUKA_ARM_ASSEMBLY',TunedBlocks,tLinearize);

% Mark outputs of PID blocks as plant inputs
addPoint(ST0,TunedBlocks)

% Mark joint angles as plant outputs
addPoint(ST0,'KUKA LBR arm model')

% Mark reference signals
RefSignals = {...
   'ref Select/tREF',...
   'ref Select/bREF',...
   'ref Select/fREF',...
   'ref Select/wREF',...
   'ref Select/hREF',...
   'ref Select/gREF',...
   'ref Select/J7ref'};
addPoint(ST0,RefSignals)


%%
% In its simplest use, |looptune| only needs to know the target control
% bandwidth, which should be at least twice the reciprocal of the desired
% response time. Here the desired response time is 1 second so try a target
% bandwidth of 3 rad/s (bearing in mind that the plant dynamics vary least
% near 10 rad/s).
% 'sm:sli:setup:compile:SteadyStateStartNotSupported';
wc = 3;  % target gain crossover frequency
Controls = TunedBlocks;      % actuator commands
Measurements = 'KUKA LBR arm model';  % joint angle measurements
ST1 = looptune(ST0,Controls,Measurements,wc);

%%
% A final value near or below 1 indicates that |looptune| achieved the requested 
% bandwidth. Compare the responses to a step command in angular position 
% for the initial and tuned controllers.

T0 = getIOTransfer(ST0,RefSignals,Measurements);
T1 = getIOTransfer(ST1,RefSignals,Measurements);

opt = timeoptions; opt.IOGrouping = 'all'; opt.Grid = 'on';
stepplot(T0,'b--',T1,'r',4,opt)
legend('Initial','Tuned','location','SouthEast')

%%
% The six curves settling near y=1 represent the step responses of each joint,
% and the curves settling near y=0 represent the cross-coupling terms. The tuned
% controller is a clear improvement, but there is some overshoot and the Bicep 
% response takes a long time to settle.

%% Exploiting the Second Degree of Freedom
%

TR = TuningGoal.StepTracking(RefSignals,Measurements,0.5);
ST2 = looptune(ST0,Controls,Measurements,TR);

%%

T2 = getIOTransfer(ST2,RefSignals,Measurements);
stepplot(T1,'r--',T2,'g',4,opt)
legend('1-DOF tuning','2-DOF tuning','location','SouthEast')


%% Validating the Tuned Controller
% The tuned linear responses look satisfactory so write the tuned values
% of the PI gains back to the Simulink blocks and simulate
% the overall maneuver. The simulation results appear in Figure 6.

writeBlockValue(ST2)


% 
% H2 = T2([2 4],[2 4]) * diag([-10 90]);  % scale by step amplitude
% H2.u = {'Bicep','Wrist'};
% H2.y = {'Bicep','Wrist'};
% step(H2,5), grid

%% Refining The Design
 

JointDisp = [60 10 60 90 90 60];  % commanded angular displacements, in degrees
TR.InputScaling = JointDisp;

%%
% To reduce saturation of the actuators, limit the gain from reference
% signals to control signals.

UR = TuningGoal.Gain(RefSignals,Controls,6); 

%%
% Retune the controller with these refined tuning goals.

ST3 = looptune(ST0,Controls,Measurements,TR,UR);

%%
% % Compare the scaled responses with the previous design. Notice the significant
% % reduction of the coupling between Wrist and Bicep motion, both in 
% % peak value and total energy.
% 
% T2s = diag(1./JointDisp) * T2 * diag(JointDisp);
% T3s = diag(1./JointDisp) * getIOTransfer(ST3,RefSignals,Measurements) * diag(JointDisp);
% stepplot(T2s,'g--',T3s,'m',4,opt)
% legend('Initial 2-DOF','Refined 2-DOF','location','SouthEast')

%%
% Push the retuned values to Simulink for further validation.

writeBlockValue(ST3)

