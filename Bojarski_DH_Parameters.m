%% RBE502 Robot Control - Term Project
    %--------------------------
    %--------------------------
        % Course:     RBE502 Robot Control
        % Authors:     Brian Bojarski  (bjbojarski@wpi.edu) 
        %              Kevin Ducharme  (kgducharme@gmail.com) 
        %              Jean Gonzalez   (jgonzalez@wpi.edu)
        %              Amiel Fernandez (fernandez.amiel@gmail.com)
        % Title:      RBE502 Project - KUKA Arm Control Project
        % Date:       Fall 2016
    %--------------------------

function RBE502_Project_KUKA
clc; clear;
%% A-1 Define Parameters
syms q1 q2 q3 q4 q5 q6 q7 real;
pi = sym('pi');

l1 = 0;
l2 = 0;
l3 = 0;
l4 = 0;
l5 = 0;
l6 = 0;
l7 = 0;

d1 = 340;
d2 = 0;
d3 = 400;
d4 = 0;
d5 = 400;
d6 = 0;
d7 = 126;

%% A-2 Forward Kinematics
%DH table created using coordinate frames

%    | Theta  |    d     |      a    |    alpha    |
%---------------------------------------------------
DH = [ q1    ,    d1     ,      11    ,      pi/2      ;
%---------------------------------------------------
       q2    ,    d2     ,      12    ,     -pi/2      ;
%---------------------------------------------------
       q3    ,    d3     ,      13    ,     -pi/2      ;
%---------------------------------------------------
       q4    ,    d4     ,      14    ,      -pi/2      ;
%--------------------------------------------------
       q5    ,    d5     ,      15    ,     -pi/2      ;
%---------------------------------------------------
       q6    ,    d6     ,      16    ,     -pi/2      ;
%---------------------------------------------------
       q7    ,    d7     ,      17    ,      0      ;];
%---------------------------------------------------

%Calculate Forward Kinematics using DH table
link = 1; 
      T0_1 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4));               
link = 2;
      T1_2 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4));                 
link = 3;
      T2_3 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4));    
link = 4;
      T3_4 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4));                           
link = 5;
      T4_5 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4)); 
link = 6;
      T5_6 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4)); 
link = 7;
      T6_7 = dh2matsym_rad(DH(link,1),DH(link,2),...
                               DH(link,3),DH(link,4));                            
                     
T0_2 = T0_1*T1_2;                           
T0_3 = T0_1*T1_2*T2_3;                              
T0_4 = T0_1*T1_2*T2_3*T3_4;                          
T0_5 = T0_1*T1_2*T2_3*T3_4*T4_5;                          
T0_6 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6;                        
T0_7 = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
T0_7 = simplify(T0_7);

% 
% %% A-3 Euler Lagrange Dynamics
% Jacobian_l1 = simplify(jacobian(T0_1(1:3,4),[q1]));
% Jacobian_l2 = simplify(jacobian(T0_2(1:3,4),[q1 q2]));
% Jacobian_l3 = simplify(jacobian(T0_3(1:3,4),[q1 q2 q3]));
% Jacobian_l4 = simplify(jacobian(T0_4(1:3,4),[q1 q2 q3 q4]));
% Jacobian_l5 = simplify(jacobian(T0_5(1:3,4),[q1 q2 q3 q4 q5]));
% Jacobian_l6 = simplify(jacobian(T0_6(1:3,4),[q1 q2 q3 q4 q5 q6]));
% Jacobian_l7 = simplify(jacobian(T0_6(1:3,4),[q1 q2 q3 q4 q5 q6 q7]));
% 
% 
% %find the velocity at each mass
% syms q1_d q2_d q3_d q4_d q5_d q6_d q7_d real;
% syms I1 I2 I3 I4 I5 I6 I7 real;
% syms m1 m2 m3 m4 m5 m6 m7 real;
% 
% vel_m1 = simplify(Jacobian_l1*[q1_d]);
% vel_m2 = simplify(Jacobian_l2*[q1_d;q2_d]);
% vel_m3 = simplify(Jacobian_l3*[q1_d;q2_d;q3_d]);
% vel_m4 = simplify(Jacobian_l4*[q1_d;q2_d;q3_d;q4_d]);
% vel_m5 = simplify(Jacobian_l5*[q1_d;q2_d;q3_d;q4_d;q5_d]);
% vel_m6 = simplify(Jacobian_l6*[q1_d;q2_d;q3_d;q4_d;q5_d;q6_d]);
% vel_m7 = simplify(Jacobian_l7*[q1_d;q2_d;q3_d;q4_d;q5_d;q6_d;q7_d]);
% 
% %Kenetic energy
% K1 = 0.5*m1*(vel_m1.' * vel_m1) + (0.5 * I1 * q1_d^2);
% K2 = 0.5*m2*(vel_m2.' * vel_m2) + (0.5 * I2 * (q1_d+q2_d)^2);
% K3 = 0.5*m3*(vel_m3.' * vel_m3) + (0.5 * I3 * (q1_d+q2_d+q3_d)^2);
% K4 = 0.5*m4*(vel_m4.' * vel_m4) + (0.5 * I4 * (q1_d+q2_d+q3_d+q4_d)^2);
% K5 = 0.5*m5*(vel_m5.' * vel_m5) + (0.5 * I5 * (q1_d+q2_d+q3_d+q4_d+q4_d+q5_d)^2);
% K6 = 0.5*m6*(vel_m6.' * vel_m6) + (0.5 * I6 * (q1_d+q2_d+q3_d+q4_d+q4_d+q5_d+q6_d)^2);
% K7 = 0.5*m7*(vel_m7.' * vel_m7) + (0.5 * I7 * (q1_d+q2_d+q3_d+q4_d+q4_d+q5_d+q6_d+q7_d)^2);
% 
% %Potential energy
% syms g real;
% P1 = simplify(m1*g*subs(T0_1(3,4),[l1],[l1/2]));
% P2 = simplify(m2*g*subs(T0_2(3,4),[l2],[l2/2]));
% P3 = simplify(m3*g*subs(T0_3(3,4),[l3],[l3/2]));
% P4 = simplify(m4*g*subs(T0_4(3,4),[l4],[l4/2]));
% P5 = simplify(m5*g*subs(T0_5(3,4),[l5],[l5/2]));
% P6 = simplify(m6*g*subs(T0_6(3,4),[l6],[l6/2]));
% P7 = simplify(m7*g*subs(T0_7(3,4),[l7],[l7/2]));
% 
% %Lagrangian
% L = simplify((K1+K2+K3+K4+K5+K6+K7)-(P1+P2+P3+P4+P5+P6+P7));
% 
% %Euler Lagrange Dynamics
% %Dynamics and time deritivatives
% 
% syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real;
% syms theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta6(t) theta7(t);
% syms q1_dd q2_dd q3_dd q4_dd q5_dd q6_dd q7_dd real;
% 
% %derivitive wrt Q
% L_q1_d = diff(L,q1_d);
% L_q2_d = diff(L,q2_d);
% L_q3_d = diff(L,q3_d);
% L_q4_d = diff(L,q4_d);
% L_q5_d = diff(L,q5_d);
% L_q6_d = diff(L,q6_d);
% L_q7_d = diff(L,q7_d);
% 
% %sub in time dependant variable
% L_q1_d_t = subs(L_q1_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);
% L_q2_d_t = subs(L_q2_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);             
% L_q3_d_t = subs(L_q3_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);                   
% L_q4_d_t = subs(L_q4_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);                   
% L_q5_d_t = subs(L_q5_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);                   
% L_q6_d_t = subs(L_q6_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);
% L_q7_d_t = subs(L_q7_d,[  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,     q1_d        ,      q2_d       ,       q3_d      ,       q4_d      ,      q5_d      ,      q6_d       ,     q7_d       ],...
%                        [theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t)]);                   
% 
%                    
% %derivitive wrt time
% dL_q1_d_t = diff(L_q1_d_t,t);
% dL_q2_d_t = diff(L_q2_d_t,t);
% dL_q3_d_t = diff(L_q3_d_t,t);
% dL_q4_d_t = diff(L_q4_d_t,t);
% dL_q5_d_t = diff(L_q5_d_t,t);
% dL_q6_d_t = diff(L_q6_d_t,t);
% dL_q7_d_t = diff(L_q7_d_t,t);
% 
% %sub back in Q
% L_q1_d = subs(dL_q1_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q2_d = subs(dL_q2_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q3_d = subs(dL_q3_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q4_d = subs(dL_q4_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q5_d = subs(dL_q5_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q6_d = subs(dL_q6_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% L_q7_d = subs(dL_q7_d_t,[theta1,theta2,theta3,theta4,theta5,theta6,theta7,diff(theta1(t),t),diff(theta2(t),t),diff(theta3(t),t),diff(theta4(t),t),diff(theta5(t),t),diff(theta6(t),t),diff(theta7(t),t),diff(theta1(t),t,t),diff(theta2(t),t,t),diff(theta3(t),t,t),diff(theta4(t),t,t),diff(theta5(t),t,t),diff(theta6(t),t,t),diff(theta7(t),t,t)],...
%                         [  q1  ,  q2  ,  q3  ,  q4  ,  q5  ,  q6  ,  q7  ,       q1_d      ,       q2_d      ,       q3_d      ,      q4_d       ,      q5_d       ,      q6_d       ,       q7_d      ,        q1_dd       ,       q2_dd      ,       q3_dd       ,        q4_dd      ,       q5_dd       ,      q6_dd        ,     q7_dd         ]);
% 
% %Potential energy                 
% Pot1 = diff(L,q1);
% Pot2 = diff(L,q2);
% Pot3 = diff(L,q3);
% Pot4 = diff(L,q4);
% Pot5 = diff(L,q5);
% Pot6 = diff(L,q6);
% Pot7 = diff(L,q7);
% 
% 
% tau1 = L_q1_d - Pot1;
% tau2 = L_q2_d - Pot2;
% tau3 = L_q3_d - Pot3;
% tau4 = L_q4_d - Pot4;
% tau5 = L_q5_d - Pot5;
% tau6 = L_q6_d - Pot6;
% tau7 = L_q7_d - Pot7;
% 
% tau = [tau1;
%        tau2;
%        tau3;
%        tau4;
%        tau5;
%        tau6;
%        tau7;];
% 
% %split into coefficients
% 
% %interita
% M = [tau1-subs(tau1,q1_dd,0) /q1_dd  tau1-subs(tau1,q2_dd,0) /q2_dd  tau1-subs(tau1,q3_dd,0) /q3_dd  tau1-subs(tau1,q4_dd,0) /q4_dd  tau1-subs(tau1,q5_dd,0) /q5_dd  tau1-subs(tau1,q6_dd,0) /q6_dd  tau1-subs(tau1,q7_dd,0) /q7_dd;
%      tau2-subs(tau2,q1_dd,0) /q1_dd  tau2-subs(tau2,q2_dd,0) /q2_dd  tau2-subs(tau2,q3_dd,0) /q3_dd  tau2-subs(tau2,q4_dd,0) /q4_dd  tau2-subs(tau2,q5_dd,0) /q5_dd  tau2-subs(tau2,q6_dd,0) /q6_dd  tau2-subs(tau2,q7_dd,0) /q7_dd;
%      tau3-subs(tau3,q1_dd,0) /q1_dd  tau3-subs(tau3,q2_dd,0) /q2_dd  tau3-subs(tau3,q3_dd,0) /q3_dd  tau3-subs(tau3,q4_dd,0) /q4_dd  tau3-subs(tau3,q5_dd,0) /q5_dd  tau3-subs(tau3,q6_dd,0) /q6_dd  tau3-subs(tau3,q7_dd,0) /q7_dd;
%      tau4-subs(tau4,q1_dd,0) /q1_dd  tau4-subs(tau4,q2_dd,0) /q2_dd  tau4-subs(tau4,q3_dd,0) /q3_dd  tau4-subs(tau4,q4_dd,0) /q4_dd  tau4-subs(tau4,q5_dd,0) /q5_dd  tau4-subs(tau4,q6_dd,0) /q6_dd  tau4-subs(tau4,q7_dd,0) /q7_dd;
%      tau5-subs(tau5,q1_dd,0) /q1_dd  tau5-subs(tau5,q2_dd,0) /q2_dd  tau5-subs(tau5,q3_dd,0) /q3_dd  tau5-subs(tau5,q4_dd,0) /q4_dd  tau5-subs(tau5,q5_dd,0) /q5_dd  tau5-subs(tau5,q6_dd,0) /q6_dd  tau5-subs(tau5,q7_dd,0) /q7_dd;
%      tau6-subs(tau6,q1_dd,0) /q1_dd  tau6-subs(tau6,q2_dd,0) /q2_dd  tau6-subs(tau6,q3_dd,0) /q3_dd  tau6-subs(tau6,q4_dd,0) /q4_dd  tau6-subs(tau6,q5_dd,0) /q5_dd  tau6-subs(tau6,q6_dd,0) /q6_dd  tau6-subs(tau6,q7_dd,0) /q7_dd;
%      tau7-subs(tau7,q1_dd,0) /q1_dd  tau7-subs(tau7,q2_dd,0) /q2_dd  tau7-subs(tau7,q3_dd,0) /q3_dd  tau7-subs(tau7,q4_dd,0) /q4_dd  tau7-subs(tau7,q5_dd,0) /q5_dd  tau7-subs(tau7,q6_dd,0) /q6_dd  tau7-subs(tau7,q7_dd,0) /q7_dd;]           
%                     
% %gravity
% G = [subs(tau1,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau2,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau3,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau4,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau5,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau6,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);
%      subs(tau7,[q1_d,q2_d,q3_d,q4_d,q5_d,q6_d,q7_d,q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd],...
%                [  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ]);]
%            
% %Coriolis
% C = [tau1-(M(1,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(1));
%      tau2-(M(2,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(2));
%      tau3-(M(3,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(3));
%      tau4-(M(3,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(4));
%      tau5-(M(3,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(5));
%      tau6-(M(3,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(6));
%      tau7-(M(3,:)*[q1_dd,q2_dd,q3_dd,q4_dd,q5_dd,q6_dd,q7_dd].'+G(7));]         
% 
% %% A-4 State Space
% 
% M_inv = inv(M);
% 
% q_dd = (M_inv*(tau-C-G));


%% A-5 Functions
function T = dh2matsym_rad(theta, d, a, alpha)

R_z = sym([cos(theta) -sin(theta) 0 0; 
            sin(theta)  cos(theta) 0 0; 
            0 0 1 0; 
            0 0 0 1]); 
        
T_z = sym([1 0 0 0; 
           0 1 0 0; 
           0 0 1 d; 
           0 0 0 1]);
       
T_x = sym([1 0 0 a; 
           0 1 0 0; 
           0 0 1 0; 
           0 0 0 1]);
       
R_x = sym([1 0 0 0; 
            0 cos(alpha) -sin(alpha) 0; 
            0 sin(alpha) cos(alpha) 0; 
            0 0 0 1]);

T = R_z*T_z*T_x*R_x;