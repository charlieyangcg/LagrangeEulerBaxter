%% Generate correct Baxter model with Dynamics
%% Preamble
clear all 
close all
clc

%% Left arm
%% Links 
q0 = zeros(1,7);
h = 0.229525; %distance from last frame to end effector along z-axis

% % define leftarm dh params [q, d, a, alpha, 0/1 revolute/prismatic, angle offset]
Ll(1) = Link('d', 0.27035, 'a', 0.069, 'alpha', -pi/2, 'offset',    0); 
Ll(2) = Link('d', 0      , 'a', 0    , 'alpha',  pi/2, 'offset', pi/2); 
Ll(3) = Link('d', 0.36435, 'a', 0.069, 'alpha', -pi/2, 'offset',    0); 
Ll(4) = Link('d', 0.     , 'a', 0    , 'alpha',  pi/2, 'offset',    0); 
Ll(5) = Link('d', 0.37429, 'a', 0.01 , 'alpha', -pi/2, 'offset',    0); 
Ll(6) = Link('d', 0      , 'a', 0    , 'alpha',  pi/2, 'offset',    0); 
Ll(7) = Link('d', h      , 'a', 0    , 'alpha',     0, 'offset',    0); 


%% Right Arm
% because the sign on joints is the same for right and left arm joint variables,
%  only the base transformation shown below differentiates the left and
%  right arms.
Lr(1) = Link('d', 0.27035, 'a', 0.069, 'alpha', -pi/2, 'offset',    0); 
Lr(2) = Link('d', 0      , 'a', 0    , 'alpha',  pi/2, 'offset', pi/2); 
Lr(3) = Link('d', 0.36435, 'a', 0.069, 'alpha', -pi/2, 'offset',    0); 
Lr(4) = Link('d', 0.     , 'a', 0    , 'alpha',  pi/2, 'offset',    0); 
Lr(5) = Link('d', 0.37429, 'a', 0.01 , 'alpha', -pi/2, 'offset',    0); 
Lr(6) = Link('d', 0      , 'a', 0    , 'alpha',  pi/2, 'offset',    0); 
Lr(7) = Link('d', h      , 'a', 0    , 'alpha',     0, 'offset',    0); 

%% Dynamic properties
%% Mass
m(1) = 5.700440;
m(2) = 3.226980;
m(3) = 4.312720;
m(4) = 2.072060;
m(5) = 2.246650;
m(6) = 1.609790;
m(7) = 0.350930 + 0.191250;

%% CoG
r = zeros(7,3);
r(1,:) = [ -0.0511700000000000,     0.0790800000000000,        0.000859999999999956];
r(2,:) = [  0.00269000000000000,    -0.00529000000000003,      0.0684499999999999];
r(3,:) = [ -0.0717600000000000,     0.0814900000000001,        0.00131999999999994];
r(4,:) = [  0.00159000000000006,    -0.0111700000000000,       0.0261799999999999];
r(5,:) = [ -0.0116799999999999,     0.131110000000000,         0.00459999999999992];
r(6,:) = [  0.00697000000000011,    0.00599999999999981,       0.0604800000000000];
r(7,:) = [  0.00513704655280540,    0.000957223615773138,      -0.0668234671142425];

%% Inertia                      %% NOT CORRECT ORDER! THIS IS: Ixx Ixy Ixz Iyy Iyz Izz
I = zeros(7,6);                                              % Ixx Iyy Izz Ixy Iyz Ixz
I(1,:) = [0.04709102262	 -0.00614870039     0.00012787556	0.0359598847	   -0.00078086899	  0.03766976455];
I(2,:) = [0.0278859752	 -0.00018821993    -0.000300963979	0.0207874929	    0.00207675762     0.01175209419];
I(3,:) = [0.02661733557	 -0.00392189887     0.00029270634	0.0124800832	   -0.0010838933	  0.02844355207];
I(4,:) = [0.01318227876	 -0.00019663418     0.00036036173	0.0092685206	    0.0007459496	  0.00711582686];
I(5,:) = [0.01667742825	 -0.00018657629     0.00018403705	0.0037463115	    0.00064732352	  0.01675457264];
I(6,:) = [0.00700537914	  0.00015348067    -0.00044384784	0.0055275524	   -0.00021115038	  0.00387607152];
I(7,:) = [0.00081621358	  0.00012844010     0.000189698911  0.00087350127      0.00010577265	  0.00054941487];
for i = 1:7 %% Re-arrange to correct order
   I(i,:) = [I(i,1) I(i,4) I(i,6) I(i,2) I(i,5) I(i,3)];
end

%% Stiffness and damping
% K = [540 540 540 540 240 240 240];
% D = [0 0 0 0 0 0 0];

%% Create SerialLinks
Baxter_l = SerialLink(Ll(1:7), 'name', 'Baxter_L', 'base' , transl(0.024645, 0.219645, 0.118588)*trotz(pi/4)*transl(0.055695, 0, 0.011038));
Baxter_r = SerialLink(Lr(1:7), 'name', 'Baxter_R', 'base' , transl(0.024645, -0.219645, 0.118588)*trotz(-pi/4)*transl(0.055695, 0, 0.011038));
% Add the dynamic parameters
n = Baxter_l.n;
for i=1:n
   Baxter_l.links(i).m = m(i);
   Baxter_r.links(i).m = m(i);
   Baxter_l.links(i).r = r(i,:);
   Baxter_r.links(i).r = r(i,:);
   Baxter_l.links(i).I = I(i,:);
   Baxter_r.links(i).I = I(i,:);
   Baxter_l.links(i).Jm = 0;
%    Baxter_r.links(i).Jm = 0;
%    Baxter_l.links(i).K = K(i);
%    Baxter_r.links(i).K = K(i);
%    Baxter_l.links(i).D = D(i);
%    Baxter_r.links(i).D = D(i);
end


%+---+-----------+-----------+-----------+-----------+                               
%| j |     theta |         d |         a |     alpha |                               
%+---+-----------+-----------+-----------+-----------+                               
%|  1|         q1|     0.2703|      0.069|     -1.571|                               
%|  2|         q2|          0|          0|      1.571|                               
%|  3|         q3|     0.3644|      0.069|     -1.571|                               
%|  4|         q4|          0|          0|      1.571|                               
%|  5|         q5|     0.3743|       0.01|     -1.571|                               
%|  6|         q6|          0|          0|      1.571|                               
%|  7|         q7|     0.2295|          0|          0|                               
%+---+-----------+-----------+-----------+-----------+   

%% Plot Arms
% Baxter_l.plot(q0)
% hold on;
% Baxter_r.plot(q0)

%% Attempt angles
%% Calculate Start position
% Lq0 = [-0.47 0.00 -1.57 1.12 0.00 1.15 0];       % Initial joint estimates   
% Rq0 = [-0.47 0.00 1.57 1.68 0.00 1.15 0];
% LX0 = [0.6 0.2 0.4];                   % Desired initial positions
% LR0 = [pi/2 0 3];
% RX0 = [0.6 -0.2 0.4];
% RR0 = [ -pi/2 0 3];
% LT0 = transl(LX0) * rpy2tr(LR0);
% Lq0 = Baxter_l.ikine(LT0, Lq0', 'pinv','alpha',0.1);
% RT0 = transl(RX0) * rpy2tr(RR0);
% Rq0 = Baxter_r.ikine(RT0, Rq0, 'pinv','alpha',0.5);
% if max(isnan(Lq0)) || max(isnan(Rq0))
%     break
% end
% LX0 = transl(Baxter_l.fkine(Lq0));
% LR0 = tr2rpy(Baxter_l.fkine(Lq0));
% RX0 = transl(Baxter_r.fkine(Rq0));
% RR0 = tr2rpy(Baxter_r.fkine(Rq0));
% 
% Baxter_l.plot(Lq0)
% Baxter_r.plot(Rq0)
