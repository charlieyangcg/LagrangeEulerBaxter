%% Form LE dynamics of Baxter arm (left)
% Uses symbolic method with Baxter kinematic and dynamic information to
% generate symbolic matrices representing closed form dynamics of the
% Baxter manuipulator.

% Requires Peter Corke's robotics toolbox, at least version 9.10, and the
% function roundSymbolic() to be in the local folder or path. This is very
% memory intensive due to the large number of symbolic coefficents 
% generated in the matrices. To aid in this, variables are saved to a .mat
% file after generation, and then cleared from RAM.  

%% Preamble
clearvars 
close all 
clc

%% Load model and get primary variables:

Baxter;
% Number of joints:
n = Baxter_l.n;

q = sym('q',[1 n]); q = sym(q, 'real');
% velocity
qd = sym('qd',[1 n]); qd = sym(qd, 'real');

% Account for any offsets
for i = 1:n
    q(i) = q(i) + Baxter_l.links(i).offset;
end

% All joints are revolute
a = zeros(1,n);
d = zeros(1,n);
alpha = zeros(1,n);
for i = 1:n
    a(i) = Baxter_l.links(i).a;
    d(i) = Baxter_l.links(i).d;
    alpha(i) = Baxter_l.links(i).alpha;
end

% Inertia
I = zeros(6,n);
for i = 1:n
  I(:,i) = [diag(Baxter_l.links(i).I); Baxter_l.links(i).I(1,2); Baxter_l.links(i).I(2,3); Baxter_l.links(i).I(1,3)];   
end

% Mass
M = zeros(1,n);
for i = 1:n
    M(i) = Baxter_l.links(i).m;
end

% Centre of Mass
CoM = ones(4,n);
for i = 1:n
   CoM(1:3,i) = Baxter_l.links(i).r; 
end

%% Inertia tensor (J)
J = zeros(4,4,n);
for i = 1:n
    J(:,:,i) = [0.5*(-I(1,i)+I(2,i)+I(3,i))                      I(4,i)                      I(6,i)  M(i)*CoM(1,i)
                                     I(4,i)  0.5*(I(1,i)-I(2,i)+I(3,i))                      I(5,i)  M(i)*CoM(2,i)
                                     I(6,i)                      I(5,i)  0.5*(I(1,i)+I(2,i)-I(3,i))  M(i)*CoM(3,i)
                              M(i)*CoM(1,i)               M(i)*CoM(2,i)               M(i)*CoM(3,i)           M(i)];
end

%% Constants
Q = zeros(4,4);
Q(1,2) = -1; Q(2,1) = 1;

%% Link transforms
T01 = [cos(q(1)),  -cos(alpha(1))*sin(q(1)),  sin(alpha(1))*sin(q(1)), a(1)*cos(q(1));
       sin(q(1)),   cos(alpha(1))*cos(q(1)), -sin(alpha(1))*cos(q(1)), a(1)*sin(q(1));
       0,           sin(alpha(1)),            cos(alpha(1)),           d(1);
       0,           0,                        0,                       1];   

T12 = [cos(q(2)),  -cos(alpha(2))*sin(q(2)),  sin(alpha(2))*sin(q(2)), a(2)*cos(q(2));
       sin(q(2)),   cos(alpha(2))*cos(q(2)), -sin(alpha(2))*cos(q(2)), a(2)*sin(q(2));
       0,           sin(alpha(2)),            cos(alpha(2)),           d(2);
       0,           0,                        0,                       1];

T23 = [cos(q(3)),  -cos(alpha(3))*sin(q(3)),  sin(alpha(3))*sin(q(3)), a(3)*cos(q(3));
       sin(q(3)),   cos(alpha(3))*cos(q(3)), -sin(alpha(3))*cos(q(3)), a(3)*sin(q(3));
       0,           sin(alpha(3)),            cos(alpha(3)),           d(3);
       0,           0,                        0,                       1];

T34 = [cos(q(4)),  -cos(alpha(4))*sin(q(4)),  sin(alpha(4))*sin(q(4)), a(4)*cos(q(4));
       sin(q(4)),   cos(alpha(4))*cos(q(4)), -sin(alpha(4))*cos(q(4)), a(4)*sin(q(4));
       0,           sin(alpha(4)),            cos(alpha(4)),           d(4);
       0,           0,                        0,                       1];

T45 = [cos(q(5)),  -cos(alpha(5))*sin(q(5)),  sin(alpha(5))*sin(q(5)), a(5)*cos(q(5));
       sin(q(5)),   cos(alpha(5))*cos(q(5)), -sin(alpha(5))*cos(q(5)), a(5)*sin(q(5));
       0,           sin(alpha(5)),            cos(alpha(5)),           d(5);
       0,           0,                        0,                       1];

T56 = [cos(q(6)),  -cos(alpha(6))*sin(q(6)),  sin(alpha(6))*sin(q(6)), a(6)*cos(q(6));
       sin(q(6)),   cos(alpha(6))*cos(q(6)), -sin(alpha(6))*cos(q(6)), a(6)*sin(q(6));
       0,           sin(alpha(6)),            cos(alpha(6)),           d(6);
       0,           0,                        0,                       1];

T67 = [cos(q(7)),  -cos(alpha(7))*sin(q(7)),  sin(alpha(7))*sin(q(7)), a(7)*cos(q(7));
       sin(q(7)),   cos(alpha(7))*cos(q(7)), -sin(alpha(7))*cos(q(7)), a(7)*sin(q(7));
       0,           sin(alpha(7)),            cos(alpha(7)),           d(7);
       0,           0,                        0,                       1];

T00 = eye(4);
T11 = eye(4);
T22 = eye(4);
T33 = eye(4);
T44 = eye(4);
T55 = eye(4);
T66 = eye(4);
T77 = eye(4);
disp('Starting transform simplification... ')
tic
T02 = simplify(T01*T12);
T03 = simplify(T02*T23);
T04 = simplify(T03*T34);
T05 = simplify(T04*T45);
T06 = simplify(T05*T56);
T07 = simplify(T06*T67);

T13 = simplify(T12*T23);
T14 = simplify(T13*T34);
T15 = simplify(T14*T45);
T16 = simplify(T15*T56);
T17 = simplify(T16*T67);

T24 = simplify(T23*T34);
T25 = simplify(T24*T45);
T26 = simplify(T25*T56);
T27 = simplify(T26*T67);

T35 = simplify(T34*T45);
T36 = simplify(T35*T56);
T37 = simplify(T36*T67);

T46 = simplify(T45*T56);
T47 = simplify(T46*T67);

T57 = simplify(T56*T67);
toc


%% U matrix
disp('Starting U matrix calculation... ')
tic
for i = 1:n
    for j = 1:n
        if j <= i
            U(:,:,i,j) = eval(strcat('T',num2str(0),num2str(j-1))) * Q * eval(strcat('T',num2str(j-1),num2str(i)));
        else
            U(:,:,i,j) = zeros(4); 
        end
    end
end
toc
disp('Starting U matrix simplification... ')
tic
U = simplify(U);
toc

%% 3D U matrices 
disp('Starting U3 matrix calculation... ')
tic
for i = 1:n
    for j = 1:n
        for k = 1:n
            if (i >= k) && (k >= j)
                U3(:,:,i,j,k) = eval(strcat('T',num2str(0),num2str(j-1))) * Q * eval(strcat('T',num2str(j-1),num2str(k-1))) * Q * eval(strcat('T',num2str(k-1),num2str(i)));
            elseif (i >= j) && (j >= k)
                U3(:,:,i,j,k) = eval(strcat('T',num2str(0),num2str(k-1))) * Q * eval(strcat('T',num2str(k-1),num2str(j-1))) * Q * eval(strcat('T',num2str(j-1),num2str(i)));
            else
                U3(:,:,i,j,k) = zeros(4);
            end
            
        end
    end
end
toc
disp('Starting U3 matrix simplification... ')
tic
U3 = simplify(U3);
toc
save('baseSymbols.mat', 'U', 'U3', 'J', 'n', 'qd', 'CoM', 'M')
clearvars

%% D matrix (inertia)

load('baseSymbols.mat', 'n', 'U', 'J')

disp('Starting D matrix calculation... ')
D = sym('D',[n n]);
tic
for i = 1:n
    for k = 1:n
        sum = 0;
        for j = max(i,k):n
            sum = sum + trace(U(:,:,j,k)*J(:,:,j)*U(:,:,j,i)');
        end
        D(i,k) = sum;%simplify(sum);
        fprintf('Rounding row %d, column %d', i,k)
        tic
        D(i,k) = roundSymbolic(D(i,k),6);
        toc
    end
end
fprintf('Total time: ')
toc
clearvars -except D 
save inertiaD.mat
clearvars
disp('Starting D matrix simplification... ')
% tic
% D = simplify(D);
% toc

%% h vector (coriolis)
disp('Starting H vector calculation... ')

load('baseSymbols.mat', 'n', 'U', 'U3', 'J', 'qd')

tic
% h3 = sym('h3',[n n n]);
for i = 1:n
    for k = 1:n
        for m_idx = 1:n
            sum = 0;
            for j = max([i, k, m_idx]):n
                sum = sum + trace(U3(:,:,j,k,m_idx) * J(:,:,j) * U(:,:,j,i)');
            end
            fprintf('h3: i= %d, k = %d, m = %d \n', i,k,m_idx)
            h3(i,k,m_idx) = sum;
        end
    end
end
clearvars -except h3 n qd

h = sym('h',[n 1]);
for i = 1:n
    sum_k = 0;
    for k = 1:n
        sum_m = 0;
        for m_idx = 1:n
            fprintf('h: i= %d, k = %d, m = %d \n', i,k,m_idx)
            sum_m = sum_m + h3(i,k,m_idx) * qd(k) * qd(m_idx);
        end
        sum_k = sum_k + sum_m;
    end
    fprintf('Rounding row %d, column %d', i,k)
    tic
    h(i,1) = roundSymbolic(sum_k,6);
    toc
end
fprintf('Total time: ')
toc
clearvars -except h
save coriolisH.mat
clearvars
% disp('Starting H vector simplification... ')
% tic
% % h = simplify(h);
% toc

%% c vector (gravity)
load('baseSymbols.mat', 'n', 'U', 'qd')
g = max(Baxter_l.gravity);
g_vec = [0 0 -g 0];
disp('Starting c vector calculation... ')
tic
c = sym('c',[n 1]);
for i = 1:n
    sum = 0;
    for j = i:n
        sum = sum + (-M(j) * g_vec * U(:,:,j,i) * CoM(:,1));
    end
    fprintf('Rounding row %d, column %d', i,k)
    tic
    c(i,1) = roundSymbolic(sum,6);
    toc
end
fprintf('Total time: ')
toc
disp('Starting c vector simplification... ')
tic
c = simplify(c);
toc
clearvars -except c 
save gravityC.mat
clearvars


