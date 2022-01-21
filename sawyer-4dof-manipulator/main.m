% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Jun, 16, 2020
% Last Updated: Jan, 20 2022
%
% -------------------------------------------------
% adaptive fuzzy sliding mode controller
% sawyer 4 dof manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
%%
clc; clear all;
addpath(genpath('.'));

% 1:PID 2:FC_SIGN 3:FC_SAT 4: FSMC 5: AFSMC
control_mode = 5;
n=4;

% load Fuzzy model
ffis = readfis("fis\FSMC");
affis = readfis("fis\AFSMC");

%%% Gain Parameters
Kp = diag([500 1000 2000 2000]);
Ki = diag([5 5 5 5]*100);
Kd = diag([5 5 5 5]*10);
FC_SIGN.L = [35 35 10 15];
FC_SAT.L = [35 35 10 15];
FSMC.L = [500 500 100 50];

%%% simulation time
sim_time = 10;
sim_period = 0.001;
t = 0:sim_period:sim_time;
sample_size = size(t, 2);

%%% External torque
tau_ex(1,:) = 10*sin(0.1*pi*t);
tau_ex(2,:) = 10*cos(0.1*pi*i*sim_period);
tau_ex(3,:) = zeros(sample_size,1);
tau_ex(4,:) = zeros(sample_size,1);

%%% Trajectory
Ran_v1 = 10;
Ran_v2 = 5;

iq_t(1,:)=-20/pi*cos(0.5*pi*t)-10/pi*cos(1*pi*t);
iq_t(2,:)=-20/pi*cos(0.5*pi*t)-10/pi*cos(1*pi*t);
iq_t(3,:)=-Ran_v2*10/pi*cos(0.1*pi*t);
iq_t(4,:)=-Ran_v2*10/pi*cos(0.1*pi*t);

q_t(1,:)=10*sin(0.5*pi*t)+10*sin(1*pi*t);
q_t(2,:)=10*sin(0.5*pi*t)+10*sin(1*pi*t);
q_t(3,:)=Ran_v2*sin(0.1*pi*t);
q_t(4,:)=Ran_v2*sin(0.1*pi*t);

qd_t(1,:)=5*pi*cos(0.5*pi*t)+10*pi*cos(1*pi*t);
qd_t(2,:)=5*pi*cos(0.5*pi*t)+10*pi*cos(1*pi*t);
qd_t(3,:)=Ran_v2*0.1*pi*cos(0.1*pi*t);
qd_t(4,:)=Ran_v2*0.1*pi*cos(0.1*pi*t);

qdd_t(1,:)=-2.5*pi^2*sin(0.5*pi*t)-10*pi^2*sin(1*pi*t);
qdd_t(2,:)=-2.5*pi^2*sin(0.5*pi*t)-10*pi^2*sin(1*pi*t);
qdd_t(3,:)=-Ran_v2*(0.1*pi)^2*sin(0.1*pi*t);
qdd_t(4,:)=-Ran_v2*(0.1*pi)^2*sin(0.1*pi*t);

for i=1:n
    q_t(i,:) = q_t(i,:) /180*pi;
    qd_t(i,:) = qd_t(i,:) /180*pi;
    qdd_t(i,:) = qdd_t(i,:) /180*pi;
    iq_t(i,:) = iq_t(i,:) /180*pi;
end

% sliding various 
smax = [50 50 50 25];
smin = [-50 -50 -50 -75];
sdotmax = [2500 2500 2500 2500];
sdotmin = -[2500 2500 2500 2500];

% Initilization
x(:,1) = [q_t(:,1); zeros(n,1)];
qdd(:,1)= zeros(n,1);
int_q = zeros(n,1);
idx = zeros(sample_size,4);

%%% Simulation
for i=1:sample_size
    q = x(1:n, i);
    dq = x(n+1:n*2, i);
    
    % error
    e(:,i) = q_t(:,i)-q;
    e_dot(:,i) = qd_t(:,i)-dq;
    int_q = int_q+e(:,i)*sim_period;
    
    % model
    M = compute_M_4DOF(q);
    C = compute_C_4DOF(q, dq);
    G = compute_G_4DOF(q);
    
    % Sliding various
    s(:,i) =  Kp*(e(:,i)) + Ki*int_q + Kd*e_dot(:,i);
    sdot(:,i) =  Kp*(e_dot(:,i)) + Ki*e(:,i) +Kd*qdd_t(:,i) + Kd*inv(M)*C*dq + Kd*inv(M)*G; %- Kd*inv(M)*tau_ex(:,i); %  Kd*(qdd_t(:,i) + M\C*dq+ M\G - M\tau_ex(:,i));
   
    % Control Input
    ueq = inv(Kd*inv(M))*(Kp*e_dot(:,i)+Ki*e(:,i)+Kd*qdd_t(:,i) + Kd*(qdd_t(:,i) + inv(M)*C*dq+ inv(M)*G)); %- inv(M)*tau_ex(:,i)));
    
    max_kf = [800 800 150 100];
    min_kf = [200 200 50 35];
    
    for k = 1:n
        %%% AFSMC
        if control_mode == 5
            sdotnor(k, i) = ((sdotmax(k)-sdot(k,i))/(sdotmax(k)-sdotmin(k))*2 - 1);
            snor(k, i) = ((smax(k)-s(k,i))/(smax(k)-smin(k))*2 - 1);
            enor(k,i) = e(k,i)/2;
            edornor(k,i) = e_dot(k,i)/100;

            if(abs(enor(k,i)) > 0.01)
                kf(k,i) = max_kf(k);
            elseif(abs(edornor(k,i)) > 0.01)
                kf(k,i) = min_kf(k);
            else
                kf(k,i) = ((evalfis(affis, [enor(k,i)  edornor(k,i)])-4000)/3500)*(max_kf(k)-min_kf(k))+min_kf(k);
            end
            if(abs(sdotnor(k, i)) > 1 || abs(snor(k, i)) > 1)
                ur(k,i) = kf(k,i)*sign(s(k,i));
            else
                ur(k,i) = kf(k,i)*evalfis(ffis, [snor(k, i) sdotnor(k, i)]);
                idx(i,k) = 1;
            end
        end
        
        %%% FSMC
        if control_mode == 4
            sdotnor(k, i) = ((sdotmax(k)-sdot(k,i))/(sdotmax(k)-sdotmin(k))*2 - 1);
            snor(k, i) = ((smax(k)-s(k,i))/(smax(k)-smin(k))*2 - 1);

            if(abs(sdotnor(k, i)) > 1 || abs(snor(k, i)) > 1)
                ur(k,i) = FSMC.L(k)*sign(s(k,i));
            else
                ur(k,i) = FSMC.L(k)*evalfis(ffis, [snor(k, i) sdotnor(k, i)]);
                idx(i,k) = 1;
            end
        end
        
        %%% 일반 SAT
        if control_mode == 3
            ur(k,i) = FC_SAT.L(k)*sat(s(k,i),1);
        end
        
        %%% 일반 Sign
        if control_mode == 2
            ur(k,i) = FC_SIGN.L(k)*sign(s(k,i));
        end
        
        %%% PID
        if control_mode == 1
            ur(k,i) = Kp(k,k)*(e(k, i)) + Ki(k,k)*int_q(k) + Kd(k,k)*e_dot(k, i);
        end
    end
    
    U(:,i) = ur(:,i) + ueq + tau_ex(:,i);
    
    if(i ~= sample_size)
        x(:,i+1) = rk2(x(:,i), U(:,i), sim_period,M,C,G);
    end
end

% Plot
% figure 1 : Joint Position
fig = figure(1);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:n
    ax = nexttile;
    plot(t, q_t(i,:),'-m','LineWidth',1.5');
    hold on
    plot(t, x(i,:),'-k','LineWidth',1.2')
    grid on;
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("q_{"+i+ "}(rad)", 'FontSize', 10);
end
legend('ref','cur')
saveas(gcf,"fig\joint_position_result_"+num2str(control_mode)+".png");

% figure 2 : Control Input
fig = figure(2);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:n
    ax = nexttile;
    plot(t, U(i,:),'-k','LineWidth',1.5');
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("\tau_{"+i+ "}(Nm)", 'FontSize', 10);
end
legend('u')
saveas(gcf,"fig\joint_input_result_"+num2str(control_mode)+".png");

% figure 3 : Joint Error
fig = figure(3);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:n
    ax = nexttile;
    plot(t, e(i,:),'-k','LineWidth',1.5');
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("e_{"+i+ "}(Nm)", 'FontSize', 10);
end
legend('e')
saveas(gcf,"fig\position_error_result_"+num2str(control_mode)+".png");

% figure 4 : Sliding Surface
fig = figure(4);
tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:n
    ax = nexttile;
    hold off
    plot(t, s(i,:),'-k');
    hold on
    plot(t, sdot(i,:),'-b')
    grid on
    xlim([0 sim_time])
    xlabel('Time (sec)', 'FontSize', 10)
    ylabel("s", 'FontSize', 10);
end
legend('s', 'ds')

% AFSMC and FSMC
if control_mode > 3
    % figure 5 : Fuzzy
    fig = figure(5);
    tiledlayout(2,2,'TileSpacing','Compact','Padding','Compact');
    set(gcf,'color','w');
    for i=1:4
        ax = nexttile;
        idx_fuzzy = double(idx(:,i));
        plot(t,idx_fuzzy','-k')
    end

    % figure 6: 
    if control_mode == 4
        fig = figure(6);
        [x,y,z] = gensurf(ffis);
        set(gcf,'color','w');
        hold off;
        surf(x,y,z,'EdgeColor','none','FaceAlpha',0.5);
        hold on;
        for i=1:n
            plot3(snor(i,:), sdotnor(i,:), ur(i,:)/FSMC.L(i),'.');
        end
        axis([-1.2 1.2 -1.2 1.2 -1.2 1.2])
        xlabel('s')
        ylabel('sdot')
        view([45 90 45])
    end
    
    if control_mode == 5
        fig = figure(7);
        [x,y,z] = gensurf(affis);
        set(gcf,'color','w');
        hold off;
        surf(x,y,(z-4000)/3500,'EdgeColor','none','FaceAlpha',0.5);
        hold on;
        for i=1:n
            plot3(enor(i,:),edornor(i,:),((kf(i,:))-min_kf(i))/(max_kf(i)-min_kf(i)),'.');
        end
%         axis([-1.2 1.2 -1.2 1.2 -1.2 1.2])
        xlabel('e')
        ylabel('de')
        view([45 90 45])
    end
end


% RSSE : Root Sum Squared Error
for i=1:n
    RSSE(i) = sqrt(sum((e(i,:).^2)*sim_period));
end
RSSE(n+1) = sum(RSSE)
% Function Define
function tau = sat(u, u1)
if abs(u) < u1
    tau = u;
else
    tau = u1*sign(u);
end
end