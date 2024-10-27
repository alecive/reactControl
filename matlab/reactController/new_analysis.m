clear
joint_info(1).name = '1st torso - pitch'; joint_info(2).name = '2nd torso - roll'; joint_info(3).name = '3rd torso - yaw'; 
joint_info(4).name = '1st shoulder'; joint_info(5).name = '2nd shoulder'; joint_info(6).name = '3rd shoulder'; 
joint_info(7).name = '1st elbow'; joint_info(8).name = '2nd elbow';  joint_info(9).name = '1st wrist'; joint_info(10).name = '2nd wrist'; 
dT = 0.02;
vel_lim = 20;
DEG2RAD = (pi/180);
path_prefix = 'logs/';
folders={'MPC0_vmin', 'MPC0_omin', 'MPC0_omin_vmin','MPC0_omin_vmin_oc', 'MPC0_omin_vmin_oc2', 'MPC0_omin_sc', 'MPC0_omin_sc2', 'MPC0_omin_sc3', 'MPC0_omin_vmin_sc_oc'};%, 
folders = { 'MPC0_omin', 'MPC0_omin_sc', 'MPC1_omin_pc', 'MPC1_omin_pc_sc', 'MPC1_omin_pmin', 'MPC1_omin_pmin_sc'};
folders= {'omin','omin_sc0005','vmin_oc001', 'omin_vmin_oc004', 'omin_vmin_sc0005'};
names = {'ori min', 'ori min + smooth con', 'dvel min + ori con', 'dvel min + ori min + ori con' 'dvel min + ori min + smooth con'};
folders = {'obstacles_comparison/vmin_oc001','pokus', 'pokus2', 'pokus3'};
num = length(folders);
rmse = zeros(num,3);
distances = zeros(num,10);
cumsum_cart_jerk = cell(num,1);
cumsum_joint_jerk = cell(num,1);
cumsum_joint_jerk2 = cell(num,1);
time_data = cell(num,1);
thetas = cell(num,1);
posErrors = cell(num,1);
times = zeros(num,1);
solver_times = zeros(num,1);
for kk = 1:num
    [t, data, proc] = processData([path_prefix folders{kk} '/'], dT);

    %% plot time diagnostics and End-eff pose error
    f1 = figure(1+kk*100); clf; set(f1,'Name','Time diagnostics and End-eff pose error');
    subplot(4,1,1); title('Taken time comparison'); hold on;
    plot(t(2:end),data.time_diff,'k.', 'MarkerSize',10);
    plot(t,data.solver_time, 'g.', 'MarkerSize',10);
    plot([t(1) t(end)],[dT dT],'r-','LineWidth',2);   
    legend('sampling time','solver time taken');
    ylabel('Time (s)'); xlabel('Time (s)');

    subplot(4,1,2); hold on; title('ipopt exit code');
    plot(t,data.ipoptExit,'r.', 'MarkerSize',10);
    xlabel('time (s)'); ylim([-5 7]);
    set(gca,'YTick',[-4 -3 -2 -1 0 1 2 3 4 5 6]);
    set(gca,'YTickLabel',{'MaximumCpuTimeExceeded','ErrorInStepComputation','RestorationFailed','MaximumIterationsExceeded','SolveSucceeded','SolvedToAcceptableLevel','InfeasibleProblemDetected','SearchDirectionBecomesTooSmall','DivergingIterates','UserRequestedStop','FeasiblePointFound'});
    set(gca,'YTickLabel',{'MaxTime','ErrorST','RestFailed','MaxIters','Succeeded','Acceptable','Infeas','SDBTS','DivIter','UserStop','FPFound'});

    subplot(4,1,3), hold on
    plot(t,abs(proc.theta), 'ro', 'MarkerSize',4)
    plot(t,proc.posError, 'go', 'MarkerSize',4)
    subplot(4,1,4), plot(t(2:end), sum(diff(proc.joint_vels).^2,2)*DEG2RAD*DEG2RAD, 'go', 'MarkerSize',4)
    thetas{kk} = abs(proc.theta);
    posErrors{kk} = proc.posError;
    
    %% joint velocities vs. joint vel limits
    f6 = figure(6+kk*100); clf(f6); % set(f6,'Color','white','Name',names{kk});  
    set(f6,'Position',[1 1 800 800])
    index = 1;
    for j=1:10
        if j == 2, continue, end
        subplot(3,3,index); hold on; title(joint_info(j).name);
        plot(t,proc.joint_vels(:,j),'k-');
%        plot(t(2:end),proc.joint_vels2(:,j),'g:');
        index = index + 1;
        ylim([-vel_lim-1 vel_lim+1]);
        xlabel('t [s]'); ylabel('joint velocity [deg/s]');
    end  
    %% compute stats
    rmse(kk,:) = [sum(sqrt(sum((proc.posError).^2,2)).^2), ...
                sum(sqrt(sum((proc.theta).^2,2)).^2), ...
                sum(sqrt(sum((proc.pose_diff).^2,2)).^2)] /length(t);
    cumsum_cart_jerk{kk} = cumsum(proc.cart_jerk_norm);
    cumsum_joint_jerk{kk} = cumsum(proc.joint_jerk_norm);
    cumsum_joint_jerk2{kk} = cumsum(proc.joint_jerk_norm2);
    times(kk) = mean(data.time_diff);
    solver_times(kk) = mean(data.solver_time);
    time_data{kk} = t;
    distances(kk,:) = proc.distance;
end
%% jerk and velocity cart/joint
f100 = figure(100); clf(f100);

subplot(2,1,1)
hold on
title('Orientation error')
for i = 1:num
    plot(time_data{i},thetas{i}*180/pi, 'lineWidth', 2)
end
ylabel('error [deg]')
subplot(2,1,2)
hold on
title('Position error')
for i = 1:num
    plot(time_data{i},posErrors{i}, 'lineWidth', 2)
end
ylabel('error [m]')
legend(names)
%% jerk and velocity cart/joint
f1000 = figure(1000); clf(f1000);

subplot(4,1,1)
hold on
title('Cumulative sum of cart jerks')
for i = 1:num
    plot(time_data{i}(4:end),cumsum_cart_jerk{i}, 'lineWidth', 2)
end
subplot(4,1,2)
hold on
title('Cumulative sum of joint jerks v1')
for i = 1:num
    plot(time_data{i}(4:end),cumsum_joint_jerk{i}, 'lineWidth', 1.5)
end
subplot(4,1,3)
hold on
title('Cumulative sum of joint jerks v2')
for i = 1:num
    plot(time_data{i}(4:end),cumsum_joint_jerk2{i}, 'lineWidth', 1.5)
end

subplot(4,1,4)
hold on
title('Movement distance')
plot(distances', 'lineWidth', 1.5)
legend(folders)