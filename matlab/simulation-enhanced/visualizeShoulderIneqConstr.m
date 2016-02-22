save_figs=false;

EXTRA_MARGIN_SHOULDER_INEQ_DEG = (0.05 / (2 * pi) ) * 360; 
%each of the ineq. constraints for shoulder joints will have an extra safety marging of 0.05 rad on each side - i.e. the actual allowed range will be smaller

%path_prefix = 'staticTarget_staticObst/obstRad_0.04_pos_-0.35_-0.05_0.02/';
path_prefix = 'input/';

if save_figs
  mkdir('output'); 
end

% param file - columns: 1:#DOF, then joint pos min max for every DOF, then
% joint vel limits for every DOF
d_params=importdata([path_prefix 'param.log']);
% data file -  in columns on the output for 10 DOF case: 1:time, 2:4 target, 5:8 obstacle, 9:11 end-eff target, 12:21 joint velocities, 22:31 joint pos, 32:end - control points
data=importdata([path_prefix 'data.log']);

if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
    joint_info(1).name = '1st torso - pitch'; joint_info(2).name = '2nd torso - roll'; joint_info(3).name = '3rd torso - yaw'; 
    joint_info(4).name = '1st shoulder'; joint_info(5).name = '2nd shoulder'; joint_info(6).name = '3rd shoulder'; 
    joint_info(7).name = '1st elbow'; joint_info(8).name = '2nd elbow'; 
    joint_info(9).name = '1st wrist'; joint_info(10).name = '2nd wrist'; 
    
    for i=1:10
        joint_info(i).pos_limit_min = d_params(2*i); joint_info(i).pos_limit_max = d_params(2*i+1);
        joint_info(i).vel_limit_min = d_params(20+2*i); joint_info(i).vel_limit_max = d_params(20+2*i+1);
        joint_info(i).pos_column = 21+i; joint_info(i).vel_column = 11+i;
        joint_info(i).vel_limit_min_avoid_column = 30+2*i; joint_info(i).vel_limit_max_avoid_column = 30+2*i+1;
    end
end

t = data(:,1); % time column

f13 = figure(13); clf(f13); set(f13,'Color','white','Name','Inequality constraints - shoulder assembly');  
    
    subplot(3,1,1);
    hold on;
        plot(t, data(:,joint_info(4).pos_column) - data(:,joint_info(5).pos_column));
        plot([t(1) t(end)], [-347/1.71+EXTRA_MARGIN_SHOULDER_INEQ_DEG -347/1.71+EXTRA_MARGIN_SHOULDER_INEQ_DEG],'--r');
        legend([joint_info(4).name ' - ' joint_info(5).name],'lower limit');
        title('1st inequality constraint');
        ylabel('angle (deg)'); 
    hold off;
    
    subplot(3,1,2);
    hold on;
        plot(t, data(:,joint_info(4).pos_column) - data(:,joint_info(5).pos_column) - data(:,joint_info(6).pos_column));
        plot([t(1) t(end)], [-366.57/1.71+EXTRA_MARGIN_SHOULDER_INEQ_DEG -366.57/1.71+EXTRA_MARGIN_SHOULDER_INEQ_DEG],'--r');
        plot([t(1) t(end)], [112.42/1.71-EXTRA_MARGIN_SHOULDER_INEQ_DEG 112.42/1.71-EXTRA_MARGIN_SHOULDER_INEQ_DEG],'-.r');
        legend([joint_info(4).name ' - ' joint_info(5).name ' - ' joint_info(6).name],'lower limit','upper limit');
        title('2nd inequality constraint');
        ylabel('angle (deg)'); 
    hold off;
    
    subplot(3,1,3);
    hold on;
        plot(t, data(:,joint_info(5).pos_column) + data(:,joint_info(6).pos_column));
        plot([t(1) t(end)], [-66.6+EXTRA_MARGIN_SHOULDER_INEQ_DEG -66.6+EXTRA_MARGIN_SHOULDER_INEQ_DEG],'--r');
        plot([t(1) t(end)], [213.3-EXTRA_MARGIN_SHOULDER_INEQ_DEG 213.3-EXTRA_MARGIN_SHOULDER_INEQ_DEG],'-.r');
        legend([joint_info(4).name ' - ' joint_info(5).name],'lower limit','upper limit');
        title('3rd inequality constraint');
        ylabel('angle (deg)'); 
    hold off;
    
    

