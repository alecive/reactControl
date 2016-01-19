% Author: Ugo Pattacini, Matej Hoffmann

plot_target = true; % useful in Fig. if target is static
visualize_all_joint_pos = true;
visualize_all_joint_vel = true;
visualize_single_joint_in_detail = true;
save_figs = true;

%path_prefix = 'circular_target/'; %'static_target/';
path_prefix = 'staticTarget_staticObst/obstRad_0.04_pos_-0.35_-0.05_0.04/';
%path_prefix = 'staticTarget_staticObst/obstRad_0.04_pos_-0.35_-0.05_0.02/';

if save_figs
  mkdir('output'); 
end

% param file - columns: 1:#DOF, then joint pos min max for every DOF, then
% joint vel limits for every DOF
d_params=importdata([path_prefix 'param.log']);
% data file -  in columns on the output for 10 DOF case: 1:time, 2:4 target, 5:8 obstacle, 9:11 end-eff target, 12:21 joint velocities, 22:31 joint pos, 32:end - control points
d_n=importdata([path_prefix 'none.log']);
d_v=importdata([path_prefix 'visuoSscalingGain4.log']);
%d_v=importdata([path_prefix 'visuo.log']);
d_t=importdata([path_prefix 'tactile.log']);

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


sz_n=size(d_n);
sz_v=size(d_v);
sz_t=size(d_t);
if (~prod(sz_n==sz_v) || ~prod(sz_v==sz_t))
    error('input data with different sizes!');
end
L=sz_n(1);

c_n=[0.000 0.447 0.741];
c_v=[0.929 0.694 0.125];
c_t=[0.850 0.325 0.098];

%% reference vs. end-effector
f1 = figure(1); clf(f1); set(f1,'Color','white','Name','Reference vs. end-effector');  
hold; axis equal; view([-130 30]); grid;
xlim([-1.0 -0.2]); xlabel('x [m]');
ylim([-0.1 0.1]); ylabel('y [m]');
zlim([-0.2 0.2]); zlabel('z [m]');

plot3(d_n(:,52),d_n(:,53),d_n(:,54),'color',c_n,'linewidth',1.3);
plot3(-0.3+d_v(:,52),d_v(:,53),d_v(:,54),'color',c_v,'linewidth',1.3);
plot3(-0.6+d_t(:,52),d_t(:,53),d_t(:,54),'color',c_t,'linewidth',1.3);

legend({'none','vision','tactile'});

if plot_target
   plot3(d_n(1,2),d_n(1,3),d_n(1,4),'go','LineWidth',10); % plots the actual target
   plot3(-0.3+d_v(1,2),d_v(1,3),d_v(1,4),'go','LineWidth',10); % plots the actual target
   plot3(-0.6+d_t(1,2),d_t(1,3),d_t(1,4),'go','LineWidth',10); % plots the actual target
end

n=10;
[x,y,z]=sphere(n);
c(:,:,1)=ones(n); c(:,:,1) = c(:,:,1) * .9; % make the obstacle 'pale red' 
c(:,:,2)=ones(n); c(:,:,2) = c(:,:,2) * .7;
c(:,:,3)=ones(n); c(:,:,3) = c(:,:,3) * .7;
% obstacle
for i=1:10:L
    r=d_n(i,8);
    h=surf(d_n(i,5)+r*x,d_n(i,6)+r*y,d_n(i,7)+r*z,c,'EdgeColor','none');
    alpha(h,0.03);

    r=d_v(i,8);
    h=surf(-0.3+d_v(i,5)+r*x,d_v(i,6)+r*y,d_v(i,7)+r*z,c,'EdgeColor','none');
    alpha(h,0.03);

    r=d_t(i,8);
    h=surf(-0.6+d_t(i,5)+r*x,d_t(i,6)+r*y,d_t(i,7)+r*z,c,'EdgeColor','none');
    alpha(h,0.03);
    
    drawnow;
end

if save_figs
   saveas(f1,'output/ReferenceVsEndeffector.fig');
end
%% distance control-points/obstacle-center
f2 = figure(2); clf(f2); set(f2,'Color','white','Name','Distance control-points obstacle-center');

dist_n=zeros(L,3);
dist_v=zeros(L,3);
dist_t=zeros(L,3);
for i=1:L
    dist_n(i,:)=[norm(d_n(i,5:7)-d_n(i,52:54)) ...
                 norm(d_n(i,5:7)-d_n(i,55:57)) ...
                 norm(d_n(i,5:7)-d_n(i,58:60))];

    dist_v(i,:)=[norm(d_v(i,5:7)-d_v(i,52:54)) ...
                 norm(d_v(i,5:7)-d_v(i,55:57)) ...
                 norm(d_v(i,5:7)-d_v(i,58:60))];

    dist_t(i,:)=[norm(d_t(i,5:7)-d_t(i,52:54)) ...
                 norm(d_t(i,5:7)-d_t(i,55:57)) ...
                 norm(d_t(i,5:7)-d_t(i,58:60))];            
end

t=d_n(:,1);
r=d_n(1,8);

subplot(131); hold; grid;
plot(t,[dist_n(:,1) dist_v(:,1) dist_t(:,1)]);
plot([t(1) t(end)],[r r],'r--');
xlim([0 7]); xlabel('t [s]');
ylim([0 0.2]); ylabel('distance [m]');
title('control point #1');

subplot(132); hold; grid;
plot(t,[dist_n(:,2) dist_v(:,2) dist_t(:,2)]);
plot([t(1) t(end)],[r r],'r--');
xlim([0 7]); xlabel('t [s]');
ylim([0 0.2]);
title('control point #2');

subplot(133); hold; grid;
plot(t,[dist_n(:,3) dist_v(:,3) dist_t(:,3)]);
xlim([0 7]); xlabel('t [s]');
ylim([0 0.2]);
title('control point #3');
legend({'none','vision','tactile'});
plot([t(1) t(end)],[r r],'r--');

if save_figs
   saveas(f2,'output/DistanceControlPointsObstacleCenter.fig');
end

%% joint values vs. joint limits
if visualize_all_joint_pos
    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
        data = [];
        f3 = figure(3); clf(f3); set(f3,'Color','white','Name','Joint positions - No avoidance');  
        f4 = figure(4); clf(f4); set(f4,'Color','white','Name','Joint positions - Visual avoidance');  
        f5 = figure(5); clf(f5); set(f5,'Color','white','Name','Joint positions - Tactile avoidance');  

        for i=1:3 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f3); 
                    data = d_n;
                case 2 
                    set(0, 'currentfigure', f4); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f5); 
                    data = d_t;
            end
            t = data(:,1);

            for j=1:10
                subplot(4,3,j); hold on;
                plot(t,data(:,joint_info(j).pos_column));
                plot([t(1) t(end)],[joint_info(j).pos_limit_min joint_info(j).pos_limit_min],'r--'); % min joint pos limit
                plot([t(1) t(end)],[joint_info(j).pos_limit_max joint_info(j).pos_limit_max],'r--'); % max joint pos limit   
                xlabel('t [s]');
                ylabel('angle [deg]');
                title(joint_info(j).name);
                hold off;
            end    
                
        end   
    end
    if save_figs
        saveas(f3,'output/JointPositionsNoAvoidance.fig');
        saveas(f4,'output/JointPositionsVisualAvoidance.fig');
        saveas(f5,'output/JointPositionsTactileAvoidance.fig');
    end    
end




%% joint velocities vs. joint vel limits
if visualize_all_joint_vel
    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
        data = [];
        f6 = figure(6); clf(f6); set(f6,'Color','white','Name','Joint velocities - No avoidance');  
        f7 = figure(7); clf(f7); set(f7,'Color','white','Name','Joint velocities - Visual avoidance');  
        f8 = figure(8); clf(f8); set(f8,'Color','white','Name','Joint velocities - Tactile avoidance');  

        for i=1:3 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f6); 
                    data = d_n;
                case 2 
                    set(0, 'currentfigure', f7); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f8); 
                    data = d_t;
            end
            t = data(:,1);

            
            for j=1:10
                subplot(4,3,j); hold on;
                plot(t,data(:,joint_info(j).vel_limit_min_avoid_column),'--c','Marker','^','MarkerSize',2); % current min joint vel limit set by avoidance handler
                plot(t,data(:,joint_info(j).vel_limit_max_avoid_column),'--m','Marker','v','MarkerSize',2); % current max joint vel limit set by avoidance handler
                plot([t(1) t(end)],[joint_info(j).vel_limit_min joint_info(j).vel_limit_min],'-.r'); % min joint vel limit
                plot([t(1) t(end)],[joint_info(j).vel_limit_max joint_info(j).vel_limit_max],'-.r'); % max joint vel limit   
                plot(t,data(:,joint_info(j).vel_column),'-k'); % current joint velocity
                ylim([-52 52]);
                xlabel('t [s]');
                ylabel('joint velocity [deg/s]');
                title(joint_info(j).name);
                hold off;
            end    
            
           
        end

    end
    if save_figs
        saveas(f6,'output/JointVelocitiesNoAvoidance.fig');
        saveas(f7,'output/JointVelocitiesVisualAvoidance.fig');
        saveas(f8,'output/JointVelocitiesTactileAvoidance.fig');
    end
end




%% individual joint in detail 
if visualize_single_joint_in_detail

    j = 9; % joint index to visualize

    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
        data = [];
        f10 = figure(10); clf(f10); set(f10,'Color','white','Name',['No avoidance - ' joint_info(j).name]);  
        f11 = figure(11); clf(f11); set(f11,'Color','white','Name',['Visual avoidance - ' joint_info(j).name]);  
        f12 = figure(12); clf(f12); set(f12,'Color','white','Name',['Tactile avoidance - ' joint_info(j).name]);  

        for i=1:3 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f10); 
                    data = d_n;
                case 2 
                    set(0, 'currentfigure', f11); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f12); 
                    data = d_t;
            end
         t= data(:,1);
         [m,n] = size(data);

         hold on;
         [AX,H1,H2] = plotyy(t,data(:,joint_info(j).pos_column),t,data(:,joint_info(j).vel_column));   
         hold(AX(1),'on');hold(AX(2),'on');
         H7 = plot(AX(2),t,data(:,joint_info(j).vel_limit_min_avoid_column), 'Parent', AX(2), 'LineStyle',':','Color','c','Marker','v','MarkerSize',2);
         H8 = plot(AX(2),t,data(:,joint_info(j).vel_limit_max_avoid_column), 'Parent', AX(2), 'LineStyle',':','Color','m','Marker','^','MarkerSize',2);
         hold(AX(1),'off');hold(AX(2),'off');
         H3 = line([t t],[joint_info(j).pos_limit_min joint_info(j).pos_limit_min], 'Parent', AX(1), 'LineStyle','--','Color','k');
         H4 = line([t t],[joint_info(j).pos_limit_max joint_info(j).pos_limit_max], 'Parent', AX(1), 'LineStyle','--','Color','k');
         H5 = line([t t],[joint_info(j).vel_limit_min joint_info(j).vel_limit_min], 'Parent', AX(2), 'LineStyle',':','Color','r');
         H6 = line([t t],[joint_info(j).vel_limit_max joint_info(j).vel_limit_max], 'Parent', AX(2), 'LineStyle',':','Color','r');
         set(get(AX(1),'Ylabel'),'String','Joint angle [deg]'); 
         set(AX(1),'Ylim',[joint_info(j).pos_limit_min-5 joint_info(j).pos_limit_max+5]); 
         set(get(AX(2),'Ylabel'),'String','Joint velocity [deg/s]');
         set(AX(2),'Ylim',[joint_info(j).vel_limit_min-3 joint_info(j).vel_limit_max+3]); 
         xlabel('t [s]');
         %set(H1,'LineStyle','-');
         %set(H2,'LineStyle','--');
         hold off;

        end
    end
    
    if save_figs
        saveas(f10,'output/SelectedJointDetailNoAvoidance.fig');
        saveas(f11,'output/SelectedJointDetailVisualAvoidance.fig');
        saveas(f12,'output/SelectedJointDetailTactileAvoidance.fig');
    end
end


