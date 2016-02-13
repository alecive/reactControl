% Author: Matej Hoffmann
clear; 

visualize_time_stats = true;
visualize_target = true;
visualize_all_joint_pos = true;
visualize_all_joint_vel = true;
visualize_single_joint_in_detail = false;
save_figs = true;
chosen_time_column = 6; % 6 for receivet, 4 for sender

%path_prefix = 'input/';
path_prefix = 'icubTests/test_20160212e/';
path_prefix_dumper = 'data/';

if save_figs
  mkdir('output'); 
end

% param file - columns: 1:#DOF, then joint pos min max for every DOF, then joint vel limits for every DOF
d_params=importdata([path_prefix 'param.log']);

NR_EXTRA_TIME_COLUMNS = 4; % these will be created so that there is time starting from 0, plus time increment column - this 2 times (sender and receiver time stamp) - for diagnostics

% data file -  in columns on the output for 10 DOF case:
%1: packetID, 2: sender time stamp, 3:receiver time stamp, 4: nActiveDOF in chain
% 5: time from 0, sender; 6: increments of 5; 7: time from 0 receiver; 8: increments in 7 
% 9:11 desired final target (for end-effector), 12:14 current end effector position 
% 15-17 current desired target given by particle (for end-effector)
%variable - if torso on: 18:27: joint velocities as solution from ipopt and sent to robot 
%variable - if torso on: 28:37: joint positions as solution to control and sent to robot 
%variable - if torso on: 38:57; joint vel limits as input to ipopt, after avoidanceHandler
%assuming it is row by row, so min_1, max_1, min_2, max_2 etc.
d_orig=importdata([path_prefix path_prefix_dumper 'reactCtrl/data.log']);


TIME_FROM_ZERO_1_COLUMN = 4;
TIME_FROM_ZERO_DELTA_1_COLUMN = 5;
TIME_FROM_ZERO_2_COLUMN = 6;
TIME_FROM_ZERO_DELTA_2_COLUMN = 7;
PACKET_ID_COLUMN = 1;
TIME_ABS_1_COLUMN = 2;
TIME_ABS_2_COLUMN = 3;

target_x.column = 9;
target_y.column = 10;
target_z.column = 11;
    
targetEE_x.column = 15;
targetEE_y.column = 16;
targetEE_z.column = 17;
    
EE_x.column = 12;
EE_y.column = 13;
EE_z.column = 14;
    
joint_info(1).name = '1st torso - pitch'; joint_info(2).name = '2nd torso - roll'; joint_info(3).name = '3rd torso - yaw'; 
joint_info(4).name = '1st shoulder'; joint_info(5).name = '2nd shoulder'; joint_info(6).name = '3rd shoulder'; 
joint_info(7).name = '1st elbow'; joint_info(8).name = '2nd elbow'; 
joint_info(9).name = '1st wrist'; joint_info(10).name = '2nd wrist'; 

if((d_params(1) == 10) && (d_orig(1,4) == 10) ) % 10 DOF situation - 3 torso, 7 arm
    chainActiveDOF = 10;
        
    for i=1:chainActiveDOF
        joint_info(i).pos_limit_min = d_params(2*i); joint_info(i).pos_limit_max = d_params(2*i+1);
        joint_info(i).vel_limit_min = d_params(20+2*i); joint_info(i).vel_limit_max = d_params(20+2*i+1);
        joint_info(i).vel_column = 13+i+NR_EXTRA_TIME_COLUMNS; joint_info(i).pos_column = 23+i+NR_EXTRA_TIME_COLUMNS; 
        joint_info(i).vel_limit_min_avoid_column = 32+2*i+NR_EXTRA_TIME_COLUMNS; joint_info(i).vel_limit_max_avoid_column = 32+2*i+1+NR_EXTRA_TIME_COLUMNS;
    end
else
   error('This script is currently not supporting other than 10 DOF chains'); 
end



%% create extra time columns

    nr_rows_dat = size(d_orig,1);
    time_cols_to_insert = NaN(nr_rows_dat,4);
    time_cols_to_insert(1,1)=0; time_cols_to_insert(1,2)=0;time_cols_to_insert(1,3)=0; time_cols_to_insert(1,4)=0;
    for i=2:nr_rows_dat
        time_cols_to_insert(i,1)=d_orig(i,2)-d_orig(1,2); % sender time stamp
        time_cols_to_insert(i,2)=time_cols_to_insert(i,1)-time_cols_to_insert(i-1,1); % increment - to see if it is stable
        time_cols_to_insert(i,3)=d_orig(i,3)-d_orig(1,3); % this would be the receiver time stamp
        time_cols_to_insert(i,4)=time_cols_to_insert(i,3)-time_cols_to_insert(i-1,3); % increment - to see if it is stable
    end
    d = [d_orig(:,1:3) time_cols_to_insert(:,1:4) d_orig(:,4:end)];
    

%% info about data matrix

sz=size(d);
L=sz(1); % ~ nr. rows

t = d(:,chosen_time_column); 


%% plot time diagnostics

if visualize_time_stats
    f13 = figure(13); clf;
    set(f13,'Name','Time statistics');

    subplot(3,1,1);
        title('Absolute time');
        hold on;
          plot(d(:,TIME_ABS_1_COLUMN),'b+');
          plot(d(:,TIME_ABS_2_COLUMN),'k+');
        hold off;
        legend('time (sender)','time (receiver)');
        ylabel('Time (s)');
       
    subplot(3,1,2)
        title('Time starting from 0');
        hold on;
          plot(d(:,TIME_FROM_ZERO_1_COLUMN),'b+');
          plot(d(:,TIME_FROM_ZERO_2_COLUMN),'k+');
        hold off;
        legend('time (sender)','time (receiver)');
        ylabel('Time (s)');
       
     subplot(3,1,3);
           title(' Time increments');
           hold on;
             plot(d(2:end,TIME_FROM_ZERO_DELTA_1_COLUMN),'b+');
             plot(d(2:end,TIME_FROM_ZERO_DELTA_2_COLUMN),'k+');
           hold off;
           legend('joints (sender)','joints (receiver)');
           ylabel('Sampling - delta time (s)');
       
    if save_figs
       saveas(f13,'TimeStats.fig'); 
    end

end

%% reference vs. end-effector
if visualize_target

    f1 = figure(1); clf(f1); set(f1,'Color','white','Name','Target, reference, end-effector');  
    hold on; axis equal; view([-130 30]); grid;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

    plot3(d(:,target_x.column),d(:,target_y.column),d(:,target_z.column),'r*','LineWidth',6); % plots the desired target trajectory
    %plot3(d(end,5),d(end,6),d(end,7),'r*','LineWidth',10); % plots the desired target final pos

    plot3(d(:,targetEE_x.column),d(:,targetEE_y.column),d(:,targetEE_z.column),'go','LineWidth',2); % plots the end-eff targets as given by particle
    plot3(d(end,targetEE_x.column),d(end,targetEE_y.column),d(end,targetEE_z.column),'go','LineWidth',4); % plots the end-eff target final pos

    plot3(d(:,EE_x.column),d(:,EE_y.column),d(:,EE_z.column),'k.','LineWidth',3); % plots the end-eff trajectory
    plot3(d(end,EE_x.column),d(end,EE_y.column),d(end,EE_z.column),'kx','LineWidth',6); % plots the end-eff final pos

    hold off;

    f2 = figure(2); clf(f2); set(f2,'Color','white','Name','Distance Reference vs. end-effector');  
    xlabel('time (s)');
    [ax,h1,h2] = plotyy(t,myEuclDist3d_matrix(d(:,EE_x.column),d(:,EE_y.column),d(:,EE_z.column),d(:,targetEE_x.column),d(:,targetEE_y.column),d(:,targetEE_z.column)),...
        t,myEuclDist3d_matrix(d(:,EE_x.column),d(:,EE_y.column),d(:,EE_z.column),d(:,target_x.column),d(:,target_y.column),d(:,target_z.column)));
    set(h1,'Marker','o','MarkerSize',10,'Color','b');
    set(h2,'Marker','*','MarkerSize',10,'Color','g');
    legend('Distance end-eff to current target','Distance end-eff to final target');
    set(get(ax(1),'Ylabel'),'String','Distance (m)'); 
    set(get(ax(2),'Ylabel'),'String','Distance (m)');


    if save_figs
       saveas(f1,'output/TargetReferenceEndeffectorTrajectories.fig');
       saveas(f2,'output/TargetReferenceEndeffectorDistances.fig');
    end

end

%% joint values vs. joint limits
if visualize_all_joint_pos
    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
        data = [];
        f3 = figure(3); clf(f3); set(f3,'Color','white','Name','Joint positions - No avoidance');  
       % f4 = figure(4); clf(f4); set(f4,'Color','white','Name','Joint positions - Visual avoidance');  
       % f5 = figure(5); clf(f5); set(f5,'Color','white','Name','Joint positions - Tactile avoidance');  

        for i=1:1 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f3); 
                    data = d;
                case 2 
                    set(0, 'currentfigure', f4); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f5); 
                    data = d_t;
            end
            
            for j=1:chainActiveDOF
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
        %saveas(f4,'output/JointPositionsVisualAvoidance.fig');
        %saveas(f5,'output/JointPositionsTactileAvoidance.fig');
    end    
end




%% joint velocities vs. joint vel limits
if visualize_all_joint_vel
    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
       
        data = [];
        f6 = figure(6); clf(f6); set(f6,'Color','white','Name','Joint velocities - No avoidance');  
        %f7 = figure(7); clf(f7); set(f7,'Color','white','Name','Joint velocities - Visual avoidance');  
        %f8 = figure(8); clf(f8); set(f8,'Color','white','Name','Joint velocities - Tactile avoidance');  

        for i=1:1 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f6); 
                    data = d;
                case 2 
                    set(0, 'currentfigure', f7); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f8); 
                    data = d_t;
            end
            
            for j=1:chainActiveDOF
                subplot(4,3,j); hold on;
                plot(t,data(:,joint_info(j).vel_limit_min_avoid_column),'--c','Marker','v','MarkerSize',2); % current min joint vel limit set by avoidance handler
                plot(t,data(:,joint_info(j).vel_limit_max_avoid_column),'--m','Marker','^','MarkerSize',2); % current max joint vel limit set by avoidance handler
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
        %saveas(f7,'output/JointVelocitiesVisualAvoidance.fig');
        %saveas(f8,'output/JointVelocitiesTactileAvoidance.fig');
    end
end




%% individual joint in detail 
if visualize_single_joint_in_detail

    j = 9; % joint index to visualize

    if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
        data = [];
        f10 = figure(10); clf(f10); set(f10,'Color','white','Name',['No avoidance - ' joint_info(j).name]);  
        %f11 = figure(11); clf(f11); set(f11,'Color','white','Name',['Visual avoidance - ' joint_info(j).name]);  
        %f12 = figure(12); clf(f12); set(f12,'Color','white','Name',['Tactile avoidance - ' joint_info(j).name]);  

        for i=1:1 % for all variants of the simulation
            switch i
                case 1 
                    set(0, 'currentfigure', f10); 
                    data = d;
                case 2 
                    set(0, 'currentfigure', f11); 
                    data = d_v;
                case 3 
                    set(0, 'currentfigure', f12); 
                    data = d_t;
            end
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
        %saveas(f11,'output/SelectedJointDetailVisualAvoidance.fig');
        %saveas(f12,'output/SelectedJointDetailTactileAvoidance.fig');
    end
end




