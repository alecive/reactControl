% Author: Matej Hoffmann
clear; 

visualize_time_stats = true;
save_figs = true;
chosen_time_column = 6; % 6 for receiver, 4 for sender

path_prefix_robot = 'icubSimTests/'; % 'icubSimTests/' or 'icubTests';
path_prefix_arm = 'arm_joint_';
path_prefix_torso = 'torso_joint_';

joints(1).path_prefix = [path_prefix_robot path_prefix_torso '0/'];
joints(1).name = 'torso - yaw';
joints(2).path_prefix = [path_prefix_robot path_prefix_torso '1/'];
joints(2).name = 'torso - roll';
joints(3).path_prefix = [path_prefix_robot path_prefix_torso '2/'];
joints(3).name = 'torso - pitch';

joints(4).path_prefix = [path_prefix_robot path_prefix_arm '0/'];
joints(4).name = '1st shoulder';
joints(5).path_prefix = [path_prefix_robot path_prefix_arm '1/'];
joints(5).name = '2nd shoulder'; 
joints(6).path_prefix = [path_prefix_robot path_prefix_arm '2/'];
joints(6).name = '3rd shoulder'; 
joints(7).path_prefix = [path_prefix_robot path_prefix_arm '3/'];
joints(7).name = '1st elbow'; 
joints(8).path_prefix = [path_prefix_robot path_prefix_arm '4/'];
joints(8).name = '2nd elbow'; 


path_suffix = 'data/jointVelCtrlIdent/data.log';

NR_EXTRA_TIME_COLUMNS = 4; % these will be created so that there is time starting from 0, plus time increment column - this 2 times (sender and receiver time stamp) - for diagnostics

% data file -  in columns on the output, after inserting extra time columns:
%1: packetID, 2: sender time stamp, 3:receiver time stamp, 
%4: time from 0, sender; 5: increments of 4; 6: time from 0 receiver; 7: increments in 6 
% 8: counter, 9: velocity command, 10: reference 
%11 or 11-12: pwm; 12/13: joint pos feedback; 13/14: vel estimated from derivative of
%position feedback

PACKET_ID_COLUMN = 1;
TIME_ABS_1_COLUMN = 2;
TIME_ABS_2_COLUMN = 3;
TIME_FROM_ZERO_1_COLUMN = 4;
TIME_FROM_ZERO_DELTA_1_COLUMN = 5;
TIME_FROM_ZERO_2_COLUMN = 6;
TIME_FROM_ZERO_DELTA_2_COLUMN = 7;
vel_command_col = 9;
        
for i=1:length(joints)
    joints(i).d_orig=importdata([joints(i).path_prefix path_suffix]);
    if ( size(joints(i).d_orig,2) == 9) % joints with 1 pwm col 
        joints(i).pos_fb_col = 12;
        joints(i).vel_estimated_col = 13;
    elseif (size(joints(i).d_orig,2) == 10)  % joints with 2 pwm cols
        joints(i).pos_fb_col = 13;
        joints(i).vel_estimated_col = 14;
    else
        error('Unexptected nr cols');
    end
end


%% create extra time columns

for i=1:length(joints)
    nr_rows_dat = size(joints(i).d_orig,1);
    time_cols_to_insert = NaN(nr_rows_dat,4);
    time_cols_to_insert(1,1)=0; time_cols_to_insert(1,2)=0;time_cols_to_insert(1,3)=0; time_cols_to_insert(1,4)=0;
    for j=2:nr_rows_dat
        time_cols_to_insert(j,1)=joints(i).d_orig(j,2)-joints(i).d_orig(1,2); % sender time stamp
        time_cols_to_insert(j,2)=time_cols_to_insert(j,1)-time_cols_to_insert(j-1,1); % increment - to see if it is stable
        time_cols_to_insert(j,3)=joints(i).d_orig(j,3)-joints(i).d_orig(1,3); % this would be the receiver time stamp
        time_cols_to_insert(j,4)=time_cols_to_insert(j,3)-time_cols_to_insert(j-1,3); % increment - to see if it is stable
    end
    joints(i).d = [joints(i).d_orig(:,1:3) time_cols_to_insert(:,1:4) joints(i).d_orig(:,4:end)];
    
end
    
%% info about data matrix

for i=1:length(joints)
    sz=size(joints(i).d);
    joints(i).L=sz(1); % ~ nr. rows
    joints(i).t = joints(i).d(:,chosen_time_column); 
end

%% plot time diagnostics

if visualize_time_stats
    f1 = figure(1); clf;
    set(f1,'Name','Time increments');
  
   for i=1:length(joints)    
     subplot(3,3,i);
       hold on;
        title(joints(i).name);
        plot(joints(i).d(2:end,TIME_FROM_ZERO_DELTA_1_COLUMN),'b+');
        plot(joints(i).d(2:end,TIME_FROM_ZERO_DELTA_2_COLUMN),'k+');
        if (i==1)
            legend('time stamp (sender)','time stamp (receiver)');
        end
      hold off;
   end       
           
   if save_figs
     saveas(f1,'TimeStats.fig'); 
   end

end

%% plot frequency response


f2 = figure(2); clf;
set(f2,'Name','Frequency response');
   
   
    for i=1:length(joints)    
     subplot(3,3,i);
       hold on;
        title(joints(i).name);
        [AX,H1,H2] = plotyy(joints(i).t,joints(i).d(:,joints(i).pos_fb_col),joints(i).t,joints(i).d(:,vel_command_col));   
         hold(AX(1),'on');hold(AX(2),'on');
         H3 = plot(AX(2),joints(i).t,joints(i).d(:,joints(i).vel_estimated_col), 'Parent', AX(2), 'LineStyle',':','Color','c','Marker','v','MarkerSize',2);
         hold(AX(1),'off');hold(AX(2),'off');
         set(get(AX(1),'Ylabel'),'String','joint pos [deg]'); 
         %set(AX(1),'Ylim',[joint_info(j).pos_limit_min-5 joint_info(j).pos_limit_max+5]); 
         set(get(AX(2),'Ylabel'),'String','Joint velocity [deg/s]');
         %set(AX(2),'Ylim',[joint_info(j).vel_limit_min-3 joint_info(j).vel_limit_max+3]); 
         xlabel('t [s]');
         if (i==1)
             legend('pos','vel','vel estimate');
         end
         %set(H1,'LineStyle','-');
         %set(H2,'LineStyle','--');
      hold off;
    end 
   
   if save_figs
     saveas(f2,'FrequencyResponse.fig'); 
   end

