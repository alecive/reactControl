% Author: Matej Hoffmann
clear; 

save_figs = false;
chosen_time_column = 6; % 6 for receiver, 4 for sender

path_prefix_sim = 'icubSimTests/';
path_prefix_arm = 'arm_joint_';

joints(1).path_prefix = [path_prefix_sim path_prefix_arm '0/'];
joints(1).name = '1st shoulder';
joints(2).path_prefix = [path_prefix_sim path_prefix_arm '1/'];
joints(2).name = '2nd shoulder' 

path_suffix = 'data/jointVelCtrlIdent/data.log';

if save_figs
  mkdir('output');
end

NR_EXTRA_TIME_COLUMNS = 4; % these will be created so that there is time starting from 0, plus time increment column - this 2 times (sender and receiver time stamp) - for diagnostics

% data file -  in columns on the output, after inserting extra time columns:
%1: packetID, 2: sender time stamp, 3:receiver time stamp, 
%4: time from 0, sender; 5: increments of 4; 6: time from 0 receiver; 7: increments in 6 
% 8: counter, 9: velocity command, 10: reference 
%11-12: pwm; 13: joint pos feedback; 14: vel estimated from derivative of
%position feedback

for i=1:size(joints)
    joints(i).d_orig=importdata([joints(i).path_prefix path_suffix]);
end

TIME_FROM_ZERO_1_COLUMN = 4;
TIME_FROM_ZERO_DELTA_1_COLUMN = 5;
TIME_FROM_ZERO_2_COLUMN = 6;
TIME_FROM_ZERO_DELTA_2_COLUMN = 7;
PACKET_ID_COLUMN = 1;
TIME_ABS_1_COLUMN = 2;
TIME_ABS_2_COLUMN = 3;

vel_command_col = 9;
pos_fb_col = 13;
vel_estimated_col = 14;



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