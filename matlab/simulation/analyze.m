% Author: Ugo Pattacini

% param file - columns: 1:#DOF, then joint pos min max for every DOF, then
% joint vel limits for every DOF
d_params=importdata('param.log');
% data file -  in columns on the output for 10 DOF case: 1:time, 2:4 target, 5:8 obstacle, 9:18 joint velocities, 19:28 joint pos, 29:end - control points   
d_n=importdata('none.log');
d_v=importdata('visuo.log');
d_t=importdata('tactile.log');

if(d_params(1) == 10) % 10 DOF situation - 3 torso, 7 arm
    joint_info(1).name = '1st torso - pitch'; joint_info(2).name = '2nd torso - roll'; joint_info(3).name = '3rd torso - yaw'; 
    joint_info(4).name = '1st shoulder'; joint_info(5).name = '2nd shoulder'; joint_info(6).name = '3rd shoulder'; 
    joint_info(7).name = '1st elbow'; joint_info(8).name = '2nd elbow'; 
    joint_info(9).name = '1st wrist'; joint_info(10).name = '2nd wrist'; 
    
    for i=1:10
        joint_info(i).pos_limit_min = d_params(2*i); joint_info(i).pos_limit_max = d_params(2*i+1);
        joint_info(i).vel_limit_min = d_params(20+2*i); joint_info(i).vel_limit_max = d_params(20+2*i+1);
        joint_info(i).pos_column = 18+i; joint_info(i).vel_column = 8+i;
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

% reference vs. end-effector
figure('Color','white');
hold; axis equal; view([-130 30]); grid;
xlim([-1.0 -0.2]); xlabel('x [m]');
ylim([-0.1 0.1]); ylabel('y [m]');
zlim([-0.2 0.2]); zlabel('z [m]');

plot3(d_n(:,29),d_n(:,30),d_n(:,31),'color',c_n,'linewidth',1.3);
plot3(-0.3+d_v(:,29),d_v(:,30),d_v(:,31),'color',c_v,'linewidth',1.3);
plot3(-0.6+d_t(:,29),d_t(:,30),d_t(:,31),'color',c_t,'linewidth',1.3);

legend({'none','vision','tactile'});

n=10;
[x,y,z]=sphere(n);
c(:,:,1)=ones(n);
c(:,:,2)=zeros(n);
c(:,:,3)=zeros(n);
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

% distance control-points/obstacle-center
figure('Color','white');

dist_n=zeros(L,3);
dist_v=zeros(L,3);
dist_t=zeros(L,3);
for i=1:L
    dist_n(i,:)=[norm(d_n(i,5:7)-d_n(i,29:31)) ...
                 norm(d_n(i,5:7)-d_n(i,32:34)) ...
                 norm(d_n(i,5:7)-d_n(i,35:37))];

    dist_v(i,:)=[norm(d_v(i,5:7)-d_v(i,29:31)) ...
                 norm(d_v(i,5:7)-d_v(i,32:34)) ...
                 norm(d_v(i,5:7)-d_v(i,35:37))];

    dist_t(i,:)=[norm(d_t(i,5:7)-d_t(i,29:31)) ...
                 norm(d_t(i,5:7)-d_t(i,32:34)) ...
                 norm(d_t(i,5:7)-d_t(i,35:37))];            
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


%% joint values vs. joint limits
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
        
        %torso joints
        subplot(4,3,1); hold on;
        plot(t,data(:,19));
        plot([t(1) t(end)],[d_params(2) d_params(2)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(3) d_params(3)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        ylabel('angle [deg]');
        title('1st torso joint (pitch)');
        hold off;
        
        
        subplot(4,3,2); hold on;
        plot(t,data(:,20));
        plot([t(1) t(end)],[d_params(4) d_params(4)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(5) d_params(5)],'r--'); % max joint pos limit
        xlabel('t [s]');
        %ylabel('angle [deg]');
        title('2nd torso joint (roll)');
        hold off;
        
        subplot(4,3,3); hold on;
        plot(t,data(:,21));
        plot([t(1) t(end)],[d_params(6) d_params(6)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(7) d_params(7)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        %ylabel('angle [deg]');
        title('3rd torso joint (yaw)');
        hold off;
        
        %shoulder joints
        subplot(4,3,4); hold on;
        plot(t,data(:,22));
        plot([t(1) t(end)],[d_params(8) d_params(8)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(9) d_params(9)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        ylabel('angle [deg]');
        title('1st shoulder joint');
        hold off;
        
        subplot(4,3,5); hold on;
        plot(t,data(:,23));
        plot([t(1) t(end)],[d_params(10) d_params(10)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(11) d_params(11)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        %ylabel('angle [deg]');
        title('2nd shoulder joint');
        hold off;
        
        subplot(4,3,6); hold on;
        plot(t,data(:,24));
        plot([t(1) t(end)],[d_params(12) d_params(12)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(13) d_params(13)],'r--'); % max joint pos limit
        xlabel('t [s]');
        %ylabel('angle [deg]');
        title('3rd shoulder joint');
        hold off;
        
        % elbow joints
        subplot(4,3,7); hold on;
        plot(t,data(:,25));
        plot([t(1) t(end)],[d_params(14) d_params(14)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(15) d_params(15)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('angle [deg]');
        title('1st elbow joint');
        hold off;
        
        subplot(4,3,8); hold on;
        plot(t,data(:,26));
        plot([t(1) t(end)],[d_params(16) d_params(16)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(17) d_params(17)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('angle [deg]');
        title('2nd elbow joint');
        hold off;
      
        %wrist joints
        subplot(4,3,10); hold on;
        plot(t,data(:,27));
        plot([t(1) t(end)],[d_params(18) d_params(18)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(19) d_params(19)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('angle [deg]');
        title('1st wrist joint');
        hold off;
        
        subplot(4,3,11); hold on;
        plot(t,data(:,28));
        plot([t(1) t(end)],[d_params(20) d_params(20)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(21) d_params(21)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('angle [deg]');
        title('2nd wrist joint');
        hold off;
    end
    
end


%% joint velocities vs. joint vel limits
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
        
        %torso joints
        subplot(4,3,1); hold on;
        plot(t,data(:,9));
        plot([t(1) t(end)],[d_params(22) d_params(22)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(23) d_params(23)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        ylabel('joint velocity [deg/s]');
        title('1st torso joint (pitch)');
        hold off;
        
        subplot(4,3,2); hold on;
        plot(t,data(:,10));
        plot([t(1) t(end)],[d_params(24) d_params(24)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(25) d_params(25)],'r--'); % max joint pos limit
        xlabel('t [s]');
        %ylabel('joint velocity [deg/s]');
        title('2nd torso joint (roll)');
        hold off;
        
        subplot(4,3,3); hold on;
        plot(t,data(:,11));
        plot([t(1) t(end)],[d_params(26) d_params(26)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(27) d_params(27)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        %ylabel('joint velocity [deg/s]');
        title('3rd torso joint (yaw)');
        hold off;
        
        %shoulder joints
        subplot(4,3,4); hold on;
        plot(t,data(:,12));
        plot([t(1) t(end)],[d_params(28) d_params(28)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(29) d_params(29)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        ylabel('joint velocity [deg/s]');
        title('1st shoulder joint');
        hold off;
        
        subplot(4,3,5); hold on;
        plot(t,data(:,13));
        plot([t(1) t(end)],[d_params(30) d_params(30)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(31) d_params(31)],'r--'); % max joint pos limit
        %xlabel('t [s]');
        %ylabel('joint velocity [deg/s]');
        title('2nd shoulder joint');
        hold off;
        
        subplot(4,3,6); hold on;
        plot(t,data(:,14));
        plot([t(1) t(end)],[d_params(32) d_params(32)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(33) d_params(33)],'r--'); % max joint pos limit
        xlabel('t [s]');
        %ylabel('joint velocity [deg/s]');
        title('3rd shoulder joint');
        hold off;
        
        % elbow joints
        subplot(4,3,7); hold on;
        plot(t,data(:,15));
        plot([t(1) t(end)],[d_params(34) d_params(34)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(35) d_params(35)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('joint velocity [deg/s]');
        title('1st elbow joint');
        hold off;
        
        subplot(4,3,8); hold on;
        plot(t,data(:,16));
        plot([t(1) t(end)],[d_params(36) d_params(36)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(37) d_params(37)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('angle [deg]');
        title('2nd elbow joint');
        hold off;
      
        %wrist joints
        subplot(4,3,10); hold on;
        plot(t,data(:,17));
        plot([t(1) t(end)],[d_params(38) d_params(38)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(39) d_params(39)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('joint velocity [deg/s]');
        title('1st wrist joint');
        hold off;
        
        subplot(4,3,11); hold on;
        plot(t,data(:,18));
        plot([t(1) t(end)],[d_params(40) d_params(40)],'r--'); % min joint pos limit
        plot([t(1) t(end)],[d_params(41) d_params(41)],'r--'); % max joint pos limit
        xlabel('t [s]');
        ylabel('joint velocity [deg/s]');
        title('2nd wrist joint');
        hold off;
    end
    
end


%% individual joint in detail 

j = 10; % joint index to visualize

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
     H3 = line([t t],[joint_info(j).pos_limit_min joint_info(j).pos_limit_min], 'Parent', AX(1), 'LineStyle','--');
     H4 = line([t t],[joint_info(j).pos_limit_max joint_info(j).pos_limit_max], 'Parent', AX(1), 'LineStyle','--');
     H5 = line([t t],[joint_info(j).vel_limit_min joint_info(j).vel_limit_min], 'Parent', AX(2), 'LineStyle',':');
     H6 = line([t t],[joint_info(j).vel_limit_max joint_info(j).vel_limit_max], 'Parent', AX(2), 'LineStyle',':');
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



