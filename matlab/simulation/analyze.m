% Author: Ugo Pattacini

d_n=importdata('none.log');
d_v=importdata('visuo.log');
d_t=importdata('tactile.log');

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
