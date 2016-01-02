function run(filein)
% Author: Ugo Pattacini

global P;
global t t0;
global xd q ctrlp;
global hax hg1 hg2;
global tm;

data=importdata(filein);
t=data(:,1);
xd=data(:,2:4);
q=data(:,15:15+10-1);
ctrlp=data(:,15+10:end);

P{1}.A =0.032;      P{1}.D =0;        P{1}.alpha =pi/2;  P{1}.offset =0;
P{2}.A =0;          P{2}.D =-0.0055;  P{2}.alpha =pi/2;  P{2}.offset =-pi/2;
P{3}.A =0.0233647;  P{3}.D =-0.1433;  P{3}.alpha =-pi/2; P{3}.offset =pi/2+15*pi/180;
P{4}.A =0;          P{4}.D =0.10774;  P{4}.alpha =-pi/2; P{4}.offset =pi/2;
P{5}.A =0;          P{5}.D =0;        P{5}.alpha =pi/2;  P{5}.offset =-pi/2;
P{6}.A =0.015;      P{6}.D =0.15228;  P{6}.alpha =-pi/2; P{6}.offset =pi/2-15*pi/180;
P{7}.A =-0.015;     P{7}.D =0;        P{7}.alpha =pi/2;  P{7}.offset =0;
P{8}.A =0;          P{8}.D =0.1373;   P{8}.alpha =pi/2;  P{8}.offset =-pi/2;
P{9}.A =0;          P{9}.D =0;        P{9}.alpha =pi/2;  P{9}.offset =pi/2;
P{10}.A=0.0625;     P{10}.D=-0.016;   P{10}.alpha=0;     P{10}.offset=0;

hfig=figure('Name','iCub Arm');
set(hfig,'Toolbar','figure');
hold on; view(3); grid;
xlim([-0.6 0.2]); xlabel('x [m]');
ylim([-0.6 0.6]); ylabel('y [m]');
zlim([-0.6 0.6]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.2;
    
quiver3(hax,0,0,0,A/2,0,0,'Color','r','Linewidth',2);
quiver3(hax,0,0,0,0,A/2,0,'Color','g','Linewidth',2);
quiver3(hax,0,0,0,0,0,A/2,'Color','b','Linewidth',2);

hg1=[];
hg2=[];

set(hfig,'CloseRequestFcn',@Quit);
tm=timer('Period',0.1,'ExecutionMode','fixedRate',...
         'TimerFcn',@PlotQuantities);
t0=cputime;
start(tm);
      

%--------------------------------------------------------------------------
function T=DH(n,theta)

global P;

theta=theta+P{n}.offset;
c_theta=cos(theta);
s_theta=sin(theta);
c_alpha=cos(P{n}.alpha);
s_alpha=sin(P{n}.alpha);

T=[[c_theta -s_theta*c_alpha  s_theta*s_alpha P{n}.A*c_theta];...
   [s_theta  c_theta*c_alpha -c_theta*s_alpha P{n}.A*s_theta];...
   [      0          s_alpha          c_alpha         P{n}.D];...
   [      0                0                0              1]];


%--------------------------------------------------------------------------
function [x,axpoint]=fkin(q)

q=q*pi/180;

T0 =[[0 -1 0 0]; [0 0 -1 0]; [1 0 0 0]; [0 0 0 1]];
T1 =T0*DH(1,q(1));
T2 =T1*DH(2,q(2));
T3 =T2*DH(3,q(3));
T4 =T3*DH(4,q(4));
T5 =T4*DH(5,q(5));
T6 =T5*DH(6,q(6));
T7 =T6*DH(7,q(7));
T8 =T7*DH(8,q(8));
T9 =T8*DH(9,q(9));
T10=T9*DH(10,q(10));

x{1} =T0(1:3,4);
x{2} =T1(1:3,4);
x{3} =T2(1:3,4);
x{4} =T3(1:3,4);
x{5} =T4(1:3,4);
x{6} =T5(1:3,4);
x{7} =T6(1:3,4);
x{8} =T7(1:3,4);
x{9} =T8(1:3,4);
x{10}=T9(1:3,4);
x{11}=T10(1:3,4);

axpoint{1}=T10(1:3,1);
axpoint{2}=T10(1:3,2);
axpoint{3}=T10(1:3,3);


%--------------------------------------------------------------------------
function hg=drawArm(x,axpoint,ctrlpoint)

global hax;

lim=axis(hax);
A=max(abs(lim))*0.2;

arm=plot3(hax,[x{1}(1) x{2}(1) x{3}(1) x{4}(1) x{5}(1) x{6}(1) x{7}(1) x{8}(1) x{9}(1) x{10}(1) x{11}(1)],...
              [x{1}(2) x{2}(2) x{3}(2) x{4}(2) x{5}(2) x{6}(2) x{7}(2) x{8}(2) x{9}(2) x{10}(2) x{11}(2)],...
              [x{1}(3) x{2}(3) x{3}(3) x{4}(3) x{5}(3) x{6}(3) x{7}(3) x{8}(3) x{9}(3) x{10}(3) x{11}(3)],...
              'Color','k','LineWidth',3);
     
axpoint{1}=axpoint{1}*A;
axpoint{2}=axpoint{2}*A;
axpoint{3}=axpoint{3}*A;

ax(1)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{1}(1),axpoint{1}(2),axpoint{1}(3),...
              'Color','r','Linewidth',2);
ax(2)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{2}(1),axpoint{2}(2),axpoint{2}(3),...
              'Color','g','Linewidth',2);
ax(3)=quiver3(hax,x{end}(1),x{end}(2),x{end}(3),...
              axpoint{3}(1),axpoint{3}(2),axpoint{3}(3),...
              'Color','b','Linewidth',2);
       
hg=hggroup;
set(arm,'Parent',hg);
set(ax,'Parent',hg);

for i=1:3:length(ctrlpoint)
    p=plot3(hax,ctrlpoint(i+0),ctrlpoint(i+1),ctrlpoint(i+2),...
            'bo','LineWidth',1.5);
    set(p,'Parent',hg);
end


%--------------------------------------------------------------------------
function PlotQuantities(obj,event,string_arg) %#ok<INUSD>

global t t0;
global xd q ctrlp;
global hax hg1 hg2;
global tm;

dt=cputime-t0;
i=find(t>=dt,1);

if isempty(i)
    stop(tm);
    return;
end
  
if ~isempty(hg1)
    delete(hg1)
end

if ~isempty(hg2)
    delete(hg2)
end

[x,axpoint]=fkin(q(i,:));
hg1=drawArm(x,axpoint,ctrlp(i,:));
hg2=plot3(hax,xd(i,1),xd(i,2),xd(i,3),'go','LineWidth',3);
drawnow;


%--------------------------------------------------------------------------
function Quit(src,eventdata) %#ok<INUSD>

global tm;

stop(tm);
delete(src);

