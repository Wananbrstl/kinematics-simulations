%% Author : Vineet Kumar
%  Date : 5 Dec 2017

%% Input: Links Dimensions

clear all;
clc;
% Link lengths
a=6;
b=4;
c=4;
d=6;
r=2.7;  % Radius of circle
omega=0.1;  % Angular speed of circular link
t1=0:0.3:pi/(2*omega);  % Closing-time control for gripper 
t2=pi/(2*omega):-0.3:0; % Opening-time control for gripper
t=[t1 t2];  % Appending opening and closing of gripper
theta=omega*t;  % Angular displacement
centre=[0;0];   % Centre Coordinates for circular link

%% Calculations

% Change in position along horizontal direction as function of angular displacement
delta_x=-1*sqrt((r^2)*((cos(theta)).^2-1)+b^2)-r*(cos(theta)-1)+b;

% Coordinates of kinematic joints
p1=[-(b+r)+delta_x;-6*ones(1,length(delta_x))];
p2=[-(b+r)+delta_x;zeros(1,length(delta_x))];
p3=[-1*r*cos(theta);r*sin(theta)];
p4=[r*cos(theta);-1*r*sin(theta)];  
p5=[b+r-delta_x;zeros(1,length(delta_x))];
p6=[b+r-delta_x;-6*ones(1,length(delta_x))];

%% Loop control for animating kinematic motion

% To open figure window in full-screen
figure('units','normalized','outerposition',[0 0 1 1])

for i=1:length(t)
    plot1=subplot(1,1,1);
    title('Mechanical Gripper Mechanism');
    text(0,0,'(0,0)');  % Locate origin
    circle=viscircles(centre',r);   % Mark centre-point of circular link
    circle_centre=viscircles(centre',0.05);    % Plot circular link
    
    % Plot joints of every link
    p1_circle=viscircles(p1(:,i)',0.05,'Color','r');
    p2_circle=viscircles(p2(:,i)',0.05,'Color','g');
    p3_circle=viscircles(p3(:,i)',0.05);
    p4_circle=viscircles(p4(:,i)',0.05);
    p5_circle=viscircles(p5(:,i)',0.05,'Color','b');
    p6_circle=viscircles(p6(:,i)',0.05,'Color','y');
    
    % Plot links by drawing lines through joint coordinates
    link_a=line([p1(1,i) p2(1,i)],[p1(2,i) p2(2,i)]);
    link_b=line([p2(1,i) p3(1,i)],[p2(2,i) p3(2,i)]);
    link_c=line([p4(1,i) p5(1,i)],[p4(2,i) p5(2,i)]);
    link_d=line([p5(1,i) p6(1,i)],[p5(2,i) p6(2,i)]);
    
    % Motion tracking : To disable select the text and press Ctrl+R
    track1=viscircles(p1(:,i)',0.01,'Color','r');
    track2=viscircles(p2(:,i)',0.01,'Color','g');
    track3=viscircles(p3(:,i)',0.01,'Color','b');
    track4=viscircles(p4(:,i)',0.01,'Color','b');
    track5=viscircles(p5(:,i)',0.01,'Color','g');
    track6=viscircles(p6(:,i)',0.01,'Color','r');
    
    % Annontations and texts
    t1=text(p1(1,i),p1(2,i),' P1');
    t2=text(p2(1,i),p2(2,i),' P2');
    t3=text(p3(1,i),p3(2,i),' P3');
    t4=text(p4(1,i),p4(2,i),' P4');
    t5=text(p5(1,i),p5(2,i),' P5');
    t6=text(p6(1,i),p6(2,i),' P6');
    
    if(t(i+1)-t(i)>0 && i<length(t))
        str = 'Gripper Closing Action';
        dim=[0.6 0.5 0.4 0.4];
        x=annotation('textbox',dim,'String',str,'FitBoxToText','on');
    end;
    if(t(i+1)-t(i)<0 && i<length(t))
        str = 'Gripper Opening Action';
        dim=[0.6 0.5 0.4 0.4];
        x=annotation('textbox',dim,'String',str,'FitBoxToText','on');
    end;
    
    % Plot features
    axis(plot1,'equal');
    xlim([-10 10]);
    ylim([-10 10]);
    pause(0.005);
    grid on;
    
    % Delete all points, links and annotations after every iteration
    if(i<length(t))
        delete(p1_circle);
        delete(p2_circle);
        delete(p3_circle);
        delete(p4_circle);
        delete(p5_circle);
        delete(p6_circle);
        delete(link_a);
        delete(link_b);
        delete(link_c);
        delete(link_d);
        delete(t1);
        delete(t2);
        delete(t3);
        delete(t4);
        delete(t5);
        delete(t6);
        delete(x);
    end
end