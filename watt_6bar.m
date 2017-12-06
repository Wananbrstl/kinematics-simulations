%% Author : Vineet Kumar
% Date : 6-December-2017

% Accepting Inputs from user
clear all;
clc;
fprintf('---All angles for input must be provided in DEGREES---\n');
d=input('Enter r1 = ');
a=input('Enter r2 = ');
b=input('Enter r3 = ');
c=input('Enter r4 = ');
r5=input('Enter r5 = ');
r6=input('Enter r6 = ');
r7=input('Enter r7 = ');
r8=input('Enter r8 = ');
theta1=deg2rad(input('Enter theta1 = '));
theta8=deg2rad(input('Enter theta8 = '));
gamma=deg2rad(input('Enter gamma (angle between r4 and r5) = '));
theta2=deg2rad(input('Enter INPUT angle theta2 = '));

%% Calculations

%Calculation for angle theta4
k1=(b^2-(a^2+c^2+d^2)+2*a*d*cos(theta2-theta1))/(2*c);
A1=k1+d*cos(theta1)-a*cos(theta2);
B1=2*(d*sin(theta1)-a*sin(theta2));
C1=k1+a*cos(theta2)-d*cos(theta1);
D1=B1^2-4*A1*C1;
theta4_1=(-2*atan((-1*B1+sqrt(D1))/(2*A1)));
theta4_2=(-2*atan((-1*B1-sqrt(D1))/(2*A1)));

%Calculation for angle theta3
k2=(c^2-(a^2+b^2+d^2)+2*a*d*cos(theta2-theta1))/(2*b);
A2=k2+a*cos(theta2)-d*cos(theta1);
B2=2*(d*sin(theta1)-a*sin(theta2));
C2=k2+d*cos(theta1)-a*cos(theta2);
D2=B2^2-4*A2*C2;
theta3_1=(2*atan((-1*B2+sqrt(D2))/(2*A2)));
theta3_2=(2*atan((-1*B2-sqrt(D2))/(2*A2)));

%Calculation for angle theta5
theta5_1=(theta4_1-gamma);
theta5_2=(theta4_2-gamma);

%Calculation for angle theta7
k3=(r6^2-(r5^2+r7^2+r8^2)+2*r5*r8*cos(theta5_1-theta8))/(2*r7);
A3=k3+r8*cos(theta8)-r5*cos(theta5_1);
B3=2*(r8*sin(theta8)-r5*sin(theta5_1));
C3=k3-r8*cos(theta8)+r5*cos(theta5_1);
D3=B3^2-4*A3*C3;
theta7_1=(-2*atan((-1*B3+sqrt(D3))/(2*A3)));
theta7_2=(-2*atan((-1*B3-sqrt(D3))/(2*A3)));

k3_1=(r6^2-(r5^2+r7^2+r8^2)+2*r5*r8*cos(theta5_2-theta8))/(2*r7);
A3_1=k3_1+r8*cos(theta8)-r5*cos(theta5_2);
B3_1=2*(r8*sin(theta8)-r5*sin(theta5_2));
C3_1=k3_1-r8*cos(theta8)+r5*cos(theta5_2);
D3_1=B3_1^2-4*A3_1*C3_1;
theta7_1_1=(-2*atan((-1*B3_1+sqrt(D3_1))/(2*A3_1)));
theta7_2_2=(-2*atan((-1*B3_1-sqrt(D3_1))/(2*A3_1)));

%Calculation for angle theta8
k4=(r7^2-(r5^2+r6^2+r8^2)+2*r5*r8*cos(theta5_1-theta8))/(2*r6);
A4=k4+r5*cos(theta5_1)-r8*cos(theta8);
B4=2*(r8*sin(theta8)-r5*sin(theta5_1));
C4=k4+r8*cos(theta8)-r5*cos(theta5_1);
D4=B4^2-4*A4*C4;
theta6_1=(2*atan((-1*B4+sqrt(D4))/(2*A4)));
theta6_2=(2*atan((-1*B4-sqrt(D4))/(2*A4)));

k4_1=(r7^2-(r5^2+r6^2+r8^2)+2*r5*r8*cos(theta5_2-theta8))/(2*r6);
A4_1=k4_1+r5*cos(theta5_2)-r8*cos(theta8);
B4_1=2*(r8*sin(theta8)-r5*sin(theta5_2));
C4_1=k4_1+r8*cos(theta8)-r5*cos(theta5_2);
D4_1=B4_1^2-4*A4_1*C4_1;
theta6_1_1=(2*atan((-1*B4_1+sqrt(D4_1))/(2*A4_1)));
theta6_2_2=(2*atan((-1*B4_1-sqrt(D4_1))/(2*A4_1)));

%% Finding number of possible configurations
num_config=4;
config1=1;
config2=1;
config3=1;
config4=1;

if(imag(theta3_2)~=0 || imag(theta4_2)~=0 || imag(theta5_2)~=0 || imag(theta6_2_2)~=0 || imag(theta7_2_2)~=0)
    num_config=num_config-1;
    config1=0;
end
if(imag(theta3_2)~=0 || imag(theta4_2)~=0 || imag(theta5_2)~=0 || imag(theta6_1_1)~=0 || imag(theta7_1_1)~=0)
    num_config=num_config-1;
    config2=0;
end
if(imag(theta3_1)~=0 || imag(theta4_1)~=0 || imag(theta5_1)~=0 || imag(theta6_2)~=0 || imag(theta7_2)~=0)
    num_config=num_config-1;
    config3=0;
end
if(imag(theta3_1)~=0 || imag(theta4_1)~=0 || imag(theta5_1)~=0 || imag(theta6_1)~=0 || imag(theta7_1)~=0)
    num_config=num_config-1;
    config4=0;
end

%% Console Output
fprintf('\n---All output angles are in DEGREES---\n');
fprintf('\nThere are %i possible orientations :\n',num_config);
fprintf('\nOrientation 1 : Loop 1 OPEN & Loop 2 OPEN :\n');
if(config1~=0)
fprintf('theta3 = %.2f\n',rad2deg(theta3_2));
fprintf('theta4 = %.2f\n',rad2deg(theta4_2));
fprintf('theta5 = %.2f\n',rad2deg(theta5_2));
fprintf('theta6 = %.2f\n',rad2deg(theta6_2_2));
fprintf('theta7 = %.2f\n',rad2deg(theta7_2_2));
else
    fprintf('\nThis orientation is NOT possible for provided dimensions\n');
end
fprintf('\nOrientation 2 : Loop 1 OPEN & Loop 2 CROSSED :\n');
if(config2~=0)
fprintf('theta3 = %.2f\n',rad2deg(theta3_2));
fprintf('theta4 = %.2f\n',rad2deg(theta4_2));
fprintf('theta5 = %.2f\n',rad2deg(theta5_2));
fprintf('theta6 = %.2f\n',rad2deg(theta6_1_1));
fprintf('theta7 = %.2f\n',rad2deg(theta7_1_1));
else
    fprintf('\nThis orientation is NOT possible for provided dimensions\n');
end
fprintf('\nOrientation 3 : Loop 1 CROSSED & Loop 2 OPEN :\n');
if(config3~=0)
fprintf('theta3 = %.2f\n',rad2deg(theta3_1));
fprintf('theta4 = %.2f\n',rad2deg(theta4_1));
fprintf('theta5 = %.2f\n',rad2deg(theta5_1));
fprintf('theta6 = %.2f\n',rad2deg(theta6_2));
fprintf('theta7 = %.2f\n',rad2deg(theta7_2));
else
    fprintf('\nThis orientation is NOT possible for provided dimensions\n');
end
fprintf('\nOrientation 4 : Loop 1 CROSSED & Loop 2 CROSSED :\n');
if(config4~=0)
fprintf('theta3 = %.2f\n',rad2deg(theta3_1));
fprintf('theta4 = %.2f\n',rad2deg(theta4_1));
fprintf('theta5 = %.2f\n',rad2deg(theta5_1));
fprintf('theta6 = %.2f\n',rad2deg(theta6_1));
fprintf('theta7 = %.2f\n',rad2deg(theta7_1));
else
    fprintf('\nThis orientation is NOT possible for provided dimensions\n');
end

%% Plotting section

figure('units','normalized','outerposition',[0 0 1 1])
plot1=subplot(2,2,1);
title('CONFIGURATION 1 : LOOP1: OPEN & LOOP2: OPEN');
p1=[0;0];
p2=[a*cos(theta2);a*sin(theta2)];
p3=[a*cos(theta2)+b*cos(theta3_2);a*sin(theta2)+b*sin(theta3_2)];
p4=[d*cos(theta1);d*sin(theta1)];
p5=[d*cos(theta1)+r5*cos(theta5_2);d*sin(theta1)+r5*sin(theta5_2)];
p6=[d*cos(theta1)+r5*cos(theta5_2)+r6*cos(theta6_2_2);d*sin(theta1)+r5*sin(theta5_2)+r6*sin(theta6_2_2)];
p7=[d*cos(theta1)+r8*cos(theta8);d*sin(theta1)+r8*sin(theta8)];
if(config1~=0)
p1_circle=viscircles(p1',0.05,'Color','r');
p2_circle=viscircles(p2',0.05,'Color','r');
p3_circle=viscircles(p3',0.05,'Color','r');
p4_circle=viscircles(p4',0.05,'Color','r');
p5_circle=viscircles(p5',0.05,'Color','r');
p6_circle=viscircles(p6',0.05,'Color','r');
p7_circle=viscircles(p7',0.05,'Color','r');
link_a=line([p1(1) p2(1)],[p1(2) p2(2)]);
link_b=line([p2(1) p3(1)],[p2(2) p3(2)]);
link_c=line([p3(1) p4(1)],[p3(2) p4(2)]);
link_d=line([p1(1) p4(1)],[p1(2) p4(2)]);
link_e=line([p4(1) p5(1)],[p4(2) p5(2)]);
link_f=line([p5(1) p6(1)],[p5(2) p6(2)]);
link_g=line([p6(1) p7(1)],[p6(2) p7(2)]);
link_h=line([p4(1) p7(1)],[p4(2) p7(2)]);
link_i=line([p3(1) p5(1)],[p3(2) p5(2)]);
patch([p3(1) p4(1) p5(1)],[p3(2) p4(2) p5(2)],'red','FaceAlpha',.3);
else
    annotation('textbox',...
    'String',{'NOT POSSIBLE'},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
end
plot2=subplot(2,2,2);
title('CONFIGURATION 2 : LOOP1: OPEN & LOOP2: CROSSED');
p11=[0;0];
p22=[a*cos(theta2);a*sin(theta2)];
p33=[a*cos(theta2)+b*cos(theta3_2);a*sin(theta2)+b*sin(theta3_2)];
p44=[d*cos(theta1);d*sin(theta1)];
p55=[d*cos(theta1)+r5*cos(theta5_2);d*sin(theta1)+r5*sin(theta5_2)];
p66=[d*cos(theta1)+r5*cos(theta5_2)+r6*cos(theta6_1_1);d*sin(theta1)+r5*sin(theta5_2)+r6*sin(theta6_1_1)];
p77=[d*cos(theta1)+r8*cos(theta8);d*sin(theta1)+r8*sin(theta8)];
if(config2~=0)
p11_circle=viscircles(p11',0.05,'Color','r');
p22_circle=viscircles(p22',0.05,'Color','r');
p33_circle=viscircles(p33',0.05,'Color','r');
p44_circle=viscircles(p44',0.05,'Color','r');
p55_circle=viscircles(p55',0.05,'Color','r');
p66_circle=viscircles(p66',0.05,'Color','r');
p77_circle=viscircles(p77',0.05,'Color','r');
link_aa=line([p11(1) p22(1)],[p11(2) p22(2)]);
link_bb=line([p22(1) p33(1)],[p22(2) p33(2)]);
link_cc=line([p33(1) p44(1)],[p33(2) p44(2)]);
link_dd=line([p11(1) p44(1)],[p11(2) p44(2)]);
link_ee=line([p44(1) p55(1)],[p44(2) p55(2)]);
link_ff=line([p55(1) p66(1)],[p55(2) p66(2)]);
link_gg=line([p66(1) p77(1)],[p66(2) p77(2)]);
link_hh=line([p44(1) p77(1)],[p44(2) p77(2)]);
link_ii=line([p33(1) p55(1)],[p33(2) p55(2)]);
patch([p33(1) p44(1) p55(1)],[p33(2) p44(2) p55(2)],'blue','FaceAlpha',.3);
else
    annotation('textbox',...
    'String',{'NOT POSSIBLE'},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
end
plot3=subplot(2,2,3);
title('CONFIGURATION 3 : LOOP1: CROSSED & LOOP2: OPEN');
p1x=[0;0];
p2x=[a*cos(theta2);a*sin(theta2)];
p3x=[a*cos(theta2)+b*cos(theta3_1);a*sin(theta2)+b*sin(theta3_1)];
p4x=[d*cos(theta1);d*sin(theta1)];
p5x=[d*cos(theta1)+r5*cos(theta5_1);d*sin(theta1)+r5*sin(theta5_1)];
p6x=[d*cos(theta1)+r5*cos(theta5_1)+r6*cos(theta6_2);d*sin(theta1)+r5*sin(theta5_1)+r6*sin(theta6_2)];
p7x=[d*cos(theta1)+r8*cos(theta8);d*sin(theta1)+r8*sin(theta8)];
if(config3~=0)
p1x_circle=viscircles(p1x',0.05,'Color','r');
p2x_circle=viscircles(p2x',0.05,'Color','r');
p3x_circle=viscircles(p3x',0.05,'Color','r');
p4x_circle=viscircles(p4x',0.05,'Color','r');
p5x_circle=viscircles(p5x',0.05,'Color','r');
p6x_circle=viscircles(p6x',0.05,'Color','r');
p7x_circle=viscircles(p7x',0.05,'Color','r');
link_ax=line([p1x(1) p2x(1)],[p1x(2) p2x(2)]);
link_bx=line([p2x(1) p3x(1)],[p2x(2) p3x(2)]);
link_cx=line([p3x(1) p4x(1)],[p3x(2) p4x(2)]);
link_dx=line([p1x(1) p4x(1)],[p1x(2) p4x(2)]);
link_ex=line([p4x(1) p5x(1)],[p4x(2) p5x(2)]);
link_fx=line([p5x(1) p6x(1)],[p5x(2) p6x(2)]);
link_gx=line([p6x(1) p7x(1)],[p6x(2) p7x(2)]);
link_hx=line([p4x(1) p7x(1)],[p4x(2) p7x(2)]);
link_ix=line([p3x(1) p5x(1)],[p3x(2) p5x(2)]);
patch([p3x(1) p4x(1) p5x(1)],[p3x(2) p4x(2) p5x(2)],'green','FaceAlpha',.3);
else
    annotation('textbox',...
    'String',{'NOT POSSIBLE'},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
end
plot4=subplot(2,2,4);
title('CONFIGURATION 4 : LOOP1: CROSSED & LOOP2: CROSSED');
p1y=[0;0];
p2y=[a*cos(theta2);a*sin(theta2)];
p3y=[a*cos(theta2)+b*cos(theta3_1);a*sin(theta2)+b*sin(theta3_1)];
p4y=[d*cos(theta1);d*sin(theta1)];
p5y=[d*cos(theta1)+r5*cos(theta5_1);d*sin(theta1)+r5*sin(theta5_1)];
p6y=[d*cos(theta1)+r5*cos(theta5_1)+r6*cos(theta6_1);d*sin(theta1)+r5*sin(theta5_1)+r6*sin(theta6_1)];
p7y=[d*cos(theta1)+r8*cos(theta8);d*sin(theta1)+r8*sin(theta8)];
if(config4~=0)
p1y_circle=viscircles(p1y',0.05,'Color','r');
p2y_circle=viscircles(p2y',0.05,'Color','r');
p3y_circle=viscircles(p3y',0.05,'Color','r');
p4y_circle=viscircles(p4y',0.05,'Color','r');
p5y_circle=viscircles(p5y',0.05,'Color','r');
p6y_circle=viscircles(p6y',0.05,'Color','r');
p7y_circle=viscircles(p7y',0.05,'Color','r');
link_ay=line([p1y(1) p2y(1)],[p1y(2) p2y(2)]);
link_by=line([p2y(1) p3y(1)],[p2y(2) p3y(2)]);
link_cy=line([p3y(1) p4y(1)],[p3y(2) p4y(2)]);
link_dy=line([p1y(1) p4y(1)],[p1y(2) p4y(2)]);
link_ey=line([p4y(1) p5y(1)],[p4y(2) p5y(2)]);
link_fy=line([p5y(1) p6y(1)],[p5y(2) p6y(2)]);
link_gy=line([p6y(1) p7y(1)],[p6y(2) p7y(2)]);
link_hy=line([p4y(1) p7y(1)],[p4y(2) p7y(2)]);
link_iy=line([p3y(1) p5y(1)],[p3y(2) p5y(2)]);
patch([p3y(1) p4y(1) p5y(1)],[p3y(2) p4y(2) p5y(2)],'yellow','FaceAlpha',.3);
else
    annotation('textbox',...
    'String',{'NOT POSSIBLE'},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
end