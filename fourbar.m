a=2;
b=2;
c=2;
d=3;
t=0:0.05:10;
omega=2;
theta=omega*t;
e=sqrt(a^2+d^2-2*a*d*cos(theta));
alpha=asin(a*sin(theta)./e);
beta=acos((e.^2+c^2-b^2)./(2*e*c));
p1=[0;0];
p2=a*[cos(theta);sin(theta)];
p3=[d-c*cos(alpha+beta);c*sin(alpha+beta)];
p4=d*[1;0];
p3_x=p3(1,:);
p3_y=p3(2,:);
p3_vx=diff(p3_x)./diff(t);
p3_vy=diff(p3_y)./diff(t);
p3_v=sqrt(p3_vx.^2+p3_vy.^2);
for i=1:length(t)
    plot1=subplot(2,1,1);
    p1_circle=viscircles(p1',0.1);
    p2_circle=viscircles(p2(:,i)',0.1);
    p3_circle=viscircles(p3(:,i)',0.1);
    p4_circle=viscircles(p4',0.1);
    link_a=line([p1(1) p2(1,i)],[p1(2) p2(2,i)]);
    link_b=line([p2(1,i) p3(1,i)],[p2(2,i) p3(2,i)]);
    link_c=line([p3(1,i) p4(1)],[p3(2,i) p4(2)]);
    axis(plot1,'equal');
    xlim([-3 7]);
    ylim([-3 7]);
    pause(0.005);
    if(i<length(t))
        delete(p1_circle);
        delete(p2_circle);
        delete(p3_circle);
        delete(p4_circle);
        delete(link_a);
        delete(link_b);
        delete(link_c);
        plot2=subplot(2,1,2);
        plot(plot2,t(1:i),p3_v(1:i));
        title(plot2,'Velocity-Time Graph');
        xlabel(plot2,'time (s)');
        ylabel(plot2,'velocity (m/s)');
        axis([0 10 0 10]);
        grid on;
    end
end