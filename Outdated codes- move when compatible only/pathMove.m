clear;clc; close all;
% find p1,p2,p3,p4
p1=[466.06, -4.43, 214.42]';
p2=[466.06, 49.76, 214.42]';
p3=[416.838, 49.76, 214.42]';
p4=[416.838, -4.43, 214.42]';
%%
sub=rossubscriber('/ft_sensor/netft_data');

%%
% find z_mean
centroid=mean([p1,p2,p3,p4]')';
zmean=centroid(3);

%find zmax,zmin
zmax=zmean+20;%assuming units are in mm
zmin=zmean-2;


[l,indx1]=min([norm(p2(1:2)-p1(1:2)),norm(p3(1:2)-p4(1:2))]);
[b,indx2]=min([norm(p4(1:2)-p1(1:2)),norm(p3(1:2)-p2(1:2))]);

ldir=p2(1:2)-p1(1:2);

ldir=ldir/norm(ldir);

bdir=p4(1:2)-p1(1:2);

bdir=bdir/norm(bdir);

result=[];


moveFoxbotCartesianAbs(p1);
% add plot function
num_intervals=10;

x=0:l/num_intervals:l;
y=0:b/num_intervals:b;
[X,Y]=meshgrid(x,y);
Z=0*X;
figure
axis equal;
h1=surf(X,Y,Z);
shading interp
colormap hot;
colorbar
xlabel('X Axis');
ylabel('Y Axis');
title('Spatial stiffness mapping');
%%
for j=1:num_intervals+1
    for i=1:num_intervals+1
        
         p=p1(1:2)+i*l/(num_intervals+1)*ldir+j*b/(num_intervals+1)*bdir;
    
        moveFoxbotCartesianAbs([p(1) p(2) zmax]);
        
         moveFoxbotCartesianAbs([p(1) p(2) zmin]);
         
        force=getforce;
        %sense stiffness=force/(zmin-zmean)
    stiffness=abs(force)/(zmean-zmin);
    Z(j,i)=stiffness;
    
    %save result{i}=[px,py,stiffness]
    result{i}=[x(i),y(j),stiffness];
    
    %Update plot with new stiffness
    
    h1.CData=Z;
    axis equal;
    moveFoxbotCartesianAbs([p(1) p(2) zmax]);
    %Loop ends
    end
end
save expt_results_3.mat result p1 p2 p3 p4 zmin zmax