clear;clc; close all;
% find p1,p2,p3,p4
p1=[466.06, -4.43, 214.42]';
p2=[466.06, 49.76, 214.42]';
p3=[416.838, 49.76, 214.42]';
p4=[416.838, -4.43, 214.42]';
%%
% find z_mean
centroid=mean([p1,p2,p3,p4]')';
zmean=centroid(3);

%find zmax,zmin
zmax=zmean+20;%assuming units are in mm
zmin=zmean-5;

% Loop starts
[l,indx1]=min([norm(p2(1:2)-p1(1:2)),norm(p3(1:2)-p4(1:2))]);
[b,indx2]=min([norm(p4(1:2)-p1(1:2)),norm(p3(1:2)-p2(1:2))]);

ldir=p2(1:2)-p1(1:2);

ldir=ldir/norm(ldir);

bdir=p4(1:2)-p1(1:2);

bdir=bdir/norm(bdir);

result=[];

% add plot function
num_intervals=15;

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
for ii=1:100
    % find P
    lambda1=randi([1,num_intervals]);
    lambda2=randi([1,num_intervals]);
    p=p1(1:2)+lambda1*l/num_intervals*ldir+lambda2*b/num_intervals*bdir;
    
    
    flag1=moveFoxbotCartesianAbs([p(1) p(2) zmax]);
    if flag1==1
        flag2=moveFoxbotCartesianAbs([p(1) p(2) zmin]);
    end
    %sense force
    force=0;
    if flag2==1
        force=getforce;
    end
    %sense stiffness=force/(zmin-zmean)
    stiffness=abs(force)/(zmean-zmin);
    Z(lambda2,lambda1)=stiffness;
    
    %save result{i}=[px,py,stiffness]
    result{ii}=[lambda1,lambda2,stiffness];
    
    
    %Update plot with new stiffness
    
    h1.CData=Z;
    axis equal;
    moveFoxbotCartesianAbs([p(1) p(2) zmax]);
    %Loop ends
end


save expt_results.mat result p1 p2 p3 p4 zmin zmax
%%
% after you finish reading, send a <cr> to
