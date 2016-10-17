clear;clc; close all;
% find p1,p2,p3,p4
p1=[515.94,-6.32,210.58]';
p2=[515.81,42.46,209.45]';
p3=[470.52,42.46,212.70]';
p4=[470.45,-3.35,210.95]';
pall=[p1,p2,p3,p4];
%%
% find z_mean
centroid=mean(pall,2);
zmean=centroid(3);

%find zmax,zmin
zmax=zmean+20;%assuming units are in mm
zmin=zmean-3;


[l,indx1]=min([norm(p2(1:2)-p1(1:2)),norm(p3(1:2)-p4(1:2))]);
[b,indx2]=min([norm(p4(1:2)-p1(1:2)),norm(p3(1:2)-p2(1:2))]);

ldir=p3(1:2)-p4(1:2);

ldir=ldir/norm(ldir);

bdir=p1(1:2)-p4(1:2);

bdir=bdir/norm(bdir);

result=[];


moveFoxbotCartesianAbs(p1);
% add plot function
num_intervals=100;

grid_x=p4(1):l/num_intervals:l+p4(1);
grid_y=p4(2):b/num_intervals:b+p4(2);
[X,Y]=meshgrid(grid_x,grid_y);
Z=0*X;
%%
tt=0;

xsFit = meshgrid2vec(X, Y, zmean*ones(num_intervals+1))'; %3D estimated grid
%xsFit=reshape(xsFit,size(X));
Gprior=zeros(size(xsFit(:,1)));
ysmask=ones(size(xsFit(:,1)));


x=[];
cEst=[];
init_probing_num=10;
max_probing_num=100;
GPRsave.ymusave=[];GPRsave.ys2save=[]; GPRsave.EIsave=[];
%%
for jj=1:max_probing_num
    
    if jj<=4
        xNext=pall(1:2,jj);%first palpate corner points
    elseif jj<init_probing_num+4
        xNext=p4(1:2)+randi([1,num_intervals+1])*l/(num_intervals+1)*ldir+randi([1,num_intervals+1])*b/(num_intervals+1)*bdir;%then palpate some random points
    else
        save intermediate_results_before_gpr.mat 
        
        [GPRsave,ysmask,ymu,xNext,~]=GPR_prior(x,cEst,xsFit,Gprior,GPRsave,jj,ysmask);
    end
    
    p=[xNext(1),xNext(2),zmean];
    moveFoxbotCartesianAbs([p(1) p(2) zmax]);
    
    moveFoxbotCartesianAbs([p(1) p(2) zmin]);
    force=getforce;
    %sense stiffness=force/(zmin-zmean)
    stiffness=abs(force)/(zmean-zmin);
    x=[x;p];
    
    cEst=[cEst;stiffness];
    %Update plot with new stiffness
    if jj==init_probing_num+4
        h3= color_line3( xsFit(:,1),xsFit(:,2), xsFit(:,3),ymu,'.');
    end
    
    if jj>init_probing_num+4
        set(h3,'XData', [xsFit(:,1) xsFit(:,1)],'YData', [xsFit(:,2) xsFit(:,2)],...
            'ZData',[xsFit(:,3) xsFit(:,3)] ,'CData', [ymu ymu]);
        
        axis equal;
    end
    
    moveFoxbotCartesianAbs([p(1) p(2) zmax]);
    %Loop ends
end




save expt_results_3.mat result p1 p2 p3 p4 zmin zmax
%%
% after you finish reading, send a <cr> to
