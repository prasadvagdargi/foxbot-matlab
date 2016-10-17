function[GPRsave,ysmask,ymu,xNext,idxPick]= GPR_prior(x,y,xs,Gprior,GPRsave,jj,ysmask)
%-------------------------------------%
%jj:iteration number
%x:probed point  nx3
%y: corresponding stiffness values  nx1
%xs:3D prediction grid mx3
%res:Expected Information   
%xNext:2D next point
%-------------------------------------%
%search coefficients
coeffD=0;coeffP=0;np=0.99^jj;
%GP parameters
covfunc = @covSEiso; likfunc = @likGauss;
sn = 0; ell =2; sf = sqrt(1);
hyp.lik = log(sn); hyp.cov = log([ell; sf]);
if jj>2
[xfilt,yfilt]=consolidator(x,y,'max',1);%noise
end
[ymu, ys2,~,~]= gp(hyp, @infExact, [], covfunc, likfunc, xfilt, yfilt, xs);
ymu(ymu<0)=0;
yEI=max(y);
res = EI(yEI,ymu,ys2).*ysmask;%res= UCB(ymu,ys2,0.1);
res=res./max(res);
%Add distance penas)lty
xLast=x(end,1:3);
dist=ipdm(xLast,xs);
%Normalize distances
dist=dist./max(dist);
dist=1-dist;%give higher weightage to close points
res=res+coeffD*dist'+np*coeffP*Gprior;
res=res./max(res);
[~,idxPick]=max(res); %find max info point
xNext=xs(idxPick,:);
% See if next point is too close
while(abs(xLast-xNext)<0.75*ell)
    data.X=xs;
    [idxClose,~]= rangesearch(data.X,xNext,0.75*ell);
    idxSetZero=[idxClose{1,1},idxPick];
    ysmask(idxSetZero)=0;
    res = EI(yEI,ymu,ys2).*ysmask;%res= UCB(ymu,ys2,0.1);
    res=res./max(res);
    res=res+coeffD*dist'+np*coeffP*Gprior;
    res=res./max(res);
    [~,idxPick]=max(res); %find max info point
    xNext=xs(idxPick,:);    
end
xNext=xNext(1:2);
%save
GPRsave.ymusave(jj,:)=ymu;
GPRsave.ys2save(jj,:)=ys2;
GPRsave.EIsave(jj,:)=res;
end