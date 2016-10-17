function [Xreg,Er,t ] = cmu_est_fast( data,force, c,cad_points,cad_normal,Xreg,normal_sensed,varargin)
%data is of size n*x, force is of size n*1, c is of size n*1
Treg=regParamsToTransformationMatrix(Xreg);

if isempty(varargin)==1
    [TR,TT,Er,t]=cmu_icp(cad_points',computeTransformedPoints( data,Xreg),cad_normal',force,c,computeTransformedPoints( normal_sensed,[0,0,0,Xreg(4:6)]));
else
    [TR,TT,Er,t]=cmu_icp(cad_points',computeTransformedPoints( data,Xreg),cad_normal',force,c,computeTransformedPoints( normal_sensed,[0,0,0,Xreg(4:6)]),varargin{1});
end

Tt=eye(4);
Tt(1:3,1:3)=TR;
Tt(1:3,4)=TT;

Xreg=transformationMatrixToRegParams(Tt*Treg);



end

