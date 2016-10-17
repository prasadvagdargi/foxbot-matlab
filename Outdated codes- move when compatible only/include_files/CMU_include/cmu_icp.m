function [TR, TT, residue, t] = cmu_icp(q,p,q_normal,force,c,p_normal,varargin)
%q cad model, p is data
% Perform the Iterative Closest Point algorithm on three dimensional point
% clouds.
%
% [TR, TT] = icp(q,p)   returns the rotation matrix TR and translation
% vector TT that minimizes the distances from (TR * p + TT) to q.
% p is a 3xm matrix and q is a 3xn matrix.
%
% [TR, TT] = icp(q,p,k)   forces the algorithm to make k iterations
% exactly. The default is 10 iterations.
%
% [TR, TT, ER] = icp(q,p,k)   also returns the RMS of errors for k
% iterations in a (k+1)x1 vector. ER(0) is the initial error.
%
% [TR, TT, ER, t] = icp(q,p,k)   also returns the calculation times per
% iteration in a (k+1)x1 vector. t(0) is the time consumed for preprocessing.

% Extrapolation
%      the iteration direction will be evaluated
%       and extrapolated if possible using the method outlined by
%       Besl and McKay 1992.

% Martin Kjer and Jakob Wilm, Technical University of Denmark, 2011

% Use the inputParser class to validate input arguments.
inp = inputParser;

inp.addRequired('q', @(x)isreal(x) && size(x,1) == 3);
inp.addRequired('p', @(x)isreal(x) && size(x,1) == 3);

inp.addOptional('iter', 100, @(x)x > 0 && x < 10^5);

inp.parse(q,p,varargin{:});
arg = inp.Results;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actual implementation

% Allocate vector for RMS of errors in every iteration.
t = zeros(arg.iter+1,1);

% Start timer
tic;

Np = size(p,2);

% Transformed data point cloud
pt = p;
p_un=pt+bsxfun(@times, p_normal,bsxfun(@rdivide,force,c)');
p_un0=p_un;

% Allocate vector for RMS of errors in every iteration.
ER = zeros(arg.iter+1,1);

% Initialize temporary transform vector and matrix.
T = zeros(3,1);
R = eye(3,3);

% Initialize total transform vector(s) and rotation matric(es).
TT = zeros(3,1, arg.iter+1);
TR = repmat(eye(3,3), [1,1, arg.iter+1]);


kdOBJ = KDTreeSearcher(transpose(q));

% For extrapolation
% Initialize total transform vector (quaternion ; translation vec.)
qq = [ones(1,arg.iter+1);zeros(6,arg.iter+1)];
% Allocate vector for direction change and change angle.
dq = zeros(7,arg.iter+1);
theta = zeros(1,arg.iter+1);


t(1) = toc;

% Go into main iteration loop
for k=1:arg.iter
    
    % Do matching
    [match, mindist] = match_kDtree(kdOBJ,p_un);
    p_idx = true(1, Np);
    q_idx = match;
    
    if k == 1
        ER(k) = sqrt(sum(mindist.^2)/length(mindist));
    end
    
    % Minimize
    def=force(p_idx)./c(p_idx);
    [R,T] = findT(q(:,q_idx)-bsxfun(@times,q_normal(:,q_idx),def'),pt(:,p_idx));
    
    % Add to the total transformation
    TR(:,:,k+1) = R*TR(:,:,k);
    TT(:,:,k+1) = R*TT(:,:,k)+T;
    
    % Apply last transformation
    pt = TR(:,:,k+1) * p + repmat(TT(:,:,k+1), 1, Np);
    p_un = TR(:,:,k+1) * p_un0 + repmat(TT(:,:,k+1), 1, Np);
    % Root mean of objective function
    ER(k+1) = rms_error(q(:,q_idx), pt(:,p_idx));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  Extrapolation, to move quicker
    qq(:,k+1) = [rotm2quat(TR(:,:,k+1))';TT(:,:,k+1)];
    dq(:,k+1) = qq(:,k+1) - qq(:,k);
    theta(k+1) = (180/pi)*acos(dot(dq(:,k),dq(:,k+1))/(norm(dq(:,k))*norm(dq(:,k+1))));
    
    if k>2 && theta(k+1) < 10 && theta(k) < 10
        d = [ER(k+1), ER(k), ER(k-1)];
        v = [0, -norm(dq(:,k+1)), -norm(dq(:,k))-norm(dq(:,k+1))];
        vmax = 25 * norm(dq(:,k+1));
        dv = extrapolate(v,d,vmax);
        if dv ~= 0
            q_mark = qq(:,k+1) + dv * dq(:,k+1)/norm(dq(:,k+1));
            q_mark(1:4) = q_mark(1:4)/norm(q_mark(1:4));
            qq(:,k+1) = q_mark;
            TR(:,:,k+1) = quat2rotm(qq(1:4,k+1)');
            TT(:,:,k+1) = qq(5:7,k+1);
            % Reapply total transformation
            pt = TR(:,:,k+1) * p + repmat(TT(:,:,k+1), 1, Np);
            p_un = TR(:,:,k+1) * p_un0 + repmat(TT(:,:,k+1), 1, Np);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t(k+1) = toc;
    residue=ER(k+1);
    if abs(ER(k+1)-ER(k))<10^-8
        
        break;
    end
end

TR = TR(:,:,k);
TT = TT(:,:,k);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match, mindist] = match_kDtree( kdOBJ,p)

[match, mindist] = knnsearch(kdOBJ,transpose(p));
match = transpose(match);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extrapolation in quaternion space. Details are found in:
%
% Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes.
% IEEE Transactions on pattern analysis and machine intelligence, 239?256.

function [dv] = extrapolate(v,d,vmax)

p1 = polyfit(v,d,1); % linear fit
p2 = polyfit(v,d,2); % parabolic fit
v1 = -p1(2)/p1(1); % linear zero crossing
v2 = -p2(2)/(2*p2(1)); % polynomial top point

if issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1])
    dv = v2;
elseif issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
        || (v2 < 0 && issorted([0 v1 vmax]))
    dv = v1;
elseif v1 > vmax && v2 > vmax
    dv = vmax;
else
    dv = 0;
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine the RMS error between two point equally sized point clouds with
% point correspondance.
% ER = rms_error(p1,p2) where p1 and p2 are 3xn matrices.

function ER = rms_error(p1,p2)
dsq = sum(power(p1 - p2, 2),1);
ER = sqrt(mean(dsq));
end