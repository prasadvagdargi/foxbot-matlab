function [ fvec,optoMag ] = getOptoForce( optoSub,bias )
%GETOPTOFORCE Retrieves the optoforce sensor values given the bias and the
%subscriber
%   Detailed explanation goes here
            fvec.X=(optoSub.LatestMessage.Wrench.Force.X-bias.X)/1000;
            fvec.Y=(optoSub.LatestMessage.Wrench.Force.Y-bias.Y)/1000;
            fvec.Z=(optoSub.LatestMessage.Wrench.Force.Z-bias.Z)/1000;
            optoMag=sqrt((fvec.X)^2+(fvec.Y)^2+(fvec.Z)^2);
end

