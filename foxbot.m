%% Creates a Foxbot class with movement commands
% Measurements in mm and degrees for cartesian and joint space
% respectively.
classdef foxbot
    
    properties(SetAccess = private, GetAccess = private)
        jogCClient
        jogCRequest
        
        absJClient
        absJRequest
        
        absCClient
        absCRequest
        absCResponse
        
        jogJClient
        jogJRequest
        jogJResponse
        
        stopClient
        stopRequest
        
        pauseTime=0.01;
    end
    
    methods
        
        function obj=foxbot()
            % Create all the servers and request messages
            obj.jogCClient = rossvcclient('/foxbot/robot_JogCartesian');
            obj.jogCRequest = rosmessage(obj.jogCClient);
            
            obj.absJClient = rossvcclient('/foxbot/robot_SetJoints');
            obj.absJRequest = rosmessage(obj.absJClient);
            
            obj.absCClient = rossvcclient('/foxbot/robot_GetCartesian');
            obj.absCRequest = rosmessage(obj.absCClient);
            
            obj.jogJClient = rossvcclient('/foxbot/robot_GetJoints');
            obj.jogJRequest = rosmessage(obj.jogJClient);
            
            obj.stopClient=rossvcclient('/foxbot/robot_Stop');
            obj.stopRequest=rosmessage(obj.stopClient);
        end
        
        function obj=setPauseTime(obj,t)
            % Override the set pause time
            obj.pauseTime(t);
        end
        
        function [resp]=getCartesian(obj)
           resp=call(obj.absCClient,obj.absCRequest); 
        end
        
        function [resp]=getJoints(obj)
           resp=call(obj.jogJClient,obj.jogJRequest); 
        end
        
        function [resp]=moveCartesianDiff(obj,diff)
            % Move the foxbot in cartesian differential.
            obj.jogCRequest.X = diff(1);
            obj.jogCRequest.Y = diff(2);
            obj.jogCRequest.Z = diff(3);
            resp=call(obj.jogCClient, obj.jogCRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=moveJointsDiff(obj,diff)
            % Moves the foxbot to given joint angles absolute. Send difference values to
            % this function.
            obj.jogJResponse = call(obj.jogJClient,obj.jogJRequest);
            
            pos(1)=diff(1)+obj.jogJResponse.J1;
            pos(2)=diff(2)+obj.jogJResponse.J2;
            pos(3)=diff(3)+obj.jogJResponse.J3;
            pos(4)=diff(4)+obj.jogJResponse.J4;
            pos(5)=diff(5)+obj.jogJResponse.J5;
            pos(6)=diff(6)+obj.jogJResponse.J6;
            
            % Joints works in the opposite sense of Cartesian. It requires
            % absolute values therefore the current position was retrieved
            % and then the absolute calculated and sent.
            obj.absJRequest.J1= pos(1);
            obj.absJRequest.J2= pos(2);
            obj.absJRequest.J3= pos(3);
            obj.absJRequest.J4= pos(4);
            obj.absJRequest.J5= pos(5);
            obj.absJRequest.J6= pos(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=goHome(obj)
            % USE WITH CAUTION.
            % Moves the foxbot to home position
            goal=[0 0 90 0 90 0];
            obj.absJRequest.J1= goal(1);
            obj.absJRequest.J2= goal(2);
            obj.absJRequest.J3= goal(3);
            obj.absJRequest.J4= goal(4);
            obj.absJRequest.J5= goal(5);
            obj.absJRequest.J6= goal(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        
        function [resp]=moveJointsAbs(obj,goal)
            % USE WITH CAUTION.
            % Moves the foxbot to given joint angles absolute. Send absolute values to
            % this function.
            obj.absJRequest.J1= goal(1);
            obj.absJRequest.J2= goal(2);
            obj.absJRequest.J3= goal(3);
            obj.absJRequest.J4= goal(4);
            obj.absJRequest.J5= goal(5);
            obj.absJRequest.J6= goal(6);
            
            resp=call(obj.absJClient, obj.absJRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=moveCartesianAbs(obj,goal)
            % USE WITH CAUTION.
            
            % Move the foxbot in cartesian absolute. Cartesian can move
            % only in jog mode, therefore the current position is
            % subtracted to find the difference to jog and then the robot
            % is moved.
            
            % Get current position by calling the client
            obj.absCResponse = call(obj.absCClient, obj.absCRequest);
            
            % Calculate the difference and move ahead using usual jog
            diff.X = goal(1)-obj.absCResponse.X;
            diff.Y = goal(2)-obj.absCResponse.Y;
            diff.Z = goal(3)-obj.absCResponse.Z;
            
            obj.jogCRequest.X = diff.X;
            obj.jogCRequest.Y = diff.Y;
            obj.jogCRequest.Z = diff.Z;
            resp=call(obj.jogCClient, obj.jogCRequest);
            pause(obj.pauseTime);
        end
        
        function [resp]=stopRobot(obj)
           resp=call(obj.stopClient,obj.stopRequest);
           pause(obj.pauseTime);
        end
    end
end