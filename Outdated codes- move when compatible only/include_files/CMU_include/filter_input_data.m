function [ def_point, stiffness, def_force, def_normal] = filter_input_data( def_points_raw,def_forces_raw,def_normals_raw,num_valid_points )

%def_points_raw is of dimension 10x3 or nx3 in general.
%def_forces_raw is of dimension 10x3 or nx3 in general.
%Fit a line and find stiffness
%Then send as output one of the deformed points, def_point (1*3) and the corresponding
%def_force (1*1) and the stiffness(1*1)

disp=[];rel_force=[];
kk=1;dmin=0;fmin=0;
if num_valid_points>=2;
    
    for ii=1:num_valid_points-1
        a=def_points_raw(ii+1,:)-def_points_raw(1,:);
        disp(kk)=sqrt(a*a');
        rel_force(kk)=def_forces_raw(ii+1)-def_forces_raw(1);
        
        if disp(kk)>=dmin && rel_force(kk)>=fmin
            
            dmin=disp(kk);fmin=rel_force(kk);
            kk=kk+1;%% if relative depth is not increasing or force is not increasing, then something is wrong with the data. Donot save those.
        else
            disp(kk)=[];rel_force(kk)=[];
        end
    end
    if isempty(disp)==1 || length(disp)<2%% if we dont find more than one valid points then just say stiffness is 0.1
        stiffness=0.1;
    else
        st=1;en=length(disp);
        fit_line = polyfit(disp(st:en),rel_force(st:en),1);
        % f=polyval(fit_line,disp);
        % h1=  plot(disp,rel_force,'o',disp,f,'-');
        % pause('on');
        % pause;
        stiffness=abs(fit_line(1));
    end
    
    
elseif num_valid_points==1
    stiffness=0.1;
    
    
end
if num_valid_points==length(def_points_raw)
    
    def_point=def_points_raw(8,:);
    def_force=def_forces_raw(8);
    def_normal=def_normals_raw(8,:);
else
    def_point=def_points_raw(num_valid_points,:);
    def_force=def_forces_raw(num_valid_points);
    def_normal=def_normals_raw(num_valid_points,:);
end
end



