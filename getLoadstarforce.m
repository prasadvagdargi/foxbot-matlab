function [ force ] = getLoadstarforce()
% Loadstar Sensor interface with MATLAB to retrieve forces in
% Newtons(probably)

instrreset;			% reset any open ports 
s1 = serial('/dev/ttyUSB0','BaudRate',9600,'DataBits',8,'Parity','None','FlowControl','None','StopBits',1);
fopen(s1);
% Send 'WC<cr>' or 'o0w0'<cr> repeatedly to read the loads

fprintf(s1,'%s', 'O0W0');
fprintf(s1,'%s', char(13));
format compact
load(10)=[0];
for i = 1:10
    temp=fgets(s1);
    load(i)=str2double(temp);
    %disp(temp);
end
force=mean(load)*4.44822/1000;

% stop streaming
fprintf(s1,'%s', char(13));
fprintf(s1,'%s', char(13));
fclose(s1);
instrreset;			% reset any open ports 

end

