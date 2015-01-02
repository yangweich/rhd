%% GROUND CONTROL STATION
% Plot GROUND CONTROL STATION INPUT RAW
clear all;
close all;

file = fopen('rhdlog20150102_142115.txt');
C = textscan(file, '%f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d','Delimiter',' ','HeaderLines',18);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataX = cell2mat(C(:,3));
%data = data./100;
dataY = cell2mat(C(:,4));

figure;
hold on;
plot(t,dataX,'red');
plot(t,dataY,'blue');
xlabel('Time [s]');
ylabel('Data');
title('Input data');
legend('X','Y');
hold off;


[hx,wx] = freqz(dataX);
[hy,wy] = freqz(dataY);

f=50;

figure; 
hold on;
plot(wx*f*(2/pi),20*log10(abs(hx)),'blue');
plot(wy*f*(2/pi),20*log10(abs(hy)),'red');
legend('X','Y');

xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');


hold off;



