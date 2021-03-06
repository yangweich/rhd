%% GROUND CONTROL STATION
% Do the calculations as ground control
clear all;
close all;

% CALIBRATION METODE
% Calibrate X loadcell
% Get data with no load in positiv x direction
file = fopen('rhdlog_noload.txt');
C1 = textscan(file, '%f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d','Delimiter',' ','HeaderLines',18);
fclose(file);



dataX = cell2mat(C1(:,3));
%dataX = dataX./100;

% Mean X value
X_mean = mean(dataX);
Xoffset = X_mean;

% Get data with load in positive x direction
load = 600; % [gram]
file = fopen('rhdlog_xload.txt');
C2 = textscan(file, '%f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d','Delimiter',' ','HeaderLines',18);
fclose(file);

dataXload = cell2mat(C2(:,3));
X_Load_mean = mean(dataXload);

Kx = load/(X_Load_mean-Xoffset);

%%
% Calibration Y loadcell
% Get data with no load in positiv y direction
file = fopen('rhdlog_xload.txt');
C1 = textscan(file, '%f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d','Delimiter',' ','HeaderLines',18);
fclose(file);

dataY = cell2mat(C1(:,4));
%dataY = dataY./100;

% Mean Y value
Y_mean = mean(dataY);
Yoffset = Y_mean;

% Get data with load in positive x direction
load = 500; % [gram]
file = fopen('rhdlog_test6.txt');
C2 = textscan(file, '%f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d','Delimiter',' ','HeaderLines',18);
fclose(file);

dataYload = cell2mat(C2(:,4));
Y_Load_mean = mean(dataYload);

Ky = load/(Y_Load_mean-Yoffset);

%% LOAD data and run calibration filter
clear all;
close all;

% Calibration constants
%Xoffset = 0.161557079467527;
%Kx  = -4.271835679197036e+03;
%Yoffset = -6.665272445220089e+02;
%Ky = 20.245607413860206;
Xoffset = -439.2251;
Kx  = 0.5;
%Yoffset = -666;
%Ky = -1;


file = fopen('rhdlog.txt');
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
%dataX = dataX./100;
for i = 1:length(dataX)
    dataXcalib(i) = Kx*(dataX(i)-Xoffset);
end

%dataY = cell2mat(C(:,4));
%dataY = dataY./100;
%for i = 1:length(dataY)
%    dataYcalib(i) = Ky*(dataY(i)-Yoffset);
%end

figure;
hold on;
plot(t,dataX,'red');
%plot(t,dataY,'blue');
plot(t,dataXcalib,'green');
%plot(t,dataYcalib,'yellow');
xlabel('Time [s]');
ylabel('Data');
title('Input data');
legend('X','X calib');
%legend('X','Y','X calib','Y calib');
hold off;

mean(dataXcalib)
%%
freqz(dataX);
