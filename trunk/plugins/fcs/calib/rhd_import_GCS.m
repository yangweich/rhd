%% GROUND CONTROL STATION
% Do the calculations as ground control
clear all;
close all;

% CALIBRATION METODE
% Calibrate X loadcell
% Get data with no load in positiv x direction
file = fopen('rhdlog_noload.txt');
C1 = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ','Delimiter',' ','HeaderLines',20);
fclose(file);

% timevector
t_calib = cell2mat(C1(:,1))';
offset = t_calib(1);
for i = 1:length(t_calib)
    t_calib(i) = t_calib(i)-offset;
end

dataX = cell2mat(C1(:,4));
%dataX = dataX./100;

% Mean X value
X_mean = mean(dataX);
Xoffset = X_mean;


dataY = cell2mat(C1(:,3));
%dataY = dataY./100;

% Mean Y value
Y_mean = mean(dataY);
Yoffset = Y_mean;

dataZ = cell2mat(C1(:,5));
%dataZ = dataZ./100;

% Mean Z value
Z_mean = mean(dataZ);
Zoffset = Z_mean;

figure;
subplot(3,1,1);
hold on;
plot(t_calib,dataX,'Color',[153/255 0 0],'LineWidth',2);
plot([0 t_calib(end)],[Xoffset Xoffset],'Color',[153/255 204/255 51/255],'LineWidth',2);
ylabel('Load cell data Raw','FontName','Arial');
xlabel('Time [s]','FontName','Arial');
xlim([0 1]);
%ylim([-5985 -5995]);
legend('X','Average','Location','NorthEast');
title('Raw Data From FCS load cells with no load','FontSize',14,'FontName','Arial');
grid on;
hold off;

subplot(3,1,2);
hold on;
plot(t_calib,dataY,'Color',[51/255 102/255 203/255],'LineWidth',2);
plot([0 t_calib(end)],[Yoffset Yoffset],'Color',[153/255 204/255 51/255],'LineWidth',2);
grid on;
ylabel('Load cell data Raw','FontName','Arial');
xlabel('Time [s]','FontName','Arial');
xlim([0 1]);
legend('Y','Average','Location','NorthEast');
hold off;

subplot(3,1,3);
hold on;
plot(t_calib,dataZ,'Color','m','LineWidth',2);
plot([0 t_calib(end)],[Zoffset Zoffset],'Color',[153/255 204/255 51/255],'LineWidth',2);
grid on;
ylabel('Load cell data Raw','FontName','Arial');
xlabel('Time [s]','FontName','Arial');
xlim([0 1]);
legend('Z','Average','Location','NorthEast');
grid on;
set(gcf,'paperunits','centimeters','Paperposition',[0 0 15 15]);
saveas(gcf,'fcs_calib.eps','psc2');
hold off;


%% Get data with load in positive x direction
load = 500; % [gram]
file = fopen('rhdlog_xload.txt');
C2 = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ','Delimiter',' ','HeaderLines',20);
fclose(file);

dataXload = cell2mat(C2(:,4));
X_Load_mean = mean(dataXload);

Kx = load/(X_Load_mean-Xoffset);

%%
% Calibration Y loadcell

% Get data with load in positive x direction
load = 500; % [gram]
file = fopen('rhdlog_yload.txt');
C3 = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ','Delimiter',' ','HeaderLines',20);
fclose(file);

dataYload = cell2mat(C3(:,3));
Y_Load_mean = mean(dataYload);

Ky = load/(Y_Load_mean-Yoffset);

%%
% Calibration Z loadcell

% Get data with load in positive x direction
load = 1000; % [gram]
file = fopen('rhdlog_zload.txt');
C4 = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ','Delimiter',' ','HeaderLines',20);
fclose(file);

dataZload = cell2mat(C4(:,5));
Z_Load_mean = mean(dataZload);

Kz = load/(Z_Load_mean-Zoffset);

%% LOAD data and run calibration filter
%clear all;
close all;

% Calibration constants
%Xoffset = 0.161557079467527;
%Kx  = -4.271835679197036e+03;
%Yoffset = -6.665272445220089e+02;
%Ky = 20.245607413860206;
%Xoffset = -439.2251;
%Kx  = 0.5;
%Yoffset = -666;
%Ky = -1;


file = fopen('rhdlog_0phi54theta600g.txt');
C = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataX = cell2mat(C(:,4)');

for i = 1:length(dataX)
    dataXcalib(i) = Kx*(dataX(i)-Xoffset);
end

dataY = cell2mat(C(:,3)');

for i = 1:length(dataY)
    dataYcalib(i) = Ky*(dataY(i)-Yoffset);
end

dataZ = cell2mat(C(:,5)');

for i = 1:length(dataZ)
    dataZcalib(i) = Kz*(dataZ(i)-Zoffset);
end

figure;
subplot(3,1,1);
hold on;
%plot(t,dataX,'LineWidth',1,'Color','red');
%plot(t,dataY,'LineWidth',1,'Color','blue');
plot(t,dataXcalib,'Color',[153/255 0 0],'LineWidth',2);
plot([0 length(dataXcalib)],[mean(dataXcalib) mean(dataXcalib)],'Color',[153/255 204/255 51/255],'LineWidth',2);
title('Input data for FCS calibrated, 600g load, \phi=0 deg., \theta=45 deg.','FontName','Arial','FontSize',14);
legend('X','Average','Location','NorthEast');
grid on;
ylabel('Load cell [g]','FontName','Arial');
xlim([0 70]);
hold off;

subplot(3,1,2);
hold on;
plot(t,dataYcalib,'Color',[51/255 102/255 204/255],'LineWidth',2);
plot([0 length(dataYcalib)],[mean(dataYcalib) mean(dataYcalib)],'Color',[153/255 204/255 51/255],'LineWidth',2);
xlabel('Time [s]','FontName','Arial');
ylabel('Load cell [g]','FontName','Arial');
xlim([0 70]);
%ylim([-500 1000]);
legend('Y','Average','Location','NorthEast');
grid on;
hold off;

subplot(3,1,3);
hold on;
plot(t,dataZcalib,'Color','m','LineWidth',2);
plot([0 length(dataZcalib)],[mean(dataZcalib) mean(dataZcalib)],'Color',[153/255 204/255 51/255],'LineWidth',2);
legend('Z','Average','Location','NorthEast');
grid on;
ylabel('Load cell [g]','FontName','Arial');
xlim([0 70]);
hold off;

set(gcf,'paperunits','centimeters','Paperposition',[0 0 15 15]);
saveas(gcf,'calib_result_compare.eps','psc2')
hold off;



%%
hold on;
[ax1,p1,p2] = plotyy(t,dataX,t,dataXcalib,'plot');
[ax11,p11,p22] = plotyy(t,dataY,t,dataYcalib,'plot');
hold off;

%% 45 deg 1 kg test
close all; 

file = fopen('rhdlog_45deg1kg.txt');
C = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataX = cell2mat(C(:,11)');

dataY = cell2mat(C(:,12)');

dataR = cell2mat(C(:,25)');
dataPhi = cell2mat(C(:,24)');

figure;
hold on;
plot(t,dataX,'red');
plot(t,dataY,'blue');
xlabel('Time [s]');
ylabel('Data [g]');
xlim([0 70]);
ylim([710 780]);
title('Input data for GCS, 45 deg 1kg');
legend('X','Y');
grid on;
hold off;

close all;
figure;
hold on;
ax1 = gca;

plot(t,dataR,'Color','b');
ax2 = axes('Position',get(ax1,'Position'),...
       'XAxisLocation','top',...
       'YAxisLocation','right',...
       'Color','none',...
       'XColor','k','YColor','r');
linkaxes([ax1 ax2],'x');
hold on;
plot(t,dataPhi,'Parent',ax2,'Color','r');
set(ax1,'YColor','b');
ylabel(ax1,'r [g]');
ylabel(ax2,'\phi [deg]');
ylim(ax1,[1040 1060]);
ylim(ax2,[0 90]);
xlim([0 72]);
grid(ax1, 'on');
%grid(ax2,'off');
title('Calculated data for 45deg 1kg test');
hold off;

%% 45 deg Theta 1kg
close all;

file = fopen('rhdlog_45degTheta1kg.txt');
C = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataR = cell2mat(C(:,25)');

figure;
hold on;
plot(t,dataR,'b');
plot([0 t(end)],[1000*cosd(45) 1000*cosd(45)],'r');
legend('Messured','Exspected');
ylim([690 710]);
xlim([0 60]);
grid on;
hold off;

%% 45 deg theta 1kg with cable

close all;

file = fopen('rhdlog_45degTheta1kg.txt');
C = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);
file = fopen('rhdlog_45degTheta1kgCable.txt');
C2 = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

% timevector
t2 = cell2mat(C2(:,1))';
offset = t2(1);
for i = 1:length(t2)
    t2(i) = t2(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataR = cell2mat(C(:,25)');
dataRCable = cell2mat(C2(:,25)');

figure;
hold on;
plot(t,dataR,'b');
plot(t2,dataRCable,'g');
plot([0 t(end)],[1000*cosd(45) 1000*cosd(45)],'r');
legend('Direct','Cable','Exspected');
ylim([690 740]);
xlim([0 50]);
grid on;
hold off;

%% 0 deg Theta 1kg Cable

close all;

file = fopen('rhdlog_0degTheta1kgCable.txt');
C = textscan(file, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',' ','HeaderLines',25);
fclose(file);

% timevector
t = cell2mat(C(:,1))';
offset = t(1);
for i = 1:length(t)
    t(i) = t(i)-offset;
end

Ts = t(:,2)-t(:,1);
Fs = 1/Ts;

dataR = cell2mat(C(:,25)');

figure;
hold on;
plot(t,dataR,'b');
plot([0 t(end)],[1000 1000],'r');
legend('Messured','Exspected');
ylim([990 1030]);
xlim([0 40]);
grid on;
hold off;