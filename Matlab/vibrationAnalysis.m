clear all 
close all
%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: B:\Uni\Thesis\Thesis_robotic_drilling\Matlab\28-10-22-16-18-55.csv
%
% Auto-generated by MATLAB on 18-Nov-2022 10:50:42

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 14);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Var1", "Var2", "Var3", "Var4", "Var5", "ax", "ay", "az", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14"];
opts.SelectedVariableNames = ["ax", "ay", "az"];
opts.VariableTypes = ["string", "string", "string", "string", "string", "double", "double", "double", "string", "string", "string", "string", "string", "string"];

% Specify file level properties
opts.ImportErrorRule = "omitrow";
opts.MissingRule = "omitrow";
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14"], "EmptyFieldRule", "auto");

% Import the data
path = "B:\Uni\Thesis\Thesis_robotic_drilling\log_data\21-10-22\";
saveFFTPath = path + "\fft\";
saveTimePath = path + "\time\";
file = "21-10-22-17-15-26";
fileExtension = ".csv";
saveFileExtension = ".png";
tbl = readtable(path + file + fileExtension, opts);

%% Convert to output type
ax = tbl.ax;
ay = tbl.ay;
az = tbl.az;

%% Clear temporary variables
clear opts tbl

%% Analysis
% Remove first 20 values because they always seem to be garbage
ax(1:20) = 0;
ay(1:20) = 0;
az(1:20) = 0;
% Calculate frequency
totalTime = length(az)*0.0136; % s
fftx = abs(fft(ax));
ffty = abs(fft(ay));
fftz = abs(fft(az));

f = (0:length(az)-1)*totalTime/length(az);
t = 1:length(az);

figure(1)
hold on
plot(t,ax);
plot(t,ay);
plot(t,az);
legend("ax","ay","az")
title("Acceleration vs time - " + file)
xlabel("time (index)");
ylabel("Acceleration (m/s^2)");
saveas(gcf, saveTimePath + file + saveFileExtension);
hold off

figure(2)
hold on 
plot(f,fftx)
plot(f,ffty)
plot(f,fftz)
legend("ax","ay","az")
title("FFT - " + file)
xlabel("Frequency (Hz)");
ylabel("Amplitude");
saveas(gcf, saveFFTPath + file + saveFileExtension);
hold off
