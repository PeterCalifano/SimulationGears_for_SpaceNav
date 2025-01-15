close all
clear
clc


% Load gyros data
addpath('Gyro_Samples/')

gyroDataStruct = load('Gyro_Samples/gyro1_1kHz.mat');
gyroDataFields = fieldnames(gyroDataStruct);
gyroDataMat = table2array(gyroDataStruct.(gyroDataFields{1}));

%% Plot signals
timeStamps = gyroDataMat(:, 1)/1000; % [ms]
deltaTimes = gyroDataMat(:, 2)/1000; % [ms]
accData  = gyroDataMat(:, 3:5); % [m/s^2]
rateData = gyroDataMat(:, 6:8); % [deg/s]

gyrosReadings_Timeseries = timeseries(rateData, timeStamps);

%% SIMULATION MODEL

i_dTrueAngRate          = zeros(3,1); % (3,1) double

i_dScaleFactor          = ones(3, 1); % (3,1) double
i_dOrthogErrMatrix      = eye(3);     % (3,3) double
i_dConfigMat_IMUfromSCB = eye(3);     % (3,3) double
i_dSigmaRRW             = 0.001;      % (1,1) double
i_dSigmaARW             = 1E-5;       % (1,1) double
i_dPrevBiasValue        = zeros(3,1); % (3,1) double



dMaxReadingValue = 300;
dSamplingTime = 0.1; % [s]

% MPU6050
QuantizationInterval = [0.007939946551317;   0.001502340283998;   0.001835820374978];


% bias instability
biasInstability = [ 0.710927161564546;   0.866650378345066;   0.329514115496994 ] * 1.0e-03;

% rate random walk 
% sigmaRRW = [0.006926103644810,   0.005080362963961,   0.003964003501987];
% sigmaRRW = [0.002670262869543, 0.003588578676682, 0.001758297528434];

