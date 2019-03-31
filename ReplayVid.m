function [] = ReplayVid(rootdir)
%% ReplayVid: Replays Kinefly video with tracking animations
%   INPUTS:
%       rootdir  	: directory where .mat files are located
%   OUTPUTS:
%       -
%---------------------------------------------------------------------------------------------------------------------------------
% EXAMPLE INPUT %
% clear;close all;clc
% rootdir = 'H:\EXPERIMENTS\Experiment_Wing_CL\mat';
%---------------------------------------------------------------------------------------------------------------------------------
%% Setup Directories %%
%---------------------------------------------------------------------------------------------------------------------------------
% Select files
[FILES, PATH] = uigetfile({'*.mat', 'DAQ-files'}, 'Select files', rootdir, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one character array >> store in cell
[~,~,N,~] = GetFileData(FILES,'Fly','Trial','HGain','WGain');

%% Get Data %%
%---------------------------------------------------------------------------------------------------------------------------------
fly = [];
FlyState = [];
fly.Fc = 20;
span = 20:1:2100;
for kk = 1:N{1,end} % all trials
    filename = fullfile(PATH,FILES{kk}); % full file name
    load(filename,'FlyState','Vid','VidTime') % load in fly kinematics & arena voltages
    
    vid.time            = VidTime;
    fly.time            = FlyState{:,1};
    fly.Ts              = mean(diff(fly.time));
    fly.Fs              = 1/fly.Ts; 
	[b,a]               = butter(2,fly.Fc/(fly.Fs/2),'low'); % 2nd-order low-pass butterworth filter
    fly.head.pos        = filtfilt(b,a,FlyState{:,2});
    fly.left.pos        = filtfilt(b,a,FlyState{:,3});
    fly.right.pos    	= filtfilt(b,a,FlyState{:,3});
  	fly.wing.pos        = filtfilt(b,a,fly.left.pos  - fly.right.pos );
	fly.head.vel        = [diff(fly.head.pos)./fly.Ts ; 0];
	fly.wing.vel        = [diff(fly.wing.pos)./fly.Ts ; 0];
    
	fly.left.pos    	= interp1(fly.time, fly.left.pos  , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
	fly.right.pos    	= interp1(fly.time, fly.right.pos , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
	fly.wing.pos    	= interp1(fly.time, fly.wing.pos  , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.pos        = interp1(fly.time, fly.head.pos  , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
	fly.wing.vel    	= interp1(fly.time, fly.wing.vel  , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.vel        = interp1(fly.time, fly.head.vel  , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
 	
    fly.time            = fly.time      (span);
    fly.head.pos        = fly.head.pos  (span);
	fly.left.pos        = fly.left.pos  (span);
	fly.right.pos    	= fly.right.pos (span);
 	fly.wing.pos        = fly.wing.pos  (span);
	fly.head.vel        = fly.head.vel  (span);
 	fly.wing.vel        = fly.wing.vel  (span);
    Vid             	= Vid(:,:,span);
    
    disp(FILES{kk})
    plot_head_wings(fly.left.pos,fly.right.pos,fly.head.pos,Vid,10)
    close
end