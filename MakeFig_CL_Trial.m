function [] = MakeFig_CL_Trial(rootdir)
%% MakeFig_CL_Trial: plot CL Kinefly trial
%   INPUTS:
%       root  	: directory where .mat files are located
%   OUTPUTS:
%       -
%---------------------------------------------------------------------------------------------------------------------------------
% EXAMPLE INPUT %
rootdir = 'F:\MOVIE\Kinefly_Demo\mat\';
%---------------------------------------------------------------------------------------------------------------------------------
%% Setup Directories %%
%---------------------------------------------------------------------------------------------------------------------------------
% Select files
[FILES, PATH] = uigetfile({'*.mat', 'DAQ-files'}, 'Select files', rootdir, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one character array >> store in cell

% [~,I,N,U] = GetFileData(FILES);

%% Get Data %%
%---------------------------------------------------------------------------------------------------------------------------------
fly = [];
FlyState = [];
AI = [];
fly.Fc = 20;
span = 1:1:900;
for kk = 1:length(FILES) % all trials
    filename = fullfile(PATH,FILES{kk}); % full file name
    load(filename,'FlyState','AI','VidTime') % load in fly kinematics & arena voltages
    
    vid.time            = VidTime;
    fly.time            = FlyState{:,1};
    fly.Ts              = mean(diff(fly.time));
    fly.Fs              = 1/fly.Ts; 
	[b,a]               = butter(2,fly.Fc/(fly.Fs/2),'low'); % 2nd-order low-pass butterworth filter
    fly.head.pos        = filtfilt(b,a,FlyState{:,2});
  	fly.wing.pos        = filtfilt(b,a,filtfilt(b,a,FlyState{:,3}) - filtfilt(b,a,FlyState{:,4}));
	fly.head.vel        = [diff(fly.head.pos)./fly.Ts ; 0];
	fly.wing.vel        = [diff(fly.wing.pos)./fly.Ts ; 0];
    pat.time            = AI{:,1};
    pat.Ts              = mean(diff(pat.time));
    pat.Fs              = 1/pat.Ts;
 	pat.xpos            = 3.75*round((96/5)*AI{:,2});
    
	fly.wing.pos    	= interp1(fly.time, fly.wing.pos , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.pos        = interp1(fly.time, fly.head.pos , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
	fly.wing.vel    	= interp1(fly.time, fly.wing.vel , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.vel        = interp1(fly.time, fly.head.vel , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
 	pat.xpos         	= interp1(pat.time, pat.xpos     , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	
    fly.time            = fly.time      (span);
    fly.head.pos        = fly.head.pos  (span);
 	fly.wing.pos        = fly.wing.pos  (span);
	fly.head.vel        = fly.head.vel  (span);
 	fly.wing.vel        = fly.wing.vel  (span);
 	pat.xpos            = pat.xpos      (span);
end
%% Plot head position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (1) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Head Position')
set(gcf,'Position',[0 0 1*900 1*400])
movegui(gcf,'center')
xlim([0 10])
ylim(25*[-1 1])
yticks(-30:5:30)
ylabel(['Angle (' char(176) ')'],'FontSize',13,'fontweight','bold')
xlabel('Time (s)','FontSize',12,'fontweight','bold')
plot(fly.time,rad2deg(fly.head.pos-mean(fly.head.pos)),'b','LineWidth',1)
plot(fly.time,rad2deg(fly.wing.pos-mean(fly.wing.pos)),'r','LineWidth',1)
plot(fly.time,pat.xpos - mean(pat.xpos),'g','LineWidth',1)
legend({'Head','\Delta WBA'},'FontSize',12)
end