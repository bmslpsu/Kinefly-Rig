function [] = Analyze_Template(rootdir)
%% Analyze_Template: Reads in all raw trials, transforms data, and saves in organized structure
%   INPUTS:
%       root  	: directory where .mat files are located
%   OUTPUTS:
%       -
%---------------------------------------------------------------------------------------------------------------------------------
% EXAMPLE INPUT %
% rootdir = 'H:\EXPERIMENTS\Experiment_Wing_CL\mat';
%---------------------------------------------------------------------------------------------------------------------------------
%% Setup Directories %%
%---------------------------------------------------------------------------------------------------------------------------------
% Select files
[FILES, PATH] = uigetfile({'*.mat', 'DAQ-files'}, 'Select files', rootdir, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one character array >> store in cell

[~,I,N,U] = GetFileData(FILES,'Fly','Trial','HGain','WGain');

%% Get Data %%
%---------------------------------------------------------------------------------------------------------------------------------
TIME        = cell(N{1,1},1);
HEAD.Pos    = cell(N{1,1},1);
WING.Pos    = cell(N{1,1},1);
PAT.XPos  	= cell(N{1,1},1);
PAT.YPos  	= cell(N{1,1},1);
for kk = 1:N{1,1} % fly
    for jj = 1:N{1,4} % wing gain
        for ii = 1:N{1,3} % head gain
            TIME{kk,1}{jj,1}{ii,1}      = [];
            HEAD.Pos{kk,1}{jj,1}{ii,1}  = [];
            WING.Pos{kk,1}{jj,1}{ii,1}  = [];
            PAT.XPos{kk,1}{jj,1}{ii,1}  = [];
            PAT.YPos{kk,1}{jj,1}{ii,1}  = [];
        end
    end
end
fly = [];
FlyState = [];
AI = [];
fly.Fc = 20;
span = 20:1:2100;
for kk = 1:N{1,end} % all trials
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
 	pat.xpos            = AI{:,2};
 	pat.ypos            = AI{:,3};
    
	fly.wing.pos    	= interp1(fly.time, fly.wing.pos , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.pos        = interp1(fly.time, fly.head.pos , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
	fly.wing.vel    	= interp1(fly.time, fly.wing.vel , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	fly.head.vel        = interp1(fly.time, fly.head.vel , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
 	pat.xpos         	= interp1(pat.time, pat.xpos     , vid.time, 'nearest'); % interpolate pattern x-pos to match fly
 	pat.ypos         	= interp1(pat.time, pat.ypos  	 , vid.time, 'nearest'); % interpolate pattern y-pos to match fly
 	
    fly.time            = fly.time      (span);
    fly.head.pos        = fly.head.pos  (span);
 	fly.wing.pos        = fly.wing.pos  (span);
	fly.head.vel        = fly.head.vel  (span);
 	fly.wing.vel        = fly.wing.vel  (span);
 	pat.xpos            = pat.xpos      (span);
 	pat.ypos            = pat.ypos      (span);

    TIME                {I{kk,1}}{I{kk,4},1}{I{kk,3},1}(:,end+1) = fly.time;
    HEAD.Pos            {I{kk,1}}{I{kk,4},1}{I{kk,3},1}(:,end+1) = fly.head.pos;
  	WING.Pos            {I{kk,1}}{I{kk,4},1}{I{kk,3},1}(:,end+1) = fly.wing.pos;
  	PAT.XPos            {I{kk,1}}{I{kk,4},1}{I{kk,3},1}(:,end+1) = pat.xpos ;
  	PAT.YPos            {I{kk,1}}{I{kk,4},1}{I{kk,3},1}(:,end+1) = pat.ypos;
end
%% Plot head position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (1) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Head Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
for kk = 1:N{1,1}
    pp = 1;
    for jj = 1:N{1,4}
        for ii = 1:N{1,3}
            subplot(N{1,3},N{1,4},pp) ; hold on
                xlim([0 20])
                ylim(20*[-1 1])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel({['Head Gain = ' num2str(U{1,3}{1}(ii))],['Head Position(' char(176) ')']},...
                        'FontSize',13,'fontweight','bold')
                else
                    yticks(0)
                    yticklabels('')  
                end
                if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel('Time (s)','FontSize',12,'fontweight','bold')
                else
                    xticks(0)
                    xticklabels('')
                end

                time = TIME{kk}{jj}{ii};
                pos  = rad2deg(HEAD.Pos{kk}{jj}{ii});
                plot(time,pos)

            pp = pp + 1;
        end
    end
end

%% Plot wing position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (2) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Wing Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
for kk = 1:N{1,1}
    pp = 1;
    for jj = 1:N{1,4}
        for ii = 1:N{1,3}
            subplot(N{1,3},N{1,4},pp) ; hold on
                xlim([0 20])
                ylim(120*[-1 1])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel({['Head Gain = ' num2str(U{1,3}{1}(ii))],['Wing Position(' char(176) ')']},...
                        'FontSize',13,'fontweight','bold')
                else
                    yticks(0)
                    yticklabels('')  
                end
                if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel('Time (s)','FontSize',12,'fontweight','bold')
                else
                    xticks(0)
                    xticklabels('')
                end

                time = TIME{kk}{jj}{ii};
                pos  = rad2deg(WING.Pos{kk}{jj}{ii});
                plot(time,pos)

            pp = pp + 1;
        end
    end
end