function [] = CL_HeadWing_Figure(root)
%%
clc
clear

%% CL_HeadWing_Figure:
%   INPUTS:
%       root  	: directory where .mat files are located
%   OUTPUTS:
%       -
%

 root = 'home';

% Select files
[D,I,N,U,T,FILES,PATH,~] = GetFileData(root,[],false,'Fly','Trial','HGain','WGain');

if ~iscell(U.HGain)
    U.HGain = {U.HGain};
end
if ~iscell(U.WGain)
    U.WGain = {U.WGain};
end

% newvars = zeros(size(I,1),4);
% stats = addvars(I,newvars,'PatStr','HStr','WStr','ErrStr');

% Get Data
TIME            = cell(N.Fly,1);
HEAD.Pos        = cell(N.Fly,1);
WING.Pos        = cell(N.Fly,1);
PAT.XPos        = cell(N.Fly,1);
HEAD.All.Pos    = cell(N.WGain,N.HGain);
WING.All.Pos    = cell(N.WGain,N.HGain);
PAT.All.XPos    = cell(N.WGain,N.HGain);
for kk = 1:N.Fly % fly
    for ww = 1:N.WGain % wing gain
        for hh = 1:N.HGain % head gain
            TIME    {kk,1}{ww,hh} 	= [];
            HEAD.Pos{kk,1}{ww,hh}  	= []; 
            WING.Pos{kk,1}{ww,hh}   = [];
            PAT.XPos{kk,1}{ww,hh}   = [];
        end
    end
end

fly = [];
fly.Fc = 20;
span = 20:1:2000;
for kk = 1:N.file % all trials
    filename = fullfile(PATH,FILES{kk});
    load(filename,'FlyState','AI','rawTime','VidTime') % load in fly kinematics & arena voltages
    if exist('VidTime','var')==1
        vrange = 5;
        center = 93;
        fly.time = VidTime;
    elseif exist('rawTime','var')==1
        center = 45;
        vrange = 10;
        fly.time = rawTime;
        
            plot(rad2deg(FlyState.Left))
    
    end
	pat.time    	= AI.Time;
 	pat.xpos     	= (round((AI{:,2})*(96/vrange)));
    % pat.xpos     	= 3.75*(pat.xpos - center);
    
    fly.Ts       	= mean(diff(fly.time));
    fly.Fs          = 1/fly.Ts; 
    [fly.b,fly.a]  	= butter(2,fly.Fc/(fly.Fs/2),'low');
    fly.head     	= filtfilt(fly.b,fly.a,FlyState{:,2});
    fly.lwing      	= FlyState{:,3};
    fly.rwing     	= FlyState{:,4};
    fly.wba       	= fly.lwing - fly.rwing;
    fly.wba         = hampel(1:length(fly.wba),fly.wba);
    fly.wba         = filtfilt(fly.b,fly.a,fly.wba);
    
    fly.time      	= fly.time 	(span);
    fly.head     	= fly.head 	(span);
 	fly.wba      	= fly.wba 	(span);
 	pat.xpos     	= pat.xpos	(span);
    
    TIME            {I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.time;
    HEAD.Pos       	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.head;
  	WING.Pos    	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.wba;
  	PAT.XPos     	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = pat.xpos;
    
    HEAD.All.Pos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.head;
    WING.All.Pos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.wba;
    PAT.All.XPos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = pat.xpos;
    
    hold on
    plot(rad2deg(FlyState.Right))
    disp(FILES{kk})
    pause
    cla
    clc
    
end

%% Histogram of PATTERN Position ALL FLIES %%
fig(1) = figure (1) ; clf ; hold on
set(fig,'Color','w','Name','Histogram of Wing Position','Position',[0 0 N.WGain*300 N.HGain*200])
movegui(gcf,'center')
ax = gobjects(N.HGain,N.WGain);
vector = 1:96;
pp = 1;
for ww = 1:N.WGain
    for hh = 1:N.HGain
        ax(hh,ww) = subplot(N.HGain,N.WGain,pp); hold on
            if any(pp==(1:N.WGain))
                title(['Wing Gain = ' num2str(U.WGain{1}(ww))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N.WGain) == floor((pp-1)/N.WGain)
                ylabel(['Head Gain = ' num2str(U.HGain{1}(hh))],...
                    'FontSize',13,'fontweight','bold')
            else
                % yticklabels('')  
            end
            if any(pp==(N.WGain*N.HGain-N.WGain+1):(N.WGain*N.HGain))
                    xlabel(['\Delta WBA(' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                % xticklabels('')
            end

            pos = PAT.All.XPos{ww,hh};
            histogram(pos,vector,'facecolor','k','Normalization','pdf');

        pp = pp + 1;
    end
end
set(ax,'XLim',[1 96])
linkaxes(ax,'xy')

%% Histogram of WING Position ALL FLIES %%
fig(1) = figure (1) ; clf ; hold on
set(fig,'Color','w','Name','Delta WBA','Position',[0 0 N.WGain*300 N.HGain*200])
movegui(gcf,'center')
ax = gobjects(N.HGain,N.WGain);
vector = -70:3:70;
pp = 1;
for ww = 1:N.WGain
    for hh = 1:N.HGain
        ax(hh,ww) = subplot(N.HGain,N.WGain,pp); hold on
            if any(pp==(1:N.WGain))
                title(['Wing Gain = ' num2str(U.WGain{1}(ww))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N.WGain) == floor((pp-1)/N.WGain)
                ylabel(['Head Gain = ' num2str(U.HGain{1}(hh))],...
                    'FontSize',13,'fontweight','bold')
            else
                % yticklabels('')  
            end
            if any(pp==(N.WGain*N.HGain-N.WGain+1):(N.WGain*N.HGain))
                    xlabel(['\Delta WBA(' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                % xticklabels('')
            end

            pos = rad2deg(WING.All.Pos{ww,hh}); 
            histogram(pos,vector,'facecolor','k','Normalization','pdf');

        pp = pp + 1;
    end
end
set(ax,'XLim',80*[-1 1])
linkaxes(ax,'xy')

%% Histogram of HEAD Position ALL FLIES %%
fig(1) = figure (1) ; clf ; hold on
set(fig,'Color','w','Name','Histogram of Wing Position','Position',[0 0 N.WGain*300 N.HGain*200])
movegui(gcf,'center')
ax = gobjects(N.HGain,N.WGain);
vector = -20:1:20;
pp = 1;
for ww = 1:N.WGain
    for hh = 1:N.HGain
        ax(hh,ww) = subplot(N.HGain,N.WGain,pp); hold on
            if any(pp==(1:N.WGain))
                title(['Wing Gain = ' num2str(U.WGain{1}(ww))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N.WGain) == floor((pp-1)/N.WGain)
                ylabel(['Head Gain = ' num2str(U.HGain{1}(hh))],...
                    'FontSize',13,'fontweight','bold')
            else
                % yticklabels('')  
            end
            if any(pp==(N.WGain*N.HGain-N.WGain+1):(N.WGain*N.HGain))
                    xlabel(['\Delta WBA(' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                % xticklabels('')
            end

            pos = rad2deg(HEAD.All.Pos{ww,hh}); 
            histogram(pos,vector,'facecolor','k','Normalization','pdf');

        pp = pp + 1;
    end
end
set(ax,'XLim',20*[-1 1])
linkaxes(ax,'xy')

end