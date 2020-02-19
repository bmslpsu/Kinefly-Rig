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

 root = '/Volumes/HD_1000GB/JF';

% Select files
[D,I,N,U,T,FILES,PATH,~] = GetFileData(root,[],false,'Fly','Trial','HGain','WGain');

if ~iscell(U.HGain)
    U.HGain = {U.HGain};
end
if ~iscell(U.WGain)
    U.WGain = {U.WGain};
end
%%
% **CREATE VECTOR STRENGTH TABLES** 
newvars = zeros(size(I,1),4);
AddTableVars = splitvars((table(zeros(size(I,1),8))));
AddTableVars.Properties.VariableNames = {'zPat', 'pPat','zH','pH','zW','pW','zErr','pErr'};
stats = [I AddTableVars];

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
        center =89;
        fly.time = VidTime;
    elseif exist('rawTime','var')==1
        center = 41;
        vrange = 10;
        fly.time = rawTime;    
    end
	pat.time    	= AI.Time;
    
 	pat.xpos     	= (round((AI{:,2})*(96/vrange)));
    pat.xpos     	= pat.xpos - center;   
    
%      pat.xpos(pat.xpos<(-1*(96/2))) = pat.xpos(pat.xpos<(-1*(96/2))) + (96); %look at this!!!!
%     pat.xpos = deg2rad(3.75*pat.xpos);
    
    fly.Ts       	= mean(diff(fly.time));
    fly.Fs          = 1/fly.Ts; 
    [fly.b,fly.a]  	= butter(2,fly.Fc/(fly.Fs/2),'low');
    fly.head     	= deg2rad(filtfilt(fly.b,fly.a,FlyState{:,2}));
    fly.lwing      	= hampel(1:length(FlyState{:,3}),FlyState{:,3});
    fly.rwing     	= hampel(1:length(FlyState{:,4}),FlyState{:,4});
    fly.lwing       = hampel(1:length(fly.lwing),fly.lwing);
    fly.rwing     	= hampel(1:length(fly.rwing),fly.rwing);
    fly.wba       	= fly.lwing - fly.rwing;
    fly.wba         = deg2rad(filtfilt(fly.b,fly.a,fly.wba));
    
    fly.time      	= fly.time 	(span);
    fly.head     	= fly.head 	(span);
 	fly.wba      	= fly.wba 	(span);
 	pat.xpos     	= pat.xpos	(span);
    Err             = pat.xpos - fly.head;
    
    [stats.pPat(kk),stats.zPat(kk)] = circ_rtest(pat.xpos);
    [stats.pH(kk),stats.zH(kk)] = circ_rtest(fly.head);
    [stats.pW(kk),stats.zW(kk)] = circ_rtest(fly.wba);
    [stats.pErr(kk),stats.zErr(kk)] = circ_rtest(Err);
    
    
    TIME            {I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.time;
    HEAD.Pos       	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.head;
  	WING.Pos    	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.wba;
  	PAT.XPos     	{I.Fly(kk)}{I.WGain(kk),I.HGain(kk)}(:,end+1) = pat.xpos;
    
    HEAD.All.Pos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.head;
    WING.All.Pos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = fly.wba;
    PAT.All.XPos  	{I.WGain(kk),I.HGain(kk)}(:,end+1) = pat.xpos;
   
    
    
% **UNCOMMENT TO PLOT ONE TRIAL AT A TIME** 
%     clf
%     cla
%     hold on
%     plot(rad2deg(FlyState{:,4}))
%     plot((FlyState{:,3}),'b')
%     plot(fly.lwing,'r')
%     plot(pat.xpos)
%     disp(FILES{kk})           
%     pause
%     cla
%     clc
    
end

%% Histogram of PATTERN Position ALL FLIES %%
fig(1) = figure (1); 
clf; 
hold on

set(fig,'Color','w','Name','Histogram of Pattern Position','Position',[0 0 N.WGain*300 N.HGain*200])
movegui(gcf,'center')
ax = gobjects(N.HGain,N.WGain);
vector = 0:deg2rad(3.75*2):2*pi;

pp = 1;
for ww = 1:N.WGain
    for hh = 1:N.HGain
        ax(hh,ww) = subplot(N.HGain,N.WGain,pp); 
          
        pos = deg2rad(3.75*(PAT.All.XPos{ww,hh}));
            pos = pos(:);
%           histogram(pos,vector,'facecolor','k','Normalization','pdf');
            polarhistogram(pos,vector,'facecolor','k','Normalization','pdf');
            hold on
            
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
                    xlabel(['Pattern Position(' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                % xticklabels('')
            end


           

        pp = pp + 1;
    end
end
%  set(ax,'XLim',[1 96])
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