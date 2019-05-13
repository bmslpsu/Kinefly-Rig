function [] = JACK_Analyze_Template(rootdir)
%% Analyze_Template: Reads in all raw trials, transforms data, and saves in organized structure
%   INPUTS:
%       root  	: directory where .mat files are located
%   OUTPUTS:
%       -
%---------------------------------------------------------------------------------------------------------------------------------
% EXAMPLE INPUT %
rootdir = 'E:\Jack\Experiment_Wing_CL_Figure\mat';
%---------------------------------------------------------------------------------------------------------------------------------
%% Setup Directories %%
%---------------------------------------------------------------------------------------------------------------------------------
% Select files
[FILES, PATH] = uigetfile({'*.mat', 'DAQ-files'}, 'Select files', rootdir, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one character array >> store in cell

[~,I,N,U] = GetFileData(FILES,false,'Fly','Trial','HGain','WGain');

%% Get Data %%
%---------------------------------------------------------------------------------------------------------------------------------
TIME            = cell(N{1,1},1);
HEAD.Pos        = cell(N{1,1},1);
WING.Pos        = cell(N{1,1},1);
PAT.XPos        = cell(N{1,1},1);
PAT.YPos        = cell(N{1,1},1);
HEAD.All.Pos    = cell(N{1,4},N{1,3});
WING.All.Pos    = cell(N{1,4},N{1,3});
PAT.All.XPos    = cell(N{1,4},N{1,3});
PAT.All.YPos    = cell(N{1,4},N{1,3});
for kk = 1:N{1,1} % fly
    for jj = 1:N{1,4} % wing gain
        for ii = 1:N{1,3} % head gain
            TIME    {kk,1}{jj,ii} 	= [];
            HEAD.Pos{kk,1}{jj,ii}  	= [];
            WING.Pos{kk,1}{jj,ii}   = [];
            PAT.XPos{kk,1}{jj,ii}   = [];
            PAT.YPos{kk,1}{jj,ii}   = [];
        end
    end
end
fly = [];
FlyState = [];
AI = [];
fly.Fc = 20;
span = 20:1:2000;
for kk = 1:N{1,end} % all trials
    filename = fullfile(PATH,FILES{kk}); % full file name
    load(filename,'FlyState','AI','VidTime') % load in fly kinematics & arena voltages
    
    vid.time            = VidTime;
    fly.time            = FlyState{:,1};
    fly.Ts              = mean(diff(fly.time));
    fly.Fs              = 1/fly.Ts; 
	[fly.b,fly.a]    	= butter(2,fly.Fc/(fly.Fs/2),'low'); % 2nd-order low-pass butterworth filter
    fly.head.pos        = filtfilt(fly.b,fly.a,FlyState{:,2});
  	fly.wing.pos        = filtfilt(fly.b,fly.a,filtfilt(fly.b,fly.a,FlyState{:,3}) - filtfilt(fly.b,fly.a,FlyState{:,4}));
	fly.head.vel        = [diff(fly.head.pos)./fly.Ts ; 0];
	fly.wing.vel        = [diff(fly.wing.pos)./fly.Ts ; 0];
    pat.time            = AI{:,1};
    pat.Ts              = mean(diff(pat.time));
    pat.Fs              = 1/pat.Ts;
    pat.Fc              = 30;
	[pat.b,pat.a]    	= butter(2,pat.Fc/(pat.Fs/2),'low'); % 2nd-order low-pass butterworth filter
    pat.xpos            = filtfilt(pat.b,pat.a,AI{:,2});
 	pat.xpos            = 3.75*(round((AI{:,2})*(96/5)));
    pat.xpos            = FitPanel(pat.xpos,pat.time,vid.time,false,false); % true does debugging, false does not 
    pat.xpos            = medfilt1(pat.xpos,5);
%     pat.xpos            = rad2deg(wrapToPi(deg2rad(pat.xpos)));
    pat.xpos            = pat.xpos/3.75;
 	pat.ypos            = (round((AI{:,3})*(96/5)));
	
    fly.time            = vid.time    	(span);
    fly.head.pos        = fly.head.pos 	(span);
 	fly.wing.pos        = fly.wing.pos 	(span);
	fly.head.vel        = fly.head.vel 	(span);
 	fly.wing.vel        = fly.wing.vel 	(span);
 	pat.xpos            = pat.xpos     	(span);
 	pat.ypos            = pat.ypos     	(span);
    
    TIME                {I{kk,1}}{I{kk,4},I{kk,3}}(:,end+1) = fly.time;
    HEAD.Pos            {I{kk,1}}{I{kk,4},I{kk,3}}(:,end+1) = fly.head.pos;
  	WING.Pos            {I{kk,1}}{I{kk,4},I{kk,3}}(:,end+1) = fly.wing.pos;
  	PAT.XPos            {I{kk,1}}{I{kk,4},I{kk,3}}(:,end+1) = pat.xpos ;
  	PAT.YPos            {I{kk,1}}{I{kk,4},I{kk,3}}(:,end+1) = pat.ypos;
    HEAD.All.Pos        {I{kk,4},I{kk,3}}(:,end+1) = fly.head.pos;
    WING.All.Pos        {I{kk,4},I{kk,3}}(:,end+1) = fly.wing.pos;
    PAT.All.XPos        {I{kk,4},I{kk,3}}(:,end+1) = pat.xpos;
    PAT.All.YPos        {I{kk,4},I{kk,3}}(:,end+1) = pat.ypos;
end
%% Plot pattern position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (100) ; clf ; hold on
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
                ylim([1 96])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel({['Head Gain = ' num2str(U{1,3}{1}(ii))],['Pattern Position(' char(176) ')']},...
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

                time = TIME{kk}{jj,ii};
                pos  = (PAT.XPos{kk}{jj,ii});
                plot(time,pos)

            pp = pp + 1;
        end
    end
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

                time = TIME{kk}{jj,ii};
                pos  = rad2deg(HEAD.Pos{kk}{jj,ii});
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
                ylim(40*[-1 1])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel({['Head Gain = ' num2str(U{1,3}{1}(ii))],['\Delta WBA(' char(176) ')']},...
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

                time = TIME{kk}{jj,ii};
                pos  = rad2deg(WING.Pos{kk}{jj,ii});
                plot(time,pos)

            pp = pp + 1;
        end
    end
end
%% Histogram of Pattern Position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (3) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Pattern Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector = 1:96;
for kk = 1:N{1,1}
    pp = 1;
    for jj = 1:N{1,4}
        for ii = 1:N{1,3}
            subplot(N{1,3},N{1,4},pp) ; hold on
                %xlim([0 20])
                %ylim(120*[-1 1])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                        'FontSize',13,'fontweight','bold')
                else
                    yticks(0)
                    yticklabels('')  
                end
                if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel('Pattern Position','FontSize',12,'fontweight','bold')
                else
                    xticks(0)
                    xticklabels('')
                end
                
                pos  = (PAT.XPos{kk}{jj,ii});
                histogram(pos,1:96);

            pp = pp + 1;
        end
    end
end

%% Histogram of HEAD Position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (4) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Head Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector1 = -20:1:20;
for kk = 1:N{1,1}
    pp = 1;
    for jj = 1:N{1,4}
        for ii = 1:N{1,3}
            subplot(N{1,3},N{1,4},pp) ; hold on
                ylim([0 3500])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                        'FontSize',13,'fontweight','bold')
                else
                    yticks(0)
                    yticklabels('')  
                end
                if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel(['Head Angle (' char(176) ')'],'FontSize',12,'fontweight','bold')
                else
                    xticks(0)
                    xticklabels('')
                end

                pos  = rad2deg(HEAD.Pos{kk}{jj,ii});
                histogram(pos,vector1);

            pp = pp + 1;
        end
    end
end
%% Histogram of WING Position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (5) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Wing Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector1 = -35:1:35;
for kk = 1:N{1,1}
    pp = 1;
    for jj = 1:N{1,4}
        for ii = 1:N{1,3}
            subplot(N{1,3},N{1,4},pp) ; hold on
                ylim([0 1500])
                if any(pp==(1:N{1,4}))
                    title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
                end
                if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                    ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                        'FontSize',13,'fontweight','bold')
                else
                    yticks(0)
                    yticklabels('')  
                end
                if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel(['\Delta WBA(' char(176) ')'],'FontSize',12,'fontweight','bold')
                else
                    xticks(0)
                    xticklabels('')
                end

                pos  = rad2deg(WING.Pos{kk}{jj,ii});
                histogram(pos,vector1);

            pp = pp + 1;
        end
    end
end
%% Histogram of Head Position ALL FLIES %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (6) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Head Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector = -20:1:20;
pp = 1;
for jj = 1:N{1,4}
    for ii = 1:N{1,3}
        subplot(N{1,3},N{1,4},pp) ; hold on
%             xlim([0 20])
            ylim([0 12000])
            if any(pp==(1:N{1,4}))
                title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                    'FontSize',13,'fontweight','bold')
            else
                yticks(0)
                yticklabels('')  
            end
            if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                xlabel(['Head Angle (' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                xticks(0)
                xticklabels('')
            end

            pos  = rad2deg(HEAD.All.Pos{jj,ii}); 
            histogram(pos,vector,'facecolor','k');

        pp = pp + 1;
    end
end
%% Histogram of Pattern Position ALL FLIES %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (7) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Pattern Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector = -180:1:180;
pp = 1;
for jj = 1:N{1,4}
    for ii = 1:N{1,3}
        subplot(N{1,3},N{1,4},pp) ; hold on
            %xlim([0 20])
            %ylim(120*[-1 1])
            if any(pp==(1:N{1,4}))
                title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                    'FontSize',13,'fontweight','bold')
            else
                yticks(0)
                yticklabels('')  
            end
            if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                xlabel('Pattern Position','FontSize',12,'fontweight','bold')
            else
                xticks(0)
                xticklabels('')
            end

            pos  = (PAT.All.XPos{jj,ii}); 
            histogram(pos,'facecolor','k');

        pp = pp + 1;
    end
end
%% Histogram of Wing Position ALL FLIES %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (8) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Pattern Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector = -30:1:30;
pp = 1;
for jj = 1:N{1,4}
    for ii = 1:N{1,3}
        subplot(N{1,3},N{1,4},pp) ; hold on
            %xlim([0 20])
            ylim([0 2000])
            if any(pp==(1:N{1,4}))
                title(['Wing Gain = ' num2str(U{1,4}{1}(jj))],'FontSize',12,'fontweight','bold')
            end
            if ((pp-1)/N{1,4}) == floor((pp-1)/N{1,4})
                ylabel(['Head Gain = ' num2str(U{1,3}{1}(ii))],...
                    'FontSize',13,'fontweight','bold')
            else
                yticks(0)
                yticklabels('')  
            end
            if any(pp==(N{1,4}*N{1,3}-N{1,4}+1):(N{1,4}*N{1,3}))
                    xlabel(['\Delta WBA(' char(176) ')'],'FontSize',12,'fontweight','bold')
            else
                xticks(0)
                xticklabels('')
            end

            pos  = rad2deg(WING.All.Pos{jj,ii}); 
            histogram(pos,vector,'facecolor','k');

        pp = pp + 1;
    end
end
%% Box Plot of Head Position %%
%---------------------------------------------------------------------------------------------------------------------------------
figure (9) ; clf ; hold on
set(gcf,'Color','w')
set(gcf,'Name','Histogram of Head Position')
set(gcf,'Position',[0 0 N{1,4}*400 N{1,3}*400])
movegui(gcf,'center')
vector = -20:1:20;
pp = 1;
for jj = 1:N{1,4}
    for ii = 1:N{1,3}
        subplot(N{1,3},N{1,4},pp) ; hold on
            %xlim([0 20])
            %ylim(120*[-1 1])
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
                xlabel('Pattern Position','FontSize',12,'fontweight','bold')
            else
                xticks(0)
                xticklabels('')
            end

            pos  = rad2deg(HEAD.All.Pos{jj,ii}); 
            histogram(pos,vector,'facecolor','k');

        pp = pp + 1;
    end
end
%% Box Plot of Wing Position

%% Box Plot of Pattern Position

end