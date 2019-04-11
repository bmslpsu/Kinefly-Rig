function [MOV] = kinefly_movie(rootvid,rootpat,vidFs,export)
%% kinefly_movie: makes movie for fly in rigid tether >>> includes kinefly video ouput & pattern position
%                    pattern position and plots of data
%   INPUT:
%       rootdir     : directory containing VID files
%       rootpat     : directory containing PATTERN files
%       vidFs       : video display FPS
%       export      : boolean (1=export video to images)
%   OUTPUT:
%       MOV         : structure containing movie 
%---------------------------------------------------------------------------------------------------------------------------------
% Example Input %
clear ; clc ; close all
export = true;
vidFs = 90;
rootvid = 'C:\Users\boc5244\Box Sync\Research\bags\movie\mat\';
rootpat = 'C:\BC\Git\Arena\Patterns\';
%---------------------------------------------------------------------------------------------------------------------------------
% Set directories
root.pat    = rootpat; % pattern location
root.vid    = rootvid; % video location

% Select angle file
[FILE.vid, ~] = uigetfile({'*.mat', 'DAQ-files'}, ...
    'Select ANGLE file', root.vid, 'MultiSelect','off');
% Select pattern file
[FILE.pat, ~] = uigetfile({'*.mat', 'DAQ-files'}, ...
    'Select PATTERN file', root.pat, 'MultiSelect','off');

% Load data
disp('Loading Data...')
load([root.pat  FILE.pat],'pattern') % load pattern
load([root.vid  FILE.vid],'Vid','VidTime','AI','FlyState') % load video
disp('DONE')

% Create directories
[~,dirName,~] = fileparts([root.vid FILE.vid]); % get file name
root.mov = [root.vid 'Movie\']; % movie directory
root.image = [root.vid 'Movie\' dirName]; % image directory
mkdir(root.image) % create directory for export images

% Get video, pattern, position, & angles data 
Fly.vid = squeeze(Vid); % raw trial video data
Fly.time = VidTime; % video time
Fly.Fs = round(1/mean(diff(Fly.time)));
[Fly.xP,Fly.yP,Fly.bit,nFrame] = size(Fly.vid ); % get size of video
center = [round(Fly.yP/2) , round(Fly.xP/2)+45]; % center point for pattern & fly
radius.center = floor(max([Fly.yP Fly.xP])/1.2); % radius of pattern
radius.width = 10; % radius display width
rin  = radius.center - radius.width;
rout = radius.center + radius.width;
x1 = center(1);
y1 = center(2);
sA = 3.75 * pi/180; % angle pixel subtends
Pat.pos = round((96/5)*(AI{:,2})); % pattern position
Pat.time = AI{:,1}; % pattern time
Pat.int = interp1(Pat.time, Pat.pos, Fly.time, 'nearest'); % interpolate pattern to match fly video

% Create structure to store frames
MOV(1:nFrame) = struct('cdata', [], 'colormap',[]);

% Create video object
VID = VideoWriter([root.mov dirName '.avi'],'Uncompressed AVI');
VID.FrameRate = vidFs;
open(VID)
%%
FIG = figure ; clf ; hold on % main figure window for display & export
set(gcf, 'color', 'k');
set(FIG, 'Renderer','OpenGL');
set(FIG, 'Position',[100, 100, 1920, 1083]);
movegui(FIG,'center')
% subplot(12,1,1:8) ; cla ; hold on; axis square % for fly & pattern vid
% axis([0 Fly.yP 0 Fly.xP])
% subplot(12,1,9:10)  ; cla ; hold on ; h1 = animatedline('Color','g','LineWidth',2); % for pattern angle
% subplot(12,1,11:12) ; cla ; hold on ; h2 = animatedline('Color','c','LineWidth',2); % for head angle
pp = 1;
iter = round(Fly.Fs/vidFs);
disp('Exporting Video...')
for jj = 1:iter:nFrame % for each frame    
	pat = pattern.Pats(1,:,round(Pat.int(jj)),4); % top row of pattern
	patS = circshift(pat,[0 0]); % shift pattern to fly reference frame
    
    I = find(patS~=0);
    theta = (I.*3.75) .* (2*pi/360); % lit pixels
    theta_ALL = deg2rad(3.75*(1:96));

    Frame = Fly.vid(:,:,:,jj); % current raw frame
    DISP = Frame; % video frame to display
    
    % Display fly video
%     subplot(12,1,1:8) ; cla ; hold on ; axis square ; axis on
    imshow(DISP); hold on
    set(FIG, 'Position',[100, 100, 1920, 1083]);
    plot(x1,y1,'r.','MarkerSize',1) % display center point

    % Make pattern ring
    for kk = 1:length(theta_ALL)
        xin = x1 + rin*cos(theta_ALL(kk));
        xout = x1 + rout*cos(theta_ALL(kk));
        xinN = x1 + rin*cos(theta_ALL(kk) + sA);
        xoutN = x1 + rout*cos(theta_ALL(kk) + sA);
        yin = y1 + rin*sin(theta_ALL(kk));
        yout = y1 + rout*sin(theta_ALL(kk));
        yinN = y1 + rin*sin(theta_ALL(kk) + sA);
        youtN = y1 + rout*sin(theta_ALL(kk) + sA);
        
        if sum(ismember(theta, theta_ALL(kk))) == 1 % if lit
            patch([xout, xoutN, xinN, xin], [yout, youtN,yinN, yin],'g','linestyle','none',...
                'FaceAlpha',pat(kk)*(1/(2^(pattern.gs_val)-1)));
        else % if dark
            patch([xout, xoutN, xinN, xin], [yout, youtN,yinN, yin],'k','linestyle','none');
        end
    end
    
    % Pattern plot
%  	subplot(12,1,9:10) ; hold on ; set(gca, 'color', 'w')
%  	ylabel('Display ($^{\circ}$)','Interpreter','latex','Color','w','FontSize',12);
%     xlim([0 round(Fly.time(end))])
%     ylim([-20 20])
%  	set(gca,'ycolor','w');
%     set(gca,'xcolor','k');
%     set(gca,'YTick',[-20 0 20])
%     set(gca,'XTick',0:1:round(Fly.time(end)))
%     addpoints(h1,t_v(jj),3.75*Pat.int(jj) - 3.75*mean(Pat.int))
%     drawnow
    
    % Head plot
%     subplot(12,1,11:12) ; hold on ; set(gca, 'color', 'w')
% 	ylabel('Head ($^{\circ}$)','Interpreter','latex','Color','w','FontSize',12)
%   	xlabel('Time (s)','Interpreter','latex','Color','w','FontSize',12)    
%     xlim([0 round(Fly.time(end))])
%     ylim([-20 20])
%  	set(gca,'ycolor','w');
%  	set(gca,'xcolor','w');
%     set(gca,'YTick',[-20 0 20])
%     set(gca,'XTick',0:1:round(Fly.time(end)))
%     addpoints(h2,FlyState{jj,1},rad2deg(FlyState{jj,2}))
%     drawnow
    
    % Store frame
%     MOV(pp) = getframe(FIG);
    
    if export
        % Export frame to image
%         filename = sprintf('image%04d.jpg', pp);
%         export_fig(gcf, [root.image '\' filename], '-q95','-nocrop');
        % Write frame to .avi
        writeVideo(VID,getframe(FIG));
    end
    pp = pp + 1;
    
    pause(0.01)
end
disp('DONE')

disp('Saving...')
if export
    close(VID) % close .avi
    Fs = Fly.Fs;
% 	save([root.mov dirName '.mat'],'MOV','Fs','-v7.3','-nocompression') % save movie as .mat file
end
disp('DONE')
end