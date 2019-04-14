function [MOV] = kinefly_movie(rootvid,vidFs,export)
%% kinefly_movie:   makes movie for fly in rigid tether >>> includes kinefly video ouput & pattern position 
%                   pattern position and plots of data
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
rootvid = 'H:\MOVIE\Kinefly_Demo\mat\';
%---------------------------------------------------------------------------------------------------------------------------------
% Set directories
root.vid    = rootvid; % video location

% Select angle file
[FILE.vid, ~] = uigetfile({'*.mat', 'DAQ-files'}, ...
    'Select ANGLE file', root.vid, 'MultiSelect','off');

% Load data
disp('Loading Data...')
load(fullfile(root.vid,FILE.vid),'Vid','VidTime','AI','FlyState') % load video
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

% Create video object
VID = VideoWriter([root.mov dirName '.avi'],'Uncompressed AVI');
VID.FrameRate = vidFs;
open(VID)

close all
FIG = figure ; clf  % main figure window for display & export
set(gcf, 'color', 'k');
set(FIG, 'Renderer','OpenGL');
movegui(FIG,'center')
pp = 1;
iter = round(Fly.Fs/vidFs);
disp('Exporting Video...')
for jj = 1:iter:nFrame % for each frame       
    % Display fly video
%     imshow(Fly.vid(:,:,:,jj)) ; hold on ; axis tight
%     axis([0 Fly.yP 0 Fly.xP])
    
    if export
        % Export frame to image
%         filename = sprintf('image%04d.jpg', pp);
%         export_fig(gcf, [root.image '\' filename], '-q95','-nocrop');
        % Write frame to .avi
        writeVideo(VID,Fly.vid(:,:,:,jj));
    end
    pp = pp + 1;
    disp(jj)
    pause(0.01)
end
disp('DONE')

disp('Saving...')
if export
    close(VID) % close .avi
    Fs = Fly.Fs;
end
disp('DONE')
end