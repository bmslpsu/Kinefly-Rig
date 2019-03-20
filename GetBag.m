function [Vid,VidTime,FlyState,AI,FILES] = GetBag(root,varargin)
%% GetBag: Parses file name data and returns tables with relevant information
%   INPUTS:
%       root    :   root directory containg .bag files
%   OUTPUTS:
%       BAG     :   structure containing pared data
%---------------------------------------------------------------------------------------------------------------------------------
% clear ; clc
% root = 'Q:\Box Sync\Research\bags\TEST';
%---------------------------------------------------------------------------------------------------------------------------------
% Set name of the output file as the root directory
if nargin>1 % let user set filename
    filename = varargin{1};
else
    [~,filename,~] = fileparts([root '.mat']); % get file name
    mkdir([root '\mat']) % create directory for export *.mat data
end

% Set directory & get files
[files, PATH] = uigetfile({'*.bag', 'BAG-files'}, 'Select .bag files', root, 'MultiSelect','on');
if ischar(files)
    FILES{1} = files;
else
    FILES = files';
end
clear files

n.Files = length(FILES);

% Topic information: [video , flystate, analog in]
TopicList = {'/camera/image_raw','/kinefly/flystate','/stimulus/ai'};
TopicType = {'CompressedImage','struct','struct'}';
n.Topic = length(TopicList);
tic
% Initialize cells
Bag         = cell(n.Files,1); % all bags
Topic       = cell(n.Files,n.Topic); % topics for each bag 
Msg         = cell(n.Files,n.Topic); % messages for each topic
Vid         = cell(n.Files,1); % video data
VidTime     = cell(n.Files,1); % video time
FlyState    = cell(n.Files,1); % fly kinematic data: head, left wing, right wing
AI          = cell(n.Files,1); % analog inputs
Time        = cell(n.Files,1); % time for flystate & AI
for kk = 1:n.Files
    % Get topics, messages, & time
    Bag{kk} = rosbag([PATH FILES{kk}]); % load bag
    for jj = 1:n.Topic
        Topic{kk,jj} = select(Bag{kk}, 'Topic', TopicList{jj}); % get topics
        Msg{kk,jj} = readMessages(Topic{kk,jj},'DataFormat',TopicType{jj}); % get messages
        Time{kk,jj} = table2array(Topic{kk,jj}.MessageList(:,1)); % get raw time
    end
    
   	% Initialize variables
    n.Frame         = length(Msg{kk,1}); % # of video frames
    n.FState        = length(Msg{kk,2}); % # of fly states
  	n.AState        = length(Msg{kk,3}); % # of AI states
    n.ACh           = length(Msg{kk,3}{1}.Channels); % # of AI channels
    FlyState{kk}    = nan(n.FState,4); % fly state cell (header: time,head,LW,RW)
    AI{kk}          = nan(n.AState,n.ACh+1); % AI channel cell (header: time,ch0,ch1,ch2)

 	InitFrame = readImage(Msg{kk,1}{1});    % first video frame
    [n.PixelY,n.PixelX] = size(InitFrame);  % size of first video frame
    Vid{kk} = uint8(nan(n.PixelY,n.PixelX,n.Frame)); % video cell
    
    % Sync times
    syncTime = Time{kk,2}(1); % sync times to flystate
    VidTime{kk}(:,1)  = Time{kk,1} - syncTime; % video time
    FlyState{kk}(:,1) = Time{kk,2} - syncTime; % flystate time
    AI{kk}(:,1) = Time{kk,3} - syncTime; % AI time
    
    % Store relevant messages in cells
    for jj = 1:n.AState % most states in AI
        AI{kk}(jj,2:4) = Msg{kk,3}{jj}.Voltages; % AI voltage
        if jj<=n.Frame
            Vid{kk}(:,:,jj) = readImage(Msg{kk,1}{jj}); % video frame
        end
        if jj<=n.FState
            FlyState{kk}(jj,2) = Msg{kk,2}{jj}.Head.Angles;    % left wing angle
            FlyState{kk}(jj,3) = Msg{kk,2}{jj}.Left.Angles;    % right wing angle
            FlyState{kk}(jj,4) = Msg{kk,2}{jj}.Right.Angles;   % head angle
        end
    end
    FlyState{kk} = splitvars(table(FlyState{kk}));
    FlyState{kk}.Properties.VariableNames = {'Time','Head','LWing','RWing'};
	AI{kk} = splitvars(table(AI{kk}));
    AI{kk}.Properties.VariableNames = {'Time','Ch0','Ch1','Ch2'};
end

% Save data
disp('Saving...')
save([root '\mat\' filename '.mat'] , 'Vid','VidTime','FlyState','AI','FILES' ,'-v7.3')
disp('DONE')
toc
end