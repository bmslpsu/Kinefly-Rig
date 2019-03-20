function [Vid,VidTime,FlyState,AI,FILES] = bag2mat(root)
%% GetBag_Files: Parses file name data and returns tables with relevant information stored in .mat files
%   INPUTS:
%       root        :   root directory containg .bag files >>> files will be saved in a folder titled "mat"
%       inside this directory
%   OUTPUTS:
%       Vid         :   raw video data
%       VidTime     :   raw video time
%       FlyState  	:   fly kinematic data
%       AI          :   analog input voltages
%       FILES    	:   filename listing
%---------------------------------------------------------------------------------------------------------------------------------
% root = 'Q:\Box Sync\Research\bags\TEST';
%---------------------------------------------------------------------------------------------------------------------------------
% Set output directory to store .mat files
mkdir([root '\mat']) % create directory for export *.mat data

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

W = waitbar(0/n.Files,'Saving data...');
tic
for kk = 1:n.Files
    % Get topics, messages, & time
    Bag     = rosbag([PATH FILES{kk}]); % load bag
    Msg     = cell(1,n.Topic); % messages for each topic
    Time  	= cell(1,n.Topic); % time for flystate & AI
    for jj = 1:n.Topic
        Topic = select(Bag, 'Topic', TopicList{jj}); % get topics
        Msg{jj} = readMessages(Topic,'DataFormat',TopicType{jj}); % get messages
        Time{jj} = table2array(Topic.MessageList(:,1)); % get raw time
    end
    
   	% Initialize variables
    n.Frame         = length(Msg{1}); % # of video frames
    n.FState        = length(Msg{2}); % # of fly states
  	n.AState        = length(Msg{3}); % # of AI states
    n.ACh           = length(Msg{3}{1}.Channels); % # of AI channels
    FlyState        = nan(n.FState,4); % fly state cell (header: time,head,LW,RW)
    AI              = nan(n.AState,n.ACh+1); % AI channel cell (header: time,ch0,ch1,ch2)

 	InitFrame = readImage(Msg{1}{1});    % first video frame
    [n.PixelY,n.PixelX] = size(InitFrame);  % size of first video frame
    Vid = uint8(nan(n.PixelY,n.PixelX,n.Frame)); % video cell
    
    % Sync times
    syncTime        = Time{1}(1); % sync times to vid frame
    VidTime(:,1)    = Time{1} - syncTime; % video time
    FlyState(:,1)   = Time{2} - syncTime; % flystate time
    AI(:,1)         = Time{3} - syncTime; % AI time
    
    % Store relevant messages in cells
    for jj = 1:max([n.AState,n.Frame,n.FState]) % cycle through states in AI maximum samples in topics
        if jj<=n.AState
            AI(jj,2:4) = Msg{3}{jj}.Voltages; % AI voltage
        end
        if jj<=n.Frame
            Vid(:,:,jj) = readImage(Msg{1}{jj}); % video frame
        end
        if jj<=n.FState
            FlyState(jj,2) = Msg{2}{jj}.Head.Angles;    % left wing angle
            FlyState(jj,3) = Msg{2}{jj}.Left.Angles;    % right wing angle
            FlyState(jj,4) = Msg{2}{jj}.Right.Angles;   % head angle
        end
    end
    
    % Put data in tables
    FlyState = splitvars(table(FlyState));
    FlyState.Properties.VariableNames = {'Time','Head','LWing','RWing'};
	AI = splitvars(table(AI));
    AI.Properties.VariableNames = {'Time','Ch0','Ch1','Ch2'};
    
    % Save .mat file in directory
    [~,filename,~] = fileparts(FILES{kk}); % get filename
    dateIdx = strfind(filename,'201'); % will work until 2020
    filename = filename(1:dateIdx-2); % remove dat-time at end of filename    
    save([root '\mat\' filename '.mat'] , 'Vid','VidTime','FlyState','AI' ,'-v7.3')
    waitbar(kk/n.Files,W,'Saving data...');
    
    clear Vid VidTime FlyState AI Time Msg Topic Bag
end
close(W)
disp('DONE')
toc
beep on
for kk = 1:5
    beep
    pause(0.5)
end
end