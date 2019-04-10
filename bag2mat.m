function [Vid,VidTime,FlyState,AI,FILES] = bag2mat(varargin)
%% bag2mat: Parses file name data and returns tables with relevant information: saves in .mat files
%   INPUTS:
%       root        :   varargin=root , root directory containg .bag files >>> files will be saved in a folder titled "mat"
%                       inside this directory. If no input is given, will default to current folder.
%   OUTPUTS:
%       Vid         :   raw video data
%       VidTime     :   normalized video time
%       FlyState  	:   fly kinematic data
%       AI          :   analog input voltages
%       FILES    	:   filename listing
%---------------------------------------------------------------------------------------------------------------------------------
%   USAGE:
%       [] = bag2mat()
%           - opens dialog window to select files in current folder
%       [] = bag2mat(root)
%           - opens dialog window to select files in user defined root folder
%       [] = bag2mat(root,'kinefly')
%           - for kinefly export RGB video
%---------------------------------------------------------------------------------------------------------------------------------
% root = 'C:\Users\boc5244\Box Sync\Research\bags\TEST';
%---------------------------------------------------------------------------------------------------------------------------------
% Allow user to set root directory & set video type
vidFlag = false; % default is raw video
if nargin==0
    root = '';
elseif nargin==1
    root = varargin{1};
elseif nargin==2
    root = varargin{1};
    if strcmp(varargin{2},'raw')
        vidFlag = false;
    elseif strcmp(varargin{2},'kinefly') 
        vidFlag = true;
    else
        error('2nd input must be ''raw'' or ''kinefly''')        
    end
elseif nargin>2
    error('Too many inputs')
else
    error('DEBUG')
end

% Set directory & get files
[FILES, PATH] = uigetfile({'*.bag', 'BAG-files'}, 'Select .bag files', root, 'MultiSelect','on');
FILES = cellstr(FILES)'; % if only one file, store in cell
n.Files = length(FILES); % # of .bag files to parse

% Set output directory to store .mat files
matdir = [PATH 'mat']; % export directory to save .mat files
[status,msg,~] = mkdir(matdir); % create directory for .mat files
if status
    warning(msg)
    disp(['Folder located: ' matdir])
else
    error('Directory not created')
end

% Topic information: [video , flystate, analog in]
if vidFlag
    TopicList = {'/kinefly/image_output','/kinefly/flystate','/stimulus/ai'}';
else
    TopicList = {'/camera/image_raw','/kinefly/flystate','/stimulus/ai'}';
end

TopicType = {'CompressedImage','struct','struct'}';
n.Topic = length(TopicList); % # of topics in .bag files

W = waitbar(0/n.Files,'Saving data...');
tic
for kk = 1:n.Files
    % Get topics, messages, & time
    Bag     = rosbag([PATH FILES{kk}]); % load bag
    Msg     = cell(1,n.Topic); % messages for each topic
    Time	= cell(1,n.Topic); % time for flystate & AI
    for jj = 1:n.Topic
        Topic = select(Bag, 'Topic', TopicList{jj}); % get topics
        Msg{jj} = readMessages(Topic,'DataFormat',TopicType{jj}); % get messages
        Time{jj} = table2array(Topic.MessageList(:,1)); % get raw time
    end
    
   	% Initialize variables
    n.Frame  	= length(Msg{1}); % # of video frames
    n.FState   	= length(Msg{2}); % # of fly states
  	n.AState  	= length(Msg{3}); % # of AI states
    n.ACh    	= length(Msg{3}{1}.Channels); % # of AI channels
    FlyState	= nan(n.FState,6); % fly state cell (header: time,head,LW,RW)
    AI        	= nan(n.AState,n.ACh+1); % AI channel cell (header: time,ch0,ch1,ch2)

 	InitFrame = readImage(Msg{1}{1}); % first video frame
    [n.PixelY,n.PixelX,n.bit] = size(InitFrame); % size of first video frame
    Vid = uint8(nan(n.PixelY,n.PixelX,n.bit,n.Frame)); % video cell
    
    % Sync times
    syncTime        = Time{1}(1); % sync times to vid frame
    VidTime(:,1)    = Time{1} - syncTime; % video time
    FlyState(:,1)   = Time{2} - syncTime; % flystate time
    AI(:,1)         = Time{3} - syncTime; % AI time
    
    % Store relevant messages in cells
    for jj = 1:max([n.AState,n.Frame,n.FState]) % cycle through states
        if jj<=n.AState
            AI(jj,2:n.ACh+1) = Msg{3}{jj}.Voltages; % AI voltage
        end
        if jj<=n.Frame
            Vid(:,:,:,jj) = readImage(Msg{1}{jj}); % video frame
        end
        if jj<=n.FState
            if ~isempty(Msg{2}{jj}.Head.Angles)
                FlyState(jj,2) = Msg{2}{jj}.Head.Angles;    % left wing angle
            end
            if ~isempty(Msg{2}{jj}.Left.Angles)
                FlyState(jj,3) = Msg{2}{jj}.Left.Angles;    % right wing angle
            end
            if ~isempty(Msg{2}{jj}.Right.Angles)
                FlyState(jj,4) = Msg{2}{jj}.Right.Angles;   % head angle
            end
            if ~isempty(Msg{2}{jj}.Abdomen.Angles)
                FlyState(jj,5) = Msg{2}{jj}.Abdomen.Angles;	% abdomen angle
            end
            if ~isempty(Msg{2}{jj}.Aux.Freq)
                FlyState(jj,6) = Msg{2}{jj}.Aux.Freq;       % WBF
            end
        end
    end
    
    % Put data in tables
    FlyState = splitvars(table(FlyState));
    FlyState.Properties.VariableNames = {'Time','Head','LWing','RWing','Abdomen','WBF'}; % fly state variables
	AI = splitvars(table(AI));
    chList = cell(n.ACh+1,1);
    chList{1} = 'Time';
    for jj = 1:n.ACh
       chList{jj+1} = ['Ch' num2str(jj-1)];
    end
    AI.Properties.VariableNames = chList; % AI variables
    
    % Save .mat file in directory
    [~,filename,~] = fileparts(FILES{kk}); % get filename
    dateIdx = strfind(filename,'201'); % will work until 2020
    filename = filename(1:dateIdx-2); % remove date-time at end of filename    
    save([PATH '\mat\' filename '.mat'] , 'Vid','VidTime','FlyState','AI','FILES','-v7.3') % save data to .mat file
    waitbar(kk/n.Files,W,'Saving data...');
    
    clear Vid VidTime FlyState AI Time Msg Topic Bag syncTime
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