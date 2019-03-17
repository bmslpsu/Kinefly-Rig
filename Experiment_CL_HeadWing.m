function [] = Experiment_CL_HeadWing(Fn)
%% Experiment_CL_HeadWing: runs a experiment using the LED arena
% Forr Panel Controller v3 and NiDAQ seesion mode
%   INPUT:
%       Fn      : fly #
%---------------------------------------------------------------------------------------------------------------------------------
%% Set directories & experimental paramters %%
%---------------------------------------------------------------------------------------------------------------------------------
% add controller functions to path
% set ROS directories
rootdir = 'E:\bagfiles\Experiment_';
system(sprintf('export rootdir=%s',rootdir))
% EXPERIMENTAL PARAMETERS
n.exp   = 11;       % experiment time [s]
n.rest  = 5;     	% rest time [s]
n.pause = 0.2;  	% pause between panel commands [s]
n.rep   = 5;        % # of repetitions per fly
n.gain  = 5;        % # gains per fly
%% Set Experimental Gain Sequence %%
%---------------------------------------------------------------------------------------------------------------------------------
WG = [5 15 25]; % wing gains
HG = [0]; % head gains
Gain = combvec(WG,HG)'; % all gain combinations
Gain_rand = Gain(randperm(size(Gain,1)),:); % reshuffle randomly
Gain_all = repmat(Gain_rand,n.rep,1); % repeat for n.rep
n.trial = length(Gain_all);
%% EXPERIMENT LOOP %%
%---------------------------------------------------------------------------------------------------------------------------------
for kk = 1:n.trial      
    disp('Trial')
    disp(num2str(kk)); % print counter to command line
    WGain = Gain_all(kk,1);
	HGain = Gain_all(kk,2);
	filename = sprintf('%s_%i_%s_%i_%s_%i_%s_%i','fly',Fn,'trial',kk,'HGain',HGain,'WGain',WGain); % filename
   	system(sprintf('export WGAIN=%f', WGain)) % set wing gain
    system(sprintf('export HGAIN=%f', HGain)) % set head gain
    [status,~] = system(['export LD_LIBRARY_PATH="LD_path";' ... % start Kinefly with set AO parameters
        'roslaunch Kinelfy main.launch' '& echo $!']);
    if status~=0
        error('Kinefly did not launch')
    end
    %-----------------------------------------------------------------------------------------------------------------------------
    % Sinusoid function between closed-loop experiments
	disp('rest');
	Panel_com('stop');                                              % stop
    Panel_com('set_pattern_id', 1); pause(n_pause)                	% set pattern
    Panel_com('set_position',[15, 1]); pause(n_pause)               % set starting position (xpos,ypos)
    Panel_com('set_posfunc_id',[1, 0]); pause(n_pause)            	% arg1 = channel (x=1,y=2); arg2 = funcid
	Panel_com('set_funcX_freq', 50); pause(n_pause)                 % 50Hz update rate for x-channel
    Panel_com('set_mode', [3,0]); pause(n_pause)                    % 0=open,1=closed,2=fgen,3=vmode,4=pmode
	Panel_com('start') 	% start stimulus
    pause(2)
	Panel_com('stop')  	% stop stimulus
	%-----------------------------------------------------------------------------------------------------------------------------
    % Closed-loop trial
    disp(['Closed-loop Trial ' num2str(kk)])
    Panel_com('stop'); pause(n_pause)
    Panel_com('set_pattern_id', 1);pause(n_pause)                   % set output to p_rest_pat (Pattern_Fourier_bar_barwidth=8)
    Panel_com('set_position',[1, 4]); pause(n_pause)                % set starting position (xpos,ypos)
    Panel_com('set_mode',[1,0]); pause(n_pause)                     % closed loop tracking (NOTE: 0=open, 1=closed)
    Panel_com('send_gain_bias',[-10,0,0,0]); pause(n_pause)         % [xgain,xoffset,ygain,yoffset]
    [status,~] = system(['export LD_LIBRARY_PATH="LD_path";' ... % start recording
        'roslaunch Kinelfy record.launch prefix:=' filename ' & echo $!']);
    if status~=0
        error('Record did not launch')
    end
    pause(1)
    % Start experiment
    Panel_com('start');                                         % start closed-loop
    pause(n.exp)                                                % experiment time
    Panel_com('stop');                                          % stop experiment
    %-----------------------------------------------------------------------------------------------------------------------------
	% Sinusoid function between closed-loop experiments while waiting to save .bag file
	disp('rest');
	Panel_com('stop');                                              % stop
    Panel_com('set_pattern_id', 1); pause(n_pause)                	% set pattern
    Panel_com('set_position',[15, 1]); pause(n_pause)               % set starting position (xpos,ypos)
    Panel_com('set_posfunc_id',[1, 0]); pause(n_pause)            	% arg1 = channel (x=1,y=2); arg2 = funcid
	Panel_com('set_funcX_freq', 50); pause(n_pause)                 % 50Hz update rate for x-channel
    Panel_com('set_mode', [3,0]); pause(n_pause)                    % 0=open,1=closed,2=fgen,3=vmode,4=pmode
	Panel_com('start') 	% start stimulus
    pause(10)
	Panel_com('stop')  	% stop stimulus
    %-----------------------------------------------------------------------------------------------------------------------------
	[status,~] = system(['export LD_LIBRARY_PATH="LD_path";' ... % exit Kinefly
        'rostopic pub -1 kinefly/command Kinefly/MsgCommand commandtext exi' '& echo $!']);
    if status~=0
        error('Kinefly did not exit')
    end
    pause(2)
end
disp('Done');
end