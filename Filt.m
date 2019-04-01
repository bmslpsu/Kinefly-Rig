%% -------------------------------------------------------------------------------------------------------------------------------
root = 'H:\EXPERIMENTS\Experiment_ChirpLog_Walking\mat\';

% Set directory & get files
[files, PATH] = uigetfile({'*.mat', 'mat-files'}, 'Select .mat files', root, 'MultiSelect','on');
if ischar(files)
    FILES{1} = files;
else
    FILES = files';
end
clear files

n.Files = length(FILES);
%%
clc ; close all
span = 1:6600;
Fc = 20;
step = {};
for kk = 1:n.Files
    load([PATH FILES{kk}],'AI')
    time = AI{:,1}(span);
    Fs = 1/mean(diff(time));
	[b,a] = butter(5,Fc/(Fs/2),'low');
    step{1}         = (96/5)*AI{:,2}(span);
  	step{end+1}     = filtfilt(b,a,step{end});
    step{end+1}     = medfilt1(step{end},10);
    step{end+1}     = round(step{end});
    step{end+1}     = medfilt1(step{end},20);

    figure (1)
	hold on ; ylim([18 26])
    for jj = 1:length(step)
        clf ; hold on ; ylim([18 26])
        plot(time,step{jj})        
    end
    
    
end
