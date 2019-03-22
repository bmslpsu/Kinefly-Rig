%% -------------------------------------------------------------------------------------------------------------------------------
root = 'C:\Users\boc5244\Box Sync\Research\bags\TEST\mat\';

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
clc

Fc = 20;
for kk = 1:n.Files
    load([PATH FILES{kk}],'AI')
    time = AI{:,1};
    Fs = 1/mean(diff(time));
	[b,a] = butter(2,Fc/(Fs/2),'low');
    raw = round((96/5)*AI{:,2});
    fraw = medfilt1(raw,10);
%     filtered = filtfilt(b,a,raw);
%     medfiltered = medfilt1(filtered,10);
%     panel = round(medfiltered);
    figure (1)
    subplot(2,2,kk) ; hold on
    plot(time,raw)
    plot(time,fraw)
    
    
end
