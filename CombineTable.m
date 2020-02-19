function [] = CombineTable(root)
%%
clc
clear

%% Combine Table:
%   INPUTS:
%       root  	: directory where .mat files are located
%   OUTPUTS:
%       -
%

root = '/Volumes/HD_1000GB/JF/Misc./stats/';
 
[FILES, PATH] = uigetfile({'','files'},'Select files',root,'MultiSelect','on');
FILES = cellstr(FILES)';
%%
% Select files
% [D,I,N,U,T,FILES,PATH,~] = GetFileData(root,[],false,'Fly','Trial','HGain','WGain');
nfile = length(FILES);
T = cell(nfile,1);
icr = 0;
for kk = 1:nfile
    path = fullfile(PATH,FILES{kk});
    T{kk} = load(path,'stats');
    nfly = max(T{kk}.stats.Fly);
	T{kk}.stats.Fly = T{kk}.stats.Fly  + icr;
    icr = nfly + icr;
end

T = cellfun(@(x) x.stats,T,'UniformOutput', false);
T = cat(1,T{:});

Combined_Table = T;