function [] = plot_head_wings(varargin)
%% plot_head_wings: plots head & wing kinematics on video
%   INPUTS:
%       leftAng  	: left wing angle in radians
%       rightAng  	: right wing angle in radians
%       headAng  	: heead angle in radians
%       Vid         : Video matrix
%       playback 	: playback interval (1 shows every frame, 2 shows every other frame...)
%   OUTPUTS:
%       -
%   USAGE:
%       plot_head_wings(leftAng,reftAng,Vid)
%           - only shows wings
%       plot_head_wings(headAng,Vid)
%           - only shows head
%       plot_head_wings(leftAng,reftAng,headAng,Vid)
%           - show wings & head
%       plot_head_wings(-,playback)
%           - set playback
%---------------------------------------------------------------------------------------------------------------------------------
% Defaults
playback = 1;
wing.flag = true;
head.flag = true;

if nargin<2
    error('Not enough input arguments')
elseif nargin==2
    head.ang    = varargin{1};
    Vid         = varargin{2};
    wing.flag   = false;
elseif nargin>=3
    left.ang	= varargin{1};
	right.ang 	= varargin{2};
    if nargin==3
        Vid         = varargin{3};
        head.flag 	= false;
    elseif nargin==4
        if size(varargin{3},3)>1
            Vid         = varargin{3};
            playback    = varargin{4};
            head.flag   = false;
        elseif size(varargin{3},3)==1
            head.ang    = varargin{3};
            Vid         = varargin{4};
        end
    elseif nargin==5
        head.ang        = varargin{3};
        Vid             = varargin{4};
        playback        = varargin{5};
    end

end

Vid = squeeze(Vid);
dim = size(Vid);
head.R = 0.15*min(dim(1:2));
wing.R = 0.30*min(dim(1:2));

figure (1) ; clf
imshow(Vid(:,:,1)); hold on
axis([0 dim(2) 0 dim(1)])

% Get center points
h.neck  = impoint(gca,0.5*dim(2), 0.4*dim(1));
h.left  = impoint(gca,0.4*dim(2), 0.5*dim(1));
h.right = impoint(gca,0.6*dim(2), 0.5*dim(1));
setColor(h.neck,'b')
setColor(h.left,'r')
setColor(h.right,'g')
pause % wait for user
head.center  = getPosition(h.neck);
left.center  = getPosition(h.left);
right.center = getPosition(h.right);

% Play back video with animations
for kk = 1:playback:dim(3)
	imshow(Vid(:,:,kk))
    if head.flag
        head.end  = head.center  + head.R*[-sin(head.ang(kk))  , -cos(head.ang(kk))];
    	plot([head.center(1),head.end(1)],[head.center(2),head.end(2)],'-b','LineWidth',2)
    end
    
    if wing.flag
        left.end  = left.center  + wing.R*[-cos(left.ang(kk))  , -sin(left.ang(kk))];
        right.end = right.center + wing.R*[ cos(right.ang(kk)) , -sin(right.ang(kk))];

        plot([left.center(1),left.end(1)],[left.center(2),left.end(2)],'-r','LineWidth',2)
        plot([right.center(1),right.end(1)],[right.center(2),right.end(2)],'-g','LineWidth',2)
    end

    pause(0.01)   
end
end