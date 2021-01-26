% This function initializes an 2D vehicle animation plane
%    initialize_figure(i,varargin)
% varargin can specify xlim, ylim
%
% Changliu Liu
% 2015.8

function fighandle=initialize_figure(i,varargin)
fighandle=[0,0,0];
fighandle(1)=figure(i);clf;
set(gcf,'Position',get(0,'ScreenSize'),'color','w')
set(gcf,'renderer','opengl')
fighandle(2)=subplot(2,1,1)
grid on;
hold on;

if length(varargin)>1
    xlim=varargin{1};
    ylim=varargin{2};
else
    xlim=[-50,50];
    ylim=[0,2];
end
axis([xlim,ylim])
plot(xlim, [ylim(1):ylim(2);ylim(1):ylim(2)],'k')

if length(varargin)>2
    daspect(varargin{3});
else
    daspect([5,1,1])
end

%% initialize the control plot
fighandle(3) = subplot(2,1,2);
hold on;
grid on;
axis equal
axis([-1 1 -1 1]);

plot([-1 1],[0 0],':b','linewidth',4)
plot([0 0],[-1 1],':b','linewidth',4)

end