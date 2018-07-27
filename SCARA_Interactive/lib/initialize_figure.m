function initialize_figure(i)
fighandle = figure(i);
clf;
set(gcf,'Position',get(0,'ScreenSize')/2,'color','w')
set(gcf,'renderer','opengl') 
axis equal
axis([-0.5 0.5 -0.5 0.5]);
%grid on;
box on;
hold on;
xlabel('m');
ylabel('m');
end
