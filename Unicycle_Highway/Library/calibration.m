function [cursor_pos_center,cursor_pos_URcorner]=calibration(p)
calibmark.xy = [0,0];
calibmark.handle = plot(calibmark.xy(1),calibmark.xy(2),'o','linewidth',3,'color','r','markersize',14);
set(calibmark.handle,'XDataSource','calibmark.xy(1)');
set(calibmark.handle,'YDataSource','calibmark.xy(2)');

%set(fighandle, 'currentaxes')
%set(text2handle,'string','Please calibrate cursor')

calibmark.xy = p;
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_URcorner = get(0,'PointerLocation');

calibmark.xy = [0,0];
refreshdata([calibmark.handle],'caller');
drawnow;
pause(3)
cursor_pos_center = get(0,'PointerLocation');

calibmark.xy = [-100; -100];
refreshdata([calibmark.handle],'caller');