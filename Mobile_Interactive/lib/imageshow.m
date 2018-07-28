clc
clf
clear all
imaqmem(30000000);
%ADAPTOR:MATLAB
hard = imaqhwinfo;
name = hard.InstalledAdaptors
vid = videoinput(name{2});
start(vid);
h = figure('NumberTitle','off','Name','??',...
    'Position',[0,0,1,1],'Visible','on');
set(h,'doublebuffer','on','outerposition',get(0,'screensize'));
%h1=axes('Position',[0.3,0.1,0.4,0.8],'Parent',h);
hold on;
axis off;
while ishandle(h)
    a = getsnapshot(vid);
    flushdata(vid);
    imshow(a);    
    drawnow;
end
delete(vid);
    