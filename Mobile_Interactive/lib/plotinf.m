
figure; 
%subplot(311)
hold on; 
x=agent.x;
plot(x(:,:)');
plot(xfilt_me(:,:)', '-.');
plot(xsmooth_me(:,:)', '--');
legend('true state','filter state','smoothed state')
% subplot(312);
% hold on; 
% plot(x(2,:)');
% plot(xfilt_me(2,:)', '-.');
% plot(xsmooth_me(2,:)', '--');
% legend('true state','filter state','smoothed state')
% subplot(313);
% hold on; 
% plot(x(3,:)');
% plot(xfilt_me(3,:)', '-.');
% plot(xsmooth_me(3,:)', '--');
% legend('true state','filter state','smoothed state')


%%
figure;
hold on
plot(robot.inf.xstar');
plot(agent.x(:,500:504)','--');
legend('estimated x position','estimated x velocity','estimated y position','estimated y velocity',...
    'true x position','true x velocity','true y position','true y velocity')
xlabel('timestep')
