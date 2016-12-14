close(figure(1))
figure(1)
hold on
for j=1:size(path,1)
    plot(path(j,:))
end
legend('x','y','z')
xlabel('Iteration')
ylabel('Position')
phantomTest=phantomX;
pvals=zeros(3,length(path));
% for k=1:length(path)
%     %phantomTest.q=[step2rad(path(1,k));step2rad(path(2,k));step2rad(path(3,k));path(4,k)];
%     phantomTest.q=step2rad([path(1,k);path(2,k);path(3,k);path(4,k)]);
%     [R,P,J]=phantomX_ForwardKinematics(phantomTest);
%     pvals(:,k)=P;
% end
close(figure(2))
figure(2)
hold on
plot3(path(1,:),path(2,:),path(3,:),'ko')
xlabel('x')
ylabel('y')
zlabel('z')
close(figure(3))
figure(3)
hold on
for j=1:size(posepath,1)
    plot(posepath(j,:))
end
