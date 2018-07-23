pos3d = [bPos3DInterpolated';mPos3DInterpolated'];
pos3d = reshape(pos3d,[3 size(bPos3DInterpolated,1)*2]);
for i=1:2:size(pos3d,2)
    idx = ceil(i/2);
    dist3d(idx) = norm(pos3d(1:3,i)-pos3d(1:3,i+1));
    if idx == 1
        babySpeed(idx) = 0;
        momSpeed(idx) = 0;
    else
        babySpeed(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i)-pos3d(1:3,i+1-2));
        momSpeed(idx) = dist3d(idx - 1) - norm(pos3d(1:3,i-2)-pos3d(1:3,i+1));
    end
end
babyApproach = babySpeed > 5;

figure;
hold on;
axis([0 length(dist3d) 0 3500])
xlabel('Time','fontsize',24,'fontweight','b');
ylabel('Infant-Mother Distance (mm)','fontsize',24,'fontweight','b');
plot(dist3d, 'k', 'LineWidth',3)

% highlight approach
plot(find(babyApproach), dist3d(babyApproach), 'g+', 'LineWidth',5)
hold off;

figure;
hold on;
axis([0 length(dist3d) 0 3500])
xlabel('Time','fontsize',24,'fontweight','b');
ylabel('Infant-Mother Distance (mm)','fontsize',24,'fontweight','b');
plot(dist3d, 'k', 'LineWidth',3)

isBabyCloseToMom = dist3d<600;
plot(find(isBabyCloseToMom), dist3d(isBabyCloseToMom), 'r+', 'LineWidth',5)