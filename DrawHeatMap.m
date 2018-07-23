function DrawHeatMap(I, heatmap)

heatmap = heatmap .* (heatmap > 0);
imshow(I);
hold on;
hImg = imshow(heatmap);
set(hImg, 'AlphaData', 0.5);
colormap Hot;
hold off;
drawnow;

end