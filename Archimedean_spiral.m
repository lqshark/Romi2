a = 3;
b = 0.8;
% y = (a + b * theta) * sin(theta);
% plot(x,y)
x = zeros(1, 100 * 100 * 4);
y = zeros(1, 100 * 100 * 4);
for theta = 1 : 100*100*4
    x(1, theta) = (a + b * (theta-1) / 100) * cos((theta-1) / 100);
    y(1, theta) = (a + b * (theta-1) / 100) * sin((theta-1) / 100);
end

plot(x,y)
xlim([-50 50])
ylim([-50 50])