function [x, y] = idBox(c,r)
newbb1 = [c(1)-r, c(2)-r];
newbb3 = [c(1)+r, c(2)+r];
x = [newbb1(1), newbb3(1), newbb3(1), newbb1(1), newbb1(1)];
y = [newbb1(2), newbb1(2), newbb3(2), newbb3(2), newbb1(2)];
plot(x, y, 'b-', 'LineWidth', 3);
hold on;