function [x_dst, y_dst] = distort(x, y, k1, k2, p1, p2)

rsq = x.*x + y.*y;

x_dst = x.*(1 + k1*rsq + k2*rsq.^2) + 2*p1*x.*y + p2*(rsq + 2*x.^2);
y_dst = y.*(1 + k1*rsq + k2*rsq.^2) + p1*(rsq + 2*y.^2) + p2*x.*y;

end