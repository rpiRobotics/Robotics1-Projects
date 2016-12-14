function H=translate(h,d)

H=[eye(3,3) d*h;zeros(1,3) 1];
