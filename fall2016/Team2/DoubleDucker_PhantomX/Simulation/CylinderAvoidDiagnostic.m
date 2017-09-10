close(figure(3));
figure(3);
hold on
[X,Y,Z]=cylinder(r*[1;1;1],100);
surf(X+C(1),Y+C(2),Z)
plot3([p1(1) C1(1)],[p1(2) C1(2)],[p1(3) C1(3)])
plot3([C1(1) p0T(1)],[C1(2) p0T(2)],[C1(3) p0T(3)])
plot3(p1(1),p1(2),p1(3),'ko')
plot3(C1(1),C1(2),C1(3),'ro')
plot3(p0T(1),p0T(2),p0T(3),'bo')
box on