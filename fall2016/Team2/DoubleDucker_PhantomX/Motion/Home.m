function [phantomX] = Home(phantomX)

phantomX.q_des = [0;0;0;0];
phantomX.q = phantomX.q_des;

[R0T, p0T, J0T ] = phantomX_ForwardKinematics(phantomX);

phantomX.R0T = R0T;
phantomX.p0T = p0T;
phantomX.J0T = J0T;

phantomX.q = smoothMotion(phantomX, p0T);

