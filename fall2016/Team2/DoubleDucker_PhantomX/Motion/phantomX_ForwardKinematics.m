function [R0T, p0T, J0T ] = phantomX_ForwardKinematics(phantomX)
% phantomX_ForwardKinematics: computes the forward kinematics and Jacobian for the
%   4 DOF Phantom X Robot Arm.
%
%   Input(s): current joint angles (configuration) from phantomX structure
%   Output(s)= rotation matrix, position vector, Jacobian.(all wrt task)

% *****Note phantomX must be previous defined*****
q = [phantomX.q;0];

% Initialize Recusive Loop for Geometric Jacobian and Forward Kinematics
R=eye(3,3);p=zeros(3,1);Jtr=[];
for i=1:phantomX.n+1
    [R,p,Jtr]=fwdkinstep(R,p,Jtr,q,phantomX.H,phantomX.jointtype...
        ,phantomX.PVect,i);
end

R0T=R; % rotation matrix
p0T=p; % position vector
Jn=Jtr.';
J0T=[R zeros(3,3); zeros(3,3) R]*Jn(:,1:phantomX.n); % Jacobian

end

