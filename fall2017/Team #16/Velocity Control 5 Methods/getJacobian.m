function J = getJacobian(q,type,H,P,n)
num_joints = length(q);

P_0_i = zeros(3,num_joints+1);
R_0_i = zeros(3,3,num_joints+1);

%% Compute Forward Kinematics

[P_0_i,R_0_i]=fwdKin_alljoints(q,type,H,P,n);

P_0_T = P_0_i(:,num_joints+1);

%% Compute Jacobian
J = zeros(6,num_joints);
for i = 1:num_joints
    if type(i) == 0
        J(:,i) = [R_0_i(:,:,i)*H(:,i); hat(R_0_i(:,:,i)*H(:,i))*(P_0_T - P_0_i(:,i))];
    end    
end

end