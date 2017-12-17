function J = getJacobian_task_sawyer(q,type,H,P,n,R_EE)
num_joints = length(q);

%% Compute Forward Kinematics

[P_0_i,R_0_i]=fwdKin_alljoints_sawyer(q,type,H,P,n,R_EE);

P_0_T = P_0_i(:,num_joints+1);

%% Compute Jacobian
J = zeros(6,num_joints);
for i = 1:num_joints
    if type(i) == 0
        J(:,i) = [R_0_i(:,:,i)*H(:,i); hat(R_0_i(:,:,i)*H(:,i))*(P_0_T - P_0_i(:,i))];
    end    
end
J = [R_0_i(:,:,num_joints+1)',zeros(3,3);zeros(3,3),R_0_i(:,:,num_joints+1)']*J;

end