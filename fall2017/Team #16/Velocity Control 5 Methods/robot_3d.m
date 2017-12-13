function [pp,RR] = robot_3d(q_tgt)
% tic
%% Robot zero configuration parameters
[~,~,~,n,P,~,H,type] = robotParams();
%% Robot forward kinematics
[pp,RR]=fwdKin_alljoints(q_tgt,type,H,P,n);

%% plots
sle = 0.15*max(sum(P,2));
sle2 = sle*1.4;
schema_chain = zeros(3,4);

% lHandle = line(nan, nan, nan);      
% set(lHandle, 'XData', pp(1,:), 'YData', pp(2,:), 'ZData',pp(3,:), 'Color', [0,0,1]);
plot3(pp(1,:),pp(2,:),pp(3,:), 'b.')
hold on
for i = 1:n+1    
    if(i<=n)
        i_h = [1,2,3]*H(:,i);
        h_i = H(:,i);
        if i == n
            h_i_1 = H(:,n);
        else
            h_i_1 = H(:,i+1);
        end
        %plot schematics
        vec_i = RR(:,:,i)'*(pp(:,i+1)-pp(:,i)); 
        schema_chain = zeros(3,4);
        schema_chain(:,1) = pp(:,i);
        schema_chain(:,2) = pp(:,i)+RR(:,:,i)*(h_i'*vec_i*h_i);
        h_temp = [1,1,1]'-sign(h_i+h_i_1);
        ct_temp = 3;
        for j = 1:3
            if h_temp(j) == 1
                v_temp = zeros(3,1);
                v_temp(j) = 1;
                schema_chain(:,ct_temp) = schema_chain(:,ct_temp-1)+RR(:,:,i)*(v_temp'*vec_i*v_temp);
                ct_temp = ct_temp + 1;
            end
        end
        schema_chain(:,ct_temp) = pp(:,i+1);
        
        lHandle = line(nan, nan, nan);      
        set(lHandle, 'XData', schema_chain(1,:), 'YData', schema_chain(2,:), 'ZData',...
            schema_chain(3,:), 'Color', [0,0,0]);
%         lHandle = line(nan, nan, nan);  
%         set(lHandle, 'XData', schema_chain(1,1:2), 'YData', schema_chain(2,1:2), 'ZData',...
%             schema_chain(3,1:2), 'Color', [0.8549,0.1725,0.2627], 'LineWidth', 3);%(log(8-i)+1)*3);
        %plot rotating axes
        lHandle = line(nan, nan, nan);      
        set(lHandle, 'XData', [pp(1,i)-1/2*sle2*RR(1,i_h,i),pp(1,i)+1/2*sle2*RR(1,i_h,i)], 'YData',...
            [pp(2,i)-1/2*sle2*RR(2,i_h,i),pp(2,i)+1/2*sle2*RR(2,i_h,i)], 'ZData', ...
            [pp(3,i)-1/2*sle2*RR(3,i_h,i),pp(3,i)+1/2*sle2*RR(3,i_h,i)], 'Color',...
            [1,0.8431,0],'LineWidth', 3); 
        %plot all axes
        lHandle = line(nan, nan, nan);
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,1,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,1,i)], 'ZData', [pp(3,i),pp(3,i)+sle*RR(3,1,i)], 'Color', [1,0,0]);

        lHandle = line(nan, nan, nan); 
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,2,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,2,i)], 'ZData', [pp(3,i),pp(3,i)+sle*RR(3,2,i)], 'Color', [0,1,0]);

        lHandle = line(nan, nan, nan);
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,3,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,3,i)], 'ZData', [pp(3,i),pp(3,i)+sle*RR(3,3,i)], 'Color', [0,0,1]);
    else 
        %plot axes in xyz->rgb sequence
        lHandle = line(nan, nan, nan);
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,1,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,1,i)],...
            'ZData', [pp(3,i),pp(3,i)+sle*RR(3,1,i)], 'Color', [1,0,0]);

        lHandle = line(nan, nan, nan); 
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,2,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,2,i)],...
            'ZData', [pp(3,i),pp(3,i)+sle*RR(3,2,i)], 'Color', [0,1,0]);

        lHandle = line(nan, nan, nan);
        set(lHandle, 'XData', [pp(1,i),pp(1,i)+sle*RR(1,3,i)], 'YData', [pp(2,i),pp(2,i)+sle*RR(2,3,i)],...
            'ZData', [pp(3,i),pp(3,i)+sle*RR(3,3,i)], 'Color', [0,0,1]);
    end
end

% axis([-1.5 3.2 -1.5 3.2 -1.5 3.2]) 
% toc
end