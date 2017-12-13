function h_my_robot=showRobot(q,h_my_robot,fixaxis,viewpoint)

  theta = get_angle_structure(h_my_robot);
  theta.state=q;
  h_my_robot = updateRobot(theta,h_my_robot);
  drawnow;
 
%   axis equal;
  axis(fixaxis);
  view(viewpoint)    
