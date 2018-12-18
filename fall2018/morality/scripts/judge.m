% Test judgement
function lane = judge(heuristic, obstacles)
    client = rossvcclient('/judgement/judge');
    request = rosmessage(client);
    request.heuristic = heuristic;
    obst = rosmessage('duckietown_msgs/ObstacleProjectedDetection');
    obsts = [];
    for o = obstacles
        obst.location.x = o;
        obsts = [obsts obst];
    end
    request.obstacles.list = obsts;
    response = call(client, request);
    lane = response.lane;
end