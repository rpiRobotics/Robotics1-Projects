%% 
%Test_AR_tag
while 1
    a = duck.april_tags;
    if isempty(a) == 0 && a{1,1}.id == 1 && sqrt(a{1,1}.pos(1)^2+ a{1,1}.pos(2)^2)<= 0.5
        duck.sendCmd(0,0);
        break
    else
        duck.sendCmd(0.2,0.2);
    end
end
