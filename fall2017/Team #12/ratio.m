function [points,multi] = ratio(M,binX, binY)
    points = zeros(2,0);
    multi = 1;
    while(1)
        newM = M;
        disp(newM);
        if multi > 1
            for x = 1:multi
                newM = [newM ; M]; 
                disp(newM);
            end
        end
        [newpoints,N] = squarepack(newM,binX,binY);
        if ~isempty(N)
            points = newpoints;
            break
        end
        multi = multi + 1;
        
    end
  
        