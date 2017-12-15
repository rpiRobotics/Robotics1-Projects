function m=dist_rem(c,d)

% Compares points within a matrix against other points
% If point separation is less/equal to d, "larger" point retained

r=[];
for i=1:1:size(c,1)
    for n=1:1:size(c,1)
        if i~=n
            if sqrt((c(i,1)-c(n,1))^2+(c(i,2)-c(n,2))^2)<=d
                if c(i,1)+c(i,2) >= c(n,1)+c(n,2)
                    r=[r;i];
                else
                    r=[r;n];
                end
            end
        end
    end
end

% Ensuring no duplicates and appending coordinates to return next closest
% point
r=unique(r,'rows');
m=[];
for i=1:1:size(c,1)
    if ~ismember(i,r)
        m=[m;c(i,1) c(i,2)];
    end
end

end