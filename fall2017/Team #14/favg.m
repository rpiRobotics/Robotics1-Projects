% Calculate average 
function avg=favg(M)
    cm=mean(M);
    avg=round(sum(cm)/size(cm,2));
end