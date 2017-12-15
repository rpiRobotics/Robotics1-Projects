%% Function to generate distribution of points for each bin
% Dark points get distribution at middle and quarter edges
function gp=gtpts(avg,i,i1,n,n1)
    gp=[];
    if avg<=230 && avg>=150
          gp=[(i1+i)/2 (n1+n)/2];
%         gp=[round((i1-i).*rand(1,1)+i) round((n1-n).*rand(1,1)+n)];
    elseif avg<150 && avg>=50
          gp=[(i1+i)/2 n1/4+n*3/4; (i1+i)/2 n1*3/4+n1/4];
%         gp=[round((i1-i).*rand(2,1)+i) round((n1-n).*rand(2,1)+n)];
    elseif avg<50
          gp=[(i1+i)/2 (n1+n)/2; i1*3/4+i/4 n1*3/4+n1/4;i1*3/4+i/4 n1/4+n1*3/4;i1/4+i*3/4 n1*3/4+n1/4; i1/4+i*3/4 n1/4+n1*3/4];
%         gp=[round((i1-i).*rand(15,1)+i) round((n1-n).*rand(15,1)+n)]; 
    end
end