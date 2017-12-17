function [ f ] = getqp_f( dq, er, ep )

f = -2*[zeros(1,length(dq)),er,ep]';

end

