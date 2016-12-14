

function [Bset_Match] =  GlobalTracking(gau32CostMatrix)

INF = 100000000;

[gu8CostMSize,n] = size(gau32CostMatrix);
gu8max_match = 0;
gas8xy = -1*ones(1,gu8CostMSize);
gas8yx = -1*ones(1,gu8CostMSize);
gas32LabelX =  min(gau32CostMatrix');
gas32LabelY =  0*min(gau32CostMatrix');
			Queue = zeros(1,gu8CostMSize);
			gau8S = zeros(1,gu8CostMSize);	
			gau8T = zeros(1,gu8CostMSize);	
			gas8prev = zeros(1,gu8CostMSize);	
			gas32Slack = zeros(1,gu8CostMSize);
			gau8SlackX = zeros(1,gu8CostMSize);
											
            
%//Stop while the matching is already done
while(gu8max_match ~= gu8CostMSize)
        
		for u8Loop1 = 1: gu8CostMSize
			Queue(u8Loop1) = 1;
			gau8S(u8Loop1) = 0;	%//init set S
			gau8T(u8Loop1) = 0;	%//init set T
			gas8prev(u8Loop1) = -1;	%//init set prev -1 for the alternating tree
        end
		Delta = 0;
		u8root = 1;
		wr = 1;
		rd = 1;

		%//finding root of the tree
        for x = 1:gu8CostMSize
            if (gas8xy(x) == -1)
				Queue(wr) = x;
                wr = wr+1;
				u8root = x;
				gas8prev(x) = -2;  %//-2 as the end point of tracing a tree
				gau8S(x) = 1;
				break;
            end
        end

		%//initializing slack array
        for y = 1:gu8CostMSize
			gas32Slack(y) = gas32LabelX(u8root) + gas32LabelY(y) - gau32CostMatrix(u8root,y);
			gau8SlackX(y) = u8root;
        end

		while(1)
			while (rd < wr)
				x = Queue(rd);
                rd = rd+1;
                y = 1;
				while( y <= gu8CostMSize)
                    
					if ((gau32CostMatrix(x,y) == gas32LabelX(x) + gas32LabelY(y)) && (~gau8T(y)))
						%//if this match is not occupied, get the match!
						if (gas8yx(y) == -1)
							break;
                        end

						gau8T(y) = 1;
						Queue(wr) = gas8yx(y);
                        wr=wr+1;
						%//--- Add to Tree ---//
						gau8S(gas8yx(y)) = 1;
						gas8prev(gas8yx(y)) = x;
						for ytmp= 1:gu8CostMSize
							if((gas32LabelX(gas8yx(y)) + gas32LabelY(ytmp) - gau32CostMatrix(gas8yx(y),ytmp)) > gas32Slack(ytmp)) 
                            	gas32Slack(ytmp) = gas32LabelX(gas8yx(y)) + gas32LabelY(ytmp) - gau32CostMatrix(gas8yx(y),ytmp);
								gau8SlackX(ytmp) = gas8yx(y);
                            end
						end
						%//--- Add to Tree ---//
                    end
                    y=y+1;
                end
                
                if (y <= gu8CostMSize)
                    break;
                end
            end
            
			if (y <= gu8CostMSize)
				break;
            end

			%//----- Update Labels -------------//
			Delta = -INF;                 
			%//calculate delta using slack
			for ytmp= 1:gu8CostMSize 
				if(~gau8T(ytmp))
					if(Delta < gas32Slack(ytmp))  
						Delta = gas32Slack(ytmp);
                    end
                end
            end
			%//update X labels
			for tmp= 1:gu8CostMSize
				if(gau8S(tmp))
					gas32LabelX(tmp) = gas32LabelX(tmp)-Delta;
                end
				if(gau8T(tmp))
					gas32LabelY(tmp) = gas32LabelY(tmp) + Delta;
                else
					gas32Slack(tmp) = gas32Slack(tmp)-Delta;
                end
            end
			%//----- end of Update Labels ------//

			wr = 1;
            rd = 1;
            y=1;
			while (y<=gu8CostMSize)
				if ((~gau8T(y)) && (gas32Slack(y) == 0))

					if(gas8yx(y) == -1)
						x = gau8SlackX(y);
						break;
                    else
						gau8T(y) = 1;
						if (~gau8S(gas8yx(y)))
							Queue(wr) = gas8yx(y);
                            wr = wr+1;
							%//--- Add to Tree ---//
							gau8S(gas8yx(y)) = 1;
							gas8prev(gas8yx(y)) = gau8SlackX(y);
							for ytmp= 1:gu8CostMSize
								if(( gas32LabelX(gas8yx(y)) + gas32LabelY(ytmp) - gau32CostMatrix(gas8yx(y),ytmp)) > gas32Slack(ytmp))
									gas32Slack(ytmp) = gas32LabelX(gas8yx(y)) + gas32LabelY(ytmp) -gau32CostMatrix(gas8yx(y),ytmp);
									gau8SlackX(ytmp) = gas8yx(y);
                                end
                            end
							%//--- Add to Tree ---//
                        end
                    end
                end
                y=y+1;
            end
			if (y <= gu8CostMSize)
				break;
            end
        end%while(1)

        if (y <= gu8CostMSize)
             gu8max_match = gu8max_match + 1 ;
			%// Inverse edges along augmenting path
            xtmp=x; ytmp=y;
            while (xtmp ~= -2)
				tmp = gas8xy(xtmp);
				gas8yx(ytmp) = xtmp;
				gas8xy(xtmp) = ytmp;
%                 gas8xy-1
                xtmp = gas8prev(xtmp);
                ytmp = tmp;
            end
			%//recall function, go to step 1 of the algorithm
        end
end

% %==========================================================================
Bset_Match = gas8xy;
% Cost = 0;
% for i = 1:gu8CostMSize
%     Cost = Cost+gau32CostMatrix(i,Bset_Match(i));
% end
% Cost
% 
% Bset_Match = Bset_Match-1
% 
% Bset_Match = [5, 8, 7, 9, 4, 1, 2, 3, 6, 11, 10, 0]+1;
% Cost = 0;
% for i = 1:gu8CostMSize
%     Cost = Cost+gau32CostMatrix(i,Bset_Match(i));
% end
% Cost



