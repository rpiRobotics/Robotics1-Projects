% Run this AFTER the Robot Raconteur server is started
robot = RobotRaconteur.Connect('tcp://10.13.215.145:10001/phantomXRR/phantomXController');
%lets write stuff
disp('lets write stuff  we have the letters A,E,K,L,N,X to work with')
pause(0.5);
%robot.setJointPositions(int16([500;500;500;500;500]));
pause(0.5)
%robot.setJointPositions(int16([500;500;500;500;1000]));
%trossen_calibrate(robot);
%pcal=[0 0 1.5 0 0; 0 0 0 0 0; 5 0 6 6.25 4.5];
%phi=0;
load('Letters.mat');
load('Parameters.mat');
poff=input('define top left corner of page [x;y;z]');
pause(0.5);
words=input('what do you want to write','s');
scale=input('height of letters in inches= ');
precision=input('precision=');
s=length(words);
r=-scale;
c=0;
mapping=0;
for t=1:1:s;
    
   if words(s)=='A'
       A=length(LetterA);
      letter=zeros(3,A);
      letter(1,1:A)=c + poff(1);
      letter(2,1:A)=r + poff(2);
      letter(3,1:A)= poff(3);
      letter= letter + LetterA;
      letter=scale*letter;
      draw(letter, precision ,robot);
      c=c+scale;
      
   end
   if words(s)=='E'
      E=length(LetterE);
      letter=zeros(3,E);
      letter(1,1:E)=c + poff(1);
      letter(2,1:E)=r + poff(2);
      letter(3,1:E)= poff(3);
      letter= letter + LetterE;
      letter=scale*letter;
      draw(letter, precision, robot);
      c=c+scale;
       
   end
   if words(s)=='K'
      K=length(LetterK);
      letter=zeros(3,K);
      letter(1,1:K)=c + poff(1);
      letter(2,1:K)=r + poff(2);
      letter(3,1:K)= poff(3);
      letter= letter + LetterK;
      letter=scale*letter;
      draw(letter, precision, robot);
      c=c+scale;
       
   end
   if words(s)=='L'
      L=length(LetterL);
      letter=zeros(3,L);
      letter(1,1:L)=c + poff(1);
      letter(2,1:L)=r + poff(2);
      letter(3,1:L)= poff(3);
      letter= letter + LetterL;
      letter=scale*letter;
      draw(letter, precision, robot);
      c=c+scale;
       
   end
   if words(s)=='N'
      N=length(LetterN);
      letter=zeros(3,N);
      letter(1,1:N)=c + poff(1);
      letter(2,1:N)=r + poff(2);
      letter(3,1:N)= poff(3);
      letter= letter + LetterN;
      letter=scale*letter;
      draw(letter, precision, robot);
      c=c+scale;
       
   end
   if words(s)=='X'
      X=length(LetterX);
      letter=zeros(3,X);
      letter(1,1:X)=c + poff(1);
      letter(2,1:X)=r + poff(2);
      letter(3,1:X)= poff(3);
      letter= letter + LetterX;
      letter=scale*letter;
      draw(letter, precision, robot);
      c=c+scale;
   end
   if words(s)==' '
       r=r-scale;
       c=0;
   end
end
       