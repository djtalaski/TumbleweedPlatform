 clear all
 close all
 clc
    
    
delete(instrfindall);

lost=0;
i=1;
s=serial('COM4');
set(s,'BaudRate',19200);
fopen(s);

for m=1:1000000     %Put a very high number
    
dat{m,1}=fscanf(s);
%temp=strsplit(dat{i,1});
%if length(temp)==11
%C(i,:)=strsplit(dat{i,1});
%else 
%lost=lost+1;
%i=i-1;
end
% k(i,1)=C(i,1);
% i=i+1;
%C=cell2mat(dat{i});
%end
fclose(s);
%k=C(:,1);
%l=C(:,3);

%  for n=3:2000
%      time_ms(n,1)=str2num(C{n,1});
%      roll_deg(n,1)=str2num(C{n,3});
%      position(n,1)=str2num(C{n,5});
%     % error(n,1)=str2num(C{n,7});
%      PWM(n,1)=str2num(C{n,9});
%      No_rot(n,1)=str2num(C{n,10});
%  end
% end
 