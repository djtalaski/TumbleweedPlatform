% Do not clear workspace
% run after Data_tester is executed
clc
%% Clearing broken values and estimating lost lines of data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lost=0;

% % "place" used to store the index of null variable rows

for k=1:length(dat)    %Put a very high number
temp=strsplit(dat{k});
if length(temp)==14
C(k,:)=strsplit(dat{k});
else 
lost=lost+1;
place(lost)=k;
% k=k-1;
end
% k(i,1)=C(i,1);
% i=i+1;
%C=cell2mat(dat{i});
end

% % Fopr loop to subtract "place" values by their index (0 based). A pre requisite to
% remove them one after another in the next for loop
c_term=0;
 for g=1:length(place)
     place(g)=place(g)-c_term;
      C(place(g),:)=[];
     c_term=c_term+1;
 end
 
 disp('Lost no of data lines=');
 disp(lost);
 
%% Final Preprocessing and Data Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Converting Cell array elements to double type for handling and plotting
co=0;
  for n=1:length(C)
      time_ms(n,1)=str2num(C{n,1});
      roll_deg(n,1)=str2num(C{n,3});
      position(n,1)=str2num(C{n,5});
%        omega(n,1)=str2num(C{n,7});
%     % error(n,1)=str2num(C{n,7});
%      PWM(n,1)=str2num(C{n,9});
%       No_rot(n,1)=str2num(C{n,13});
%  end
  end
  
  for n=2:length(C)
      if abs(omega(n))>1000
          omega(n)=omega(n-1);
          co=co+1;
      end
  end
  
 % omega
 rollold=0;
 told=0;
 
% % % % % % % To Find omega from theta (if omega value is not relayed from
% sensor directly)

%  for p=1:length(C)
%      
%  om(p)=1000*(roll_deg(p)-rollold)/(time_ms(p)-told);
%  rollold=roll_deg(p);
%  told=time_ms(p);
%  
%  end
%  
%  figure(1)
%  plot(time_ms,position)
%  
%  figure(2)
%  plot(time_ms,roll_deg)
%  
%  figure(3)
%   plot(time_ms,om)
%   
%   ylim([-0.5 200])


% % % % % Manually enter range to avoid plotting end values that are not
% necessary
figure(1)
plot(time_ms(1:3463,1),-omega(1:3463,1))
xlabel('Time (ms)');
ylabel('Angular Velocity (deg/s)');
title('Angular Velocity vs Time');

figure(2)
plot(time_ms(1:3463,1),position(1:3463,1))
xlabel('Time (ms)');
ylabel('Position (cm)');
title('Mass Position vs Time');

figure(3)
plot(time_ms(1:3463,1),No_rot(1:3463,1))
xlabel('Time (ms)');
ylabel('No. of Rotations');
title('Mass Position vs Time');
 