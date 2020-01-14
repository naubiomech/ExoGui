function [right_torque,left_torque,RL_mean_time,LL_mean_time]=averaging_torque(data_name)
A=importdata(data_name);%GO_Auto_19-Feb-2019_Trial_Number__3');
%time=A.data(:,2);
RLTRQ=A(:,2);
RL_State=A(:,3);
LLTRQ=A(:,4);
LL_State=A(:,5);
%n=length(time);


[left_torque,LL_mean_time]=averaging(LL_State,LLTRQ,'left');

[right_torque,RL_mean_time]=averaging(RL_State,RLTRQ,'right');
end

function [averaged_torque,mean_time]=averaging(state,torque,mode) 
n = length(state);
te = [];
ts = [];
te2 = [];
ts2 = [];

k1=0;
for i=1:n-1
    if (i==1&&state(i)==3)
        k1=k1+1;
        ts(k1)=i;
    end
    if (state(i)~=3&&state(i+1)==3)
        k1=k1+1;
        ts(k1)=i+1;
    end 
    if (state(i)==3&&state(i+1)~=3)
        te(k1)=i;
    end
    if (state(i+1)==3&&i==n-1)
        te(k1)=i+1;
    end
end
S1=k1;


mean_time=mean(te-ts);
iqr_time=iqr(te-ts);

k2=0;
for i=1:S1
    if abs((te(i)-ts(i))-mean_time)<=1.5*iqr_time
        k2=k2+1;
        ts2(k2)=ts(i);
        te2(k2)=te(i);
    end
end
S2=k2;

%scaling the time duration to the interval of [0,100]
trq_a=zeros(S2,101);
if S2 > 0

if 0 < max(te2-ts2)&& max(te2-ts2) <= 100
for i=1:S2
trq_temp=torque(ts2(i):te2(i));
x_new=(0:1:te2(i)-ts2(i))*100/(te2(i)-ts2(i));
trq_a(i,:)=interp1(x_new,trq_temp,0:1:100,'spine');
end

end
end

rms_torque=zeros(1,S2);
for i=1:S2
    rms_torque(1,i)=sqrt(mean((trq_a(i,:)-mean(trq_a)).^2));
end

mean_rms_torque=mean(rms_torque);
iqr_rms_torque=iqr(rms_torque);

k3=0;
trq_a2 = [];

for i=1:S2
    if abs(rms_torque(1,i)-mean_rms_torque)<=1.5*iqr_rms_torque
        k3=k3+1;
        trq_a2(k3,:)=trq_a(i,:);        
    end
end

S3=k3;

averaged_torque=zeros(1,101);
if S3 > 0
for i=1:101
    averaged_torque(1,i)=mean(trq_a2(:,i));
end
end

% if mode == 0
%     name = 'left';
% elseif mode == 1
%     name = 'right';   
% else
%     return;
% 
% end
if size(trq_a2,1)>0
figure();
plot(0:100,trq_a2);
hold on;
plot(0:100,averaged_torque,'linewidth',2);
hold off;
title(['Averaged ', mode, ' torque profile']);
end
end