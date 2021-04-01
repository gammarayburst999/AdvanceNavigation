%Author: <Satish Chandra Verma >, Z5220045
%Implement a function for estimating the attitude of the platform, based on the measurements of the IMU’s
%gyroscopes.
%the platform is always operating in a 2D context
%the pitch and roll are always =0
%Program: 
load('IMU_dataC.mat');
i2=1;
skip2=1;
N2=IMU.N;
t2_f=[];
number_of_sample=[];
while 1
if i2>N2, break ;  end
 t2=0.0001*double(IMU.times(i2)-IMU.times(1));
 t2_f=[t2_f;t2];
 number_of_sample=[number_of_sample;i2];
 i2=i2+skip2;
end
 scan_i_02 = IMU.DATAf(6,:);
 MyProcessingOfAttitude(scan_i_02,number_of_sample,t2_f);
 Initial_condition=90;
function MyProcessingOfAttitude(scan_02,number_of_sample,t2_f)
biasWz = mean(scan_02(1:2500));
Initial_condition=90;
Q1 = Initial_condition+((180/pi)* 0.0001*50*cumtrapz(scan_02)); %integration without bias
Q2 = Initial_condition+((180/pi)* 0.0001*50*cumtrapz(scan_02-biasWz));
Q3 = ((180/pi)* 0.0001*50*cumtrapz(scan_02,t2_f));
figure(1)
plot(number_of_sample,Q1,'r')
hold on
plot(number_of_sample,Q2,'b')
xlabel('Sample')
ylabel('heading(degrees)')
title('yaw rate integrated')
legend('corrected','biased')
grid on
hold off
end
