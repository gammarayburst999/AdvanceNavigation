%Author: <Satish Chandra Verma >

%Dead-reckoning” process, based on the kinematic model of the platform and
%the measurements provided by sensors (speed encoder and gyroscope).
%Inputs of the process model are the angular rate and the speed encoder measurements.
%%%PART D
load('IMU_dataC.mat');
load('Speed_dataC.mat');
skip=1;
N=Vel.N;
N2=N;
i=1;
skip=1;

t_f=[];
Pose=zeros(3,N);
X0 = [0;0;pi/2] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
X=X0;
while 1
    if i>N
        break;
    end
tLastPrediction = 0.0001*double(Vel.times(1));
t = 0.0001*double(Vel.times(i));
dt = t-tLastPrediction;
tLastPrediction=t;
t_f=[t_f,dt];
i=skip+i;
end
Dt=0.005;
scan_02 = IMU.DATAf(6,:);
biasWz = mean(scan_02(1:2500));
Q2=scan_02-biasWz;
Q3=Q2*(180/pi);
speed = Vel.speeds(1,:)';
steering = -Q2;
 for i=1:N %iterations / Loop
X = PredictVehiclePose(X, steering(i), speed(i) ,Dt);
Pose(:, i)=X ;
if i==N
    final=X;
    final_pose=Pose;
end
 end

% Show the results.
figure(1) ;
plot(Pose(1,:),Pose(2,:),'blue') ;
hold on
grid on
plot(0,0,'+r','MarkerSize',15)
plot(final(1),final(2),'*g','MarkerSize',15)
legend('Trajectory','Start Point','End Point')
xlabel('X (m)'); ylabel('Y (m)'); title('Position');
hold off
figure(2)
plot(t_f,Q3)
hold on
grid on
xlabel('time(seconds)'); ylabel('Wz (degree/second)');
title('Yaw Rate');
hold off
figure(3)
plot(t_f,speed)
hold on
grid on
xlabel('time(seconds)'); ylabel('Speed (m/second)');
title('Longitudinal Velocity');
hold off
 function X = PredictVehiclePose(X0,steering,speed,dt)
% Remember: state vector X = [x; y; heading]
X=X0 ;
dL = dt*speed ;
X(3) = X0(3)+dt*steering;
X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
 return
 end
