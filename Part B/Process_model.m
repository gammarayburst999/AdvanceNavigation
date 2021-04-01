%Author: <Satish Chandra Verma>
%Applying EKF for map-based robot localization
%Using a proper Q matrix, based on the assumed noise in the inputs of the process model

%Program: Solution for AAS, T1.2020, Project2.part 1 B
function main()
stdDevGyro = 2*pi/180 ;
stdDevSpeed = 0.15 ;   % We simulate a lot of error!  (very difficult case).
sdev_rangeMeasurement = 0.25 ;          % std. of noise in range measurements. 0.25m
Dt=0.05 ;                       % "sample time", 50ms
Li = 1800 ;                     % "experiment" duration, in iterations
DtObservations=0.250 ;          % laser "sample time" (for observations), 4Hz, approximately
n_usedLanmarks = 5 ;    %it must be : 0 < n_usedLanmarks <5, but you can modify the function "CreateSomeMap()" to extend it.
global NavigationMap;
NavigationMap = CreateSomeMap(n_usedLanmarks) ;  %creates a artificial map!
Xe = [ 0; 0;pi/2 ] ;
P = zeros(3,3) ;            % initial quality --> perfect (covariance =zero )
Xdr = [ 0; 0;pi/2 ] ;
Xreal_History= zeros(3,Li) ;
Xe_History= zeros(3,Li) ;
XeDR_History= zeros(3,Li) ;
Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
time=0 ;
InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,DtObservations);
clc();
disp('Running full simulation, OFF line. Please wait...');
for i=1:Li     % loop
   % pause(0.01) ;
    time = time+Dt ;
    SimuPlatform(time,Dt);      % because NOW I do not have a real system, I simulate it.
    [Noisy_speed,Noisy_GyroZ]=GetProcessModelInputs();
    Xdr    = RunProcessModel(Xdr,Noisy_speed,Noisy_GyroZ,Dt) ;
    J = [ [1,0,-Dt*Noisy_speed*sin(Xe(3))  ]  ; [0,1,Dt*Noisy_speed*cos(Xe(3))] ;    [ 0,0,1 ] ] ;
    T=Dt;
    SDv=0.25;
    SDw= 5*pi/180;
    Fu=[T*cos(Xe(3)) 0;T*sin(Xe(3)) 0; 0 T];
    Pu=[SDv^2 0; 0 SDw^2];
    P = J*P*J'+Fu*Pu*Fu'+Q ;
    Xe    = RunProcessModel(Xe,Noisy_speed,Noisy_GyroZ,Dt) ;
    [nDetectedLandmarks,MasuredRanges,measuredAngle,IDs]=GetObservationMeasurements();
    if nDetectedLandmarks>0     % any laser data and detected landmarks?
        for u=1:nDetectedLandmarks
            ID = IDs(u);            % landmark ID?    (in "real life", this is provided by the "data association")
            eDX = (NavigationMap.landmarks(ID,1)-Xe(1)) ;      % (xu-x)
            eDY = (NavigationMap.landmarks(ID,2)-Xe(2)) ;      % (yu-y)
            eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 )
            eDD2=eDX*eDX + eDY*eDY ;
            H1 = [  -eDX/eDD , -eDY/eDD , 0 ] ;   % Jacobian of h(X); size 1x3
            H2 =[  eDY/eDD2 , -eDX/eDD2 , -1 ] ;
            H=[H1;H2];
            ExpectedRange = eDD ;   % just a coincidence: we already calculated them for the Jacobian, so I reuse i                  % Evaluate residual (innovation)  "Y-h(Xe)"

              angle_e=eDY./eDX;
              phi_02=Xe(3);
              exp_angle= atan(angle_e)-phi_02+pi/2;
            z1  = MasuredRanges(u) - ExpectedRange ;
            z2= measuredAngle(u) -exp_angle;
            z=[z1;z2];
            R1 = sdev_rangeMeasurement*sdev_rangeMeasurement*4 ;
            R2= stdDevGyro*stdDevGyro*0.45;
            R=[R1 0; 0 R2];
            S = R + H*P*H' ;
            iS=inv(S);                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
            K = P*H'*iS ;           % Kalman gain
            Xe = Xe+K*z ;       % update the  expected value
            P = P-K*H*P ;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
        end
    end
     Xreal_History(:,i) = GetCurrentSimulatedState() ;
     Xe_History(:,i)    = Xe ;
     XeDR_History(:,i)  = Xdr ;
end                           % end of while loop
fprintf('Done. Showing results, now..\n');
SomePlots(Xreal_History,Xe_History,XeDR_History,NavigationMap) ;
return ;
function Xnext=RunProcessModel(X,speed,GyroZ,dt)
    Xnext = X + dt*[ speed*cos(X(3)) ;  speed*sin(X(3)) ; GyroZ ] ;
return ;
function [ranges,angle,IDs] = GetMeasurementsFomNearbyLandmarks(X,map) %Changes made here

    if map.nLandmarks>0
        dx= map.landmarks(:,1) - X(1) ;
        dy= map.landmarks(:,2) - X(2) ;
        ranges = sqrt((dx.*dx + dy.*dy)) ;
        angle1=dy./dx;
        phi=X(3);
        angle= atan(angle1)-phi+pi/2;
        IDs = [1:map.nLandmarks];
    else
        IDs=[];ranges=[];angle=[];
    end
return ;%
function [speed,GyroZ] = SimuControl(X,t)
    speed = 2 ;                                         % cruise speed, 2m/s  ( v ~ 7km/h)
    GyroZ = 3*pi/180 + sin(0.1*2*pi*t/50)*.02 ;         % some crazy driver moving the steering wheel...
return ;
function map = CreateSomeMap(n_used)
    n_used = max(0,min(4,n_used));      % accepts no less than 1, no more than 4.
    landmarks = [  [ -40,0 ];[ -0,-20 ];[ 10,10 ] ;[ 30,10 ]  ] ;
    map.landmarks = landmarks(1:n_used,:) ;
    map.nLandmarks = n_used ;
return ;
function InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,DtObservations)
    global ContextSimulation;
    ContextSimulation.Xreal = [ 0; 0;pi/2 ] ;     % [x;y;phi]
    ContextSimulation.stdDevSpeed = stdDevSpeed;
    ContextSimulation.stdDevGyro = stdDevGyro;
    ContextSimulation.Xreal = [0;0;pi/2];
    ContextSimulation.speed=0;
    ContextSimulation.GyroZ=0;
    ContextSimulation.sdev_rangeMeasurement=sdev_rangeMeasurement;
    ContextSimulation.DtObservations=DtObservations;
    ContextSimulation.timeForNextObservation= 0;
    ContextSimulation.CurrSimulatedTime=0;
return;
function [Noisy_speed,Noisy_GyroZ]=GetProcessModelInputs()
    global ContextSimulation;
    Noisy_speed =ContextSimulation.speed+ContextSimulation.stdDevSpeed*randn(1) ;
    Noisy_GyroZ =ContextSimulation.GyroZ+ContextSimulation.stdDevGyro*randn(1);
return;
function  SimuPlatform(time,Dt)
    global ContextSimulation;
    [ContextSimulation.speed,ContextSimulation.GyroZ] = SimuControl(ContextSimulation.Xreal,time) ;      % read kinematic model inputs, ideal ones
    ContextSimulation.Xreal = RunProcessModel(ContextSimulation.Xreal,ContextSimulation.speed,ContextSimulation.GyroZ,Dt) ;
    ContextSimulation.CurrSimulatedTime = ContextSimulation.CurrSimulatedTime+Dt;
return;
function [nDetectedLandmarks,MasuredRanges,measuredAngle,IDs]=GetObservationMeasurements(map)
    global ContextSimulation NavigationMap;
    if ContextSimulation.CurrSimulatedTime<ContextSimulation.timeForNextObservation
        nDetectedLandmarks=0;
        MasuredRanges=[];
        measuredAngle=[];
        IDs=[];
        return ;
    end
    ContextSimulation.timeForNextObservation = ContextSimulation.timeForNextObservation+ContextSimulation.DtObservations;
        [RealRanges,angle,IDs] = GetMeasurementsFomNearbyLandmarks(ContextSimulation.Xreal,NavigationMap) ;
        nDetectedLandmarks = length(RealRanges) ;
        if (nDetectedLandmarks<1)       % no detected landmarks...
            MasuredRanges=[];   IDs=[];  measuredAngle=[];  return ;
        end
        noiseInMeasurements= ContextSimulation.sdev_rangeMeasurement*randn(size(RealRanges));
        noiseInangle= ContextSimulation.stdDevGyro*randn(size(angle));
        MasuredRanges = RealRanges +  noiseInMeasurements ;
        measuredAngle=angle+noiseInangle;

 return;
    function X=GetCurrentSimulatedState()
        global ContextSimulation;
        X=ContextSimulation.Xreal;
     return;
% --- This is JUST for ploting the results
function SomePlots(Xreal_History,Xe_History,Xdr_History,map)
figure(2) ; clf ; hold on ;
plot(Xreal_History(1,:),Xreal_History(2,:),'b') ;
plot(Xe_History(1,:),Xe_History(2,:),'r') ;
plot(Xdr_History(1,:),Xdr_History(2,:),'m') ;
if (map.nLandmarks>0),
    plot(map.landmarks(:,1),map.landmarks(:,2),'*r') ;
    legend({'Real path','EKF Estimated path','DR Estimated path','Landmarks'});
else
    legend({'Real path','EKF Estimated path','DR Estimated path'});
end
ii = [1:225:length(Xe_History(1,:))] ;
m=10;
quiver(Xreal_History(1,ii),Xreal_History(2,ii),m*cos(Xreal_History(3,ii)),m*sin(Xreal_History(3,ii)),'b','AutoScale','off','Marker','o' ) ;
quiver(Xe_History(1,ii),Xe_History(2,ii),m*cos(Xe_History(3,ii)),m*sin(Xe_History(3,ii)),'r','AutoScale','off','Marker','+') ;
quiver(Xdr_History(1,ii),Xdr_History(2,ii),m*cos(Xdr_History(3,ii)),m*sin(Xdr_History(3,ii)),'m','AutoScale','off','Marker','o' ) ;
axis equal ;
title('Path') ;
zoom on ; grid on; box on;
% --------- plot errors between EKF estimates and the real values
figure(3) ; clf ;
Xe=Xe_History;
subplot(311) ; plot(Xreal_History(1,:)-Xe(1,:)) ;ylabel('x-xe (m)') ;
title('Performance EKF') ;
subplot(312) ; plot(Xreal_History(2,:)-Xe(2,:)) ;ylabel('y-ye (m)') ;
subplot(313) ; plot(180/pi*(Xreal_History(3,:)-Xe(3,:))) ;ylabel('heading error (deg)') ;
figure(4) ; clf ;
Xe=Xdr_History;
subplot(311) ; plot(Xreal_History(1,:)-Xe(1,:)) ;ylabel('x-xe (m)') ;
title('Performance Dead Reckoning (usually, not good)') ;
subplot(312) ; plot(Xreal_History(2,:)-Xe(2,:)) ;ylabel('y-ye (m)') ;
subplot(313) ; plot(180/pi*(Xreal_History(3,:)-Xe(3,:))) ;ylabel('heading error (deg)') ;
Xe=[];
return ;
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
