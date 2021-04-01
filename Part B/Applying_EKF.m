%Author: <Satish Chandra Verma>, Z5220045

%Program: Solution for AAS, T1.2020, Project2.part 2
function main()

%%
stdDevGyro = 1.5*pi/180 ;        
stdDevSpeed = 0.4 ;   % We simulate a lot of error!  (very difficult case). 
sdev_rangeMeasurement = 0.2 ;          % std. of noise in range measurements. 0.25m
sdev_rangeBearing=1.5;                      % "sample time", 50ms                   % "experiment" duration, in iterations 
DtObservations=0.250 ;          % laser "sample time" (for observations), 4Hz, approximately
%%
% for i=1:1
%%
global ABCD;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.
ABCD.flagPause=0;
ABCD.endnow=0;
ABCD.fe=1;
%%
load('Laser__2.mat');
load('IMU_dataC.mat');
load('Speed_dataC.mat');
%%

%%%%%%%%%%%%%%%%%% LOCAL Cartesian%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
clf();
MyGUIHandles2.handle1 = plot(0,0,'b.');      % to be used for showing the laser points
hold on
axis([-10,10,0,20]);                          % focuses the plot on this region (of interest, close to the robot)
xlabel('X-axis');
ylabel('Y-axis');
MyGUIHandles2.handle2 = title('');
MyGUIHandles2.handle3 =plot(0,0,'+r');
MyGUIHandles2.handle4 =plot(0,0,'+g','MarkerSize',15);
legend('Reflective Pixel','Highly Reflective Pixel','center of OOI')
zoom on ;  grid on;
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%GLOBAL Cartesian%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2);

clf();
MyGUIHandles3.handle1 = plot(0,0,'+r');
hold on
axis([-4,6,0,8]);                          % focuses the plot on this region (of interest, close to the robot)
xlabel('X-axis');
ylabel('Y-axis');
MyGUIHandles3.handle2 = title('');
MyGUIHandles3.handle3 = plot(0,0,'.b','MarkerSize',12);% to be used for showing the laser points
%MyGUIHandles3.handle4 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle5 = plot(0,0,'.g','MarkerSize',12); %DR
MyGUIHandles3.handle6 = plot(0,0,'.k','MarkerSize',12); %kalman
MyGUIHandles3.handle7 = plot(0,0,'.b','MarkerSize',6);
MyGUIHandles3.handle8 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle9 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle10 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle11 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle12 = plot(0,0,'.c','MarkerSize',12);
MyGUIHandles3.handle13 = plot(0,0,'.c','MarkerSize',12);
MyGUIHandles3.handle14 = plot(0,0,'.c','MarkerSize',12);
MyGUIHandles3.handle15 = plot(0,0,'.c','MarkerSize',12);
MyGUIHandles3.handle16 = plot(0,0,'.c','MarkerSize',12);
legend('Landmarks(map)','Robot',' OOIs(projected globally,DR)','OOIs(EKF)')
zoom on ;  grid on;
hold off;

%------------------ GUI ---------------

uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});
uicontrol('Style','pushbutton','String','FE on/off','Position',[170,1,80,20],'Callback',{@MyCallBackA,3});
%--------------------------------------
%%
global spe;
N = dataL.N;
N2=Vel.N;
skip=1;
i=1;
Pose=zeros(3,N2);
X0 = [0;0;pi/2] ;
Dt=0.005;
scan_02 = IMU.DATAf(6,:);
biasWz = mean(scan_02(1:2500));
Q2=scan_02-biasWz;
spe.speed_01 = Vel.speeds(1,:)';
spe.steering = Q2';
%%

 %%
Xe = [ 0; 0;pi/2 ] ; 
P = zeros(3,3) ;            % initial quality --> perfect (covariance =zero )
Xdr = [ 0; 0;pi/2 ] ;
Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
time=0 ;
speed = Vel.speeds(1,:)';
steering = Q2';
InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,sdev_rangeBearing,DtObservations);
clc();
disp('Running full simulation, OFF line. Please wait...');   
while 1 
    if i>1 
    start_point=round(5.3760*(i-1))+1;
    else
        start_point=1;
    end
    end_point=  round(5.3760*(i));
        for i_new=start_point:end_point %iterations / Loop
            Xdr = PredictVehiclePose(Xdr, steering(i_new), speed(i_new) ,Dt);
            Pose=Xdr ;
        end
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     J = [ [1,0,-Dt*speed(end_point)*sin(Xe(3))  ]  ; [0,1,Dt*speed(end_point)*cos(Xe(3))] ;    [ 0,0,1 ] ] ; 
    T=Dt;
    SDv=0.4;
    SDw= 2*pi/180;
    Fu=[T*cos(Xe(3)) 0;T*sin(Xe(3)) 0; 0 T];
    Pu=[SDv^2 0; 0 SDw^2];
    P = J*P*J'+Fu*Pu*Fu'+Q ;
     
          for i_n=start_point:end_point  %iterations / Loop
             Xe = PredictVehiclePose(Xe, steering(i_n), speed(i_n) ,Dt);
             Pose_Xe=Xe;
          end
   if (ABCD.flagPause), pause(0.2) ; continue ; end
    if i>N, break ;  end  
    if end_point>=N2, break ;  end  
    time = time+Dt ; 
   if (ABCD.endnow), break  ; end
    scan_i = dataL.Scans(:,i);
    MyProcessingOfScan(scan_i,MyGUIHandles2,MyGUIHandles3,i,Pose,Pose_Xe,P);   % some function to use the data...   
    pause(0.01) ;                   % wait for ~10ms (approx.)
    i=i+skip; 
    
end                           % end of while loop
fprintf('Done. Showing results, now..\n');
return ;        

function InitSimulation(stdDevSpeed,stdDevGyro,sdev_rangeMeasurement,sdev_rangeBearing,DtObservations)
    global ContextSimulation;
    ContextSimulation.Xreal = [ 0; 0;pi/2 ] ;     % [x;y;phi]
    ContextSimulation.stdDevSpeed = stdDevSpeed;
    ContextSimulation.stdDevGyro = stdDevGyro;
    ContextSimulation.Xreal = [0;0;pi/2];
    ContextSimulation.speed=0;
    ContextSimulation.GyroZ=0;
    ContextSimulation.sdev_rangeMeasurement=sdev_rangeMeasurement;
    ContextSimulation.sdev_rangeBearing=sdev_rangeBearing;
    ContextSimulation.DtObservations=DtObservations;
    ContextSimulation.timeForNextObservation= 0;
    ContextSimulation.CurrSimulatedTime=0;
return;

% 
     %%
function X = PredictVehiclePose(X0,steering,speed,dt)
% Remember: state vector X = [x; y; heading]
X=X0 ;
dL = dt*speed ;
X(3) = X0(3)+dt*steering;
X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return;
  %% 
 
%%
 function MyProcessingOfScan(scan,mh2,mh3,i,position_data,Pose_Xe,P)
global ABCD;
persistent static_obstacles_x;
persistent static_obstacles_y;
global ContextSimulation;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%-------CARTESIAN--------%%%%%%%%%%%%%%%%%%%%%%%%
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ;
intensities_01 = bitand(scan,maskE000); % cartesian
ranges_01    = single(bitand(scan,mask1FFF))*0.01; %cartesian
angles_01 = [0:360]'*0.5* pi/180 ;
X_01 = cos(angles_01).*ranges_01;
Y_01 = sin(angles_01).*ranges_01;
ii = find(intensities_01~=0);
set(mh2.handle1,'xdata',X_01,'ydata',Y_01);
set(mh2.handle3,'xdata',X_01(ii),'ydata',Y_01(ii));

counter=0;
%%%%%%%%%%%%%%%%%
if ii>0
    xx2=X_01(ii);% store the x co-ordinates of the high intensity points
    yy2=Y_01(ii); 
    xx3=[0;xx2]; %just adding extra variable to start
    yy3=[0;yy2];
    ss=length(xx2);
    xx4=[]; % store the x co-ordinates of the high intensity points atleast 20 cm apart
    yy4=[]; % same as above but for y co-ordinates
    %%%%xxxxxx
    for var=1:(ss-1)
        if(abs(xx3(var+1)-xx3(var))>0.2) % over here the threshold is 20 cm
            xx4=[xx4;xx3(var+1)];
        end
        if(abs(yy3(var+1)-yy3(var))>0.2)
            yy4=[yy4;yy3(var+1)];
        end
    end
    number_of_ooi=length(xx4); % number of ooi
    pos=[]; % there position in actual data
    all_center_01=[]; % all center of the high intensity point x
    all_diameter_01=[]; % all diameter
    all_diameter_x01=[];
    all_center_x01=[];
    all_center_y01=[];
    
    %same but for y
    all_center_02=[];
    k_all=[];
    A_x=[];
    A_y=[];
    for var2=1:number_of_ooi
        k= find(xx3==xx4(var2));
        pos=[pos;k];
    end
    
    %%%center
    for var3=1:(length(pos))
        if var3 < length(pos)
            start=pos(var3);
            end2=pos(var3+1)-1;
            A=xx3(start:end2);
            B=yy3(start:end2);
        end
        if var3==length(pos)
            start=pos(var3);
            end2=length(xx3);
            A=xx3(start:end2);
            B=yy3(start:end);
        end
        center=mean(A);
        centery=mean(B);
        diameter_01=(A-center);
        diameter_02=(B-centery);
        d_xy=diameter_01.^2+diameter_02.^2;
        dd_xy=2*sqrt(d_xy);
        max_diameter_01= max(dd_xy);
        all_diameter_01=[all_diameter_01;max_diameter_01];
        all_center_01=[all_center_01;center];
        all_center_02=[all_center_02;centery];
    end
    number_of_center=length(all_center_01);
    ones_xy=ones(1,length(X_01))';
    for var4=1:number_of_center
        d= ((all_center_01(var4)*ones_xy)-X_01).^2 + (all_center_02(var4)*ones_xy-Y_01).^2;
        k5=find(sqrt(d)<0.25);
        if isempty(k5)
            k6=k5;
        else
            k6=[k5;-1];
        end
        k_all=[k_all;k6];
    end
    var7=1;
    %%%%%%%%%%%%%alll new center
    while var7<(length(k_all))
        
        if k_all(var7) == -1
            A_x=[];
            A_y=[];
            var7=var7+1;
        end
        while k_all(var7) ~= -1
            A_x= [A_x;X_01(k_all(var7))];
            A_y= [A_y;Y_01(k_all(var7))];
            var7=var7+1;
        end
        center_x=mean(A_x);
        center_y=mean(A_y);
        diameter_x01=(A_x-center_x);
        diameter_y01=(A_y-center_y);
        
        d_xy_02=diameter_x01.^2+diameter_y01.^2;
        dd_xy_02=2*sqrt(d_xy_02);
        max_diameter_x01= max(dd_xy_02);
        all_diameter_x01=[all_diameter_x01;max_diameter_x01];
        all_center_x01=[all_center_x01;center_x];
        all_center_y01=[all_center_y01;center_y];
    end
    %%%%%%%all_center_x01 contains center of ooi interest we are interested
    %%%%%%%dead reck method.
    if (ABCD.fe)
        set(mh2.handle4,'xdata',all_center_x01,'ydata',all_center_y01,'Color','green','MarkerSize',17);
    else
        set(mh2.handle4,'xdata',all_center_x01,'ydata',all_center_y01,'Color','white','MarkerSize',0.001);
    end
    if (i<2)
        
        static_obstacles_x=all_center_x01;
        static_obstacles_y=all_center_y01;
    else
        
        global_x= static_obstacles_x;
        global_y= static_obstacles_y;
        final_st_x= global_x;
        final_st_y =0.46+global_y;
       ang=position_data(3)-pi/2;
        rob_l=0.46;
        rob_l2=0.2;
        op_x=  all_center_x01;
        op_y=0.46+all_center_y01;
        ooi_point_x_02=cos(ang)*op_x-sin(ang)*op_y +position_data(1);
        ooi_point_y_02=sin(ang)*op_x+cos(ang)*op_y + position_data(2);
       
        %%%%%%%%%%%%%
        w7=[];
        if (~isempty(ooi_point_x_02))
            for w=1:length(ooi_point_x_02)
            w1=final_st_x-ooi_point_x_02(w);
            w2=final_st_y-ooi_point_y_02(w);
            w3=w1.*w1+ w2.*w2;
            w4=sqrt(w3);
            w5=min(w4);
            w6=find(w4==w5);
            w7=[w7;w6];
            end
            C = unique(w7);
       else
         C=[];
        end
        
Xe=Pose_Xe;

         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         if(~isempty(C))
         x_01=position_data(1);
         d=0.46;
         y_01=position_data(2);
         phi=position_data(3);
         x_s=x_01+d*cos(phi);
          y_s=y_01+d*sin(phi);
           dx=ooi_point_x_02-x_s;
           dy= ooi_point_y_02 - y_s ;
           
           MasuredRanges_01 = sqrt((dx.*dx + dy.*dy)) ;
          angle1=dy./dx;
         measuredAngle_01= atan(angle1)-phi+pi/2;
         noiseInMeasurements= 0;
         noiseInangle= 0;
           MasuredRanges = MasuredRanges_01 +  noiseInMeasurements ;
        measuredAngle=measuredAngle_01+noiseInangle;
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



       for u=1:length(C)
         x_e01=Xe(1);
         de=0.46;
         y_e01=Xe(2);
         phi_ee=Xe(3);
         x_es=x_e01+de*cos(phi_ee);
          y_es=y_e01+de*sin(phi_ee);
           id=C(u);
            eDX = (final_st_x(id)-x_es) ;      % (xu-x)
            eDY = (final_st_y(id)-y_es) ;      % (yu-y)
            eDD = sqrt( eDX*eDX + eDY*eDY ) ;
            eDD2=eDX*eDX + eDY*eDY ;
            H1 = [  -eDX/eDD , -eDY/eDD , 0 ] ;   % Jacobian of h(X); size 1x3
            H2 =[  eDY/eDD2 , -eDX/eDD2 , -1 ] ;
            H=[H1;H2];
            ExpectedRange = eDD; 
            
              angle_e=eDY./eDX;
              phi_021=Xe(3);
              exp_angle= atan(angle_e)-phi_021+pi/2;
            z1  = MasuredRanges(u) - ExpectedRange ;
            z2= measuredAngle(u) -exp_angle;
            z=[z1;z2];
            ssm=ContextSimulation.sdev_rangeMeasurement;
            ssa=ContextSimulation.stdDevGyro;
            R1 = ssm*ssm*1 ;
            R2= ssa*ssa*0.2;
            R=[R1 0; 0 R2];
            S = R + H*P*H' ;
            iS=inv(S);                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
            K = P*H'*iS ;           % Kalman gain
            Xe = Xe+K*z ;       % update the  expected value
            P = P-K*H*P ;       % update the C 
       end
         else
             
         end
        ange=Xe(3)-pi/2;
            ooi_point_xe_02=cos(ange)*op_x-sin(ange)*op_y +Xe(1);
            ooi_point_ye_02=sin(ange)*op_x+cos(ange)*op_y + Xe(2);   
        %%%%%%%%%%%%%%%%%%%%%%
        l_y=rob_l*cos(-ang);
        l_x=rob_l*sin(-ang);
        l_y2=0.23*cos(-ang);
        l_x2=0.23*sin(-ang);
         l_yy=rob_l2*cos(ang);
        l_xx=rob_l2*sin(ang);
        %%%%%%%%%%%%%%%%%%5
               Kl_y=rob_l*cos(-ange);
        Kl_x=rob_l*sin(-ange);
        Kl_y2=0.23*cos(-ange);
        Kl_x2=0.23*sin(-ange);
         kl_yy=rob_l2*cos(ange);
        Kl_xx=rob_l2*sin(ange);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
        set(mh3.handle1,'xdata',final_st_x,'ydata',final_st_y,'MarkerSize',17);
        set(mh3.handle3,'xdata',position_data(1),'ydata',position_data(2));
        set(mh3.handle5,'xdata',ooi_point_x_02,'ydata',ooi_point_y_02,'Color','green','MarkerSize',17);
        set(mh3.handle6,'xdata',ooi_point_xe_02,'ydata',ooi_point_ye_02,'Color','black','MarkerSize',15);%%%%%%%%%%%%%% green poinf for dead reckoning
        set(mh3.handle7,'xdata',position_data(1)+l_x,'ydata',position_data(2)+l_y);
        set(mh3.handle8,'xdata',position_data(1)+l_yy,'ydata',position_data(2)+l_xx);
        set(mh3.handle9,'xdata',position_data(1)-l_yy,'ydata',position_data(2)-l_xx);
        set(mh3.handle10,'xdata',position_data(1)+l_x2,'ydata',position_data(2)+l_y2);
        set(mh3.handle11,'xdata',position_data(1)-l_x2,'ydata',position_data(2)-l_y2);
%%%%%%%%%%%%%%%%%%%%%%%%%%5
        set(mh3.handle12,'xdata',position_data(1)+Kl_x,'ydata',position_data(2)+Kl_y);
        set(mh3.handle13,'xdata',position_data(1)+kl_yy,'ydata',position_data(2)+Kl_xx);
        set(mh3.handle14,'xdata',position_data(1)-kl_yy,'ydata',position_data(2)-Kl_xx);
        set(mh3.handle15,'xdata',position_data(1)+Kl_x2,'ydata',position_data(2)+Kl_y2);
        set(mh3.handle16,'xdata',position_data(1)-Kl_x2,'ydata',position_data(2)-Kl_y2);

       
    end
else
    counter=counter+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s2= sprintf(' Cartesian: LOCAl View(from robot)');
s3= sprintf(' Cartesian: GLOBAL View');
set(mh2.handle2,'string',s2);
set(mh3.handle2,'string',s3);
return;
%%   
   function MyCallBackA(~,~,x)
global ABCD;

if (x==1)
    ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
    return;
end
if (x==2)
    ABCD.endnow = ~ABCD.endnow;
    disp('you pressed "END NOW"');
    uiwait(msgbox('CLOSED'));
    close all;
    % students complete this.
    return;
end
if (x==3)
    ABCD.fe = ~ABCD.fe; %Switch ON->OFF->ON -> and so on.
    return;
end
return;
%%


