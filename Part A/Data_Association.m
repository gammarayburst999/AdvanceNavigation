%Author: <Satish Chandra Verma >
%Implement a Data Association process.
%The DA process would be able to identify (and keep identifying) the OOIs that are detected at each scan.

%%%%%%%%%%FOR part F
function Main()

global ABCD;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.
ABCD.flagPause=0;
ABCD.endnow=0;
ABCD.fe=1;

if ~exist('file','var'), file ='Laser__2.mat'; end

load('Laser__2.mat');
load('IMU_dataC.mat');
load('Speed_dataC.mat');

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
MyGUIHandles3.handle5 = plot(0,0,'.g','MarkerSize',12);
MyGUIHandles3.handle6 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle7 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle8 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle9 = plot(0,0,'.b','MarkerSize',12);
MyGUIHandles3.handle10 = plot(0,0,'.b','MarkerSize',12);
legend('Landmarks(map)','Robot',' OOIs(projected globally,DR)')
zoom on ;  grid on;
hold off;

%------------------ GUI ---------------

uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});
uicontrol('Style','pushbutton','String','FE on/off','Position',[170,1,80,20],'Callback',{@MyCallBackA,3});
%--------------------------------------

N = dataL.N;
N2=Vel.N;
skip=1;
i=1;
i_new=1;
skip_new=1;
t_f=[];
Pose=zeros(3,N2);
X0 = [0;0;pi/2] ;
X=X0;
Dt=0.005;
scan_02 = IMU.DATAf(6,:);
biasWz = mean(scan_02(1:2500));
Q2=scan_02-biasWz;
speed = Vel.speeds(1,:)';
steering = Q2;
while 1
    while 1
        if i_new>N2, break ;  end
        for i_new=1:N2 %iterations / Loop
            X = PredictVehiclePose(X, steering(i_new), speed(i_new) ,Dt);
            Pose(:, i_new)=X ;
            if i_new==N2
                final=X;
                final_pose=Pose;
            end
        end
        i_new=i_new+skip_new;
    end

    if (ABCD.flagPause), pause(0.2) ; continue ; end
    if i>N, break ;  end

    if (ABCD.endnow), break  ; end
    t =  double(dataL.times(i)-dataL.times(1))/10000;
    scan_i = dataL.Scans(:,i);
    MyProcessingOfScan(scan_i,t,MyGUIHandles2,MyGUIHandles3,i,final_pose);   % some function to use the data...
    pause(0.01) ;                   % wait for ~10ms (approx.)
    i=i+skip;
end
fprintf('\nDONE!\n');
return;
end

function X = PredictVehiclePose(X0,steering,speed,dt)
% Remember: state vector X = [x; y; heading]
X=X0 ;
dL = dt*speed ;
X(3) = X0(3)+dt*steering;
X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return;
end
%-----------------------------------------
function MyProcessingOfScan(scan,t,mh2,mh3,i,position_data)
global ABCD;
persistent static_obstacles_x;
persistent static_obstacles_y;
pos=position_data;
com_avg_st_x=[];
com_avg_st_y=[];
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
    all_diameter_02=[];
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

    if (ABCD.fe)
        set(mh2.handle4,'xdata',all_center_x01,'ydata',all_center_y01,'Color','green','MarkerSize',17);
    else
        set(mh2.handle4,'xdata',all_center_x01,'ydata',all_center_y01,'Color','white','MarkerSize',0.001);
    end
    if (i<11)
        count=i+1;
        static_obstacles_x=[static_obstacles_x,all_center_x01];
        static_obstacles_y=[static_obstacles_y,all_center_y01];
    else

        global_x= static_obstacles_x;
        global_y= static_obstacles_y;
        gl=size(global_x);
        for rt=1:gl
            avg_st_x=mean(global_x(rt,:));
            avg_st_y=mean(global_y(rt,:));
            com_avg_st_x=[com_avg_st_x;avg_st_x];
            com_avg_st_y=[com_avg_st_y;avg_st_y];
        end
        final_st_x= com_avg_st_x;
        final_st_y =0.46+com_avg_st_y;
        ang=position_data(3,round(5.3760*i))-pi/2;
        rob_l=0.46;
        rob_l2=0.2;
        op_x=  all_center_x01;
        op_y=0.46+all_center_y01;
        ooi_point_x_02=cos(ang)*op_x-sin(ang)*op_y +position_data(1,round(5.3760*i));
        ooi_point_y_02=sin(ang)*op_x+cos(ang)*op_y + position_data(2,round(5.3760*i));
       l_y=rob_l*cos(-ang);
        l_x=rob_l*sin(-ang);
        l_y2=0.23*cos(-ang);
        l_x2=0.23*sin(-ang);
         l_yy=rob_l2*cos(ang);
        l_xx=rob_l2*sin(ang);
        set(mh3.handle1,'xdata',final_st_x,'ydata',final_st_y,'MarkerSize',17);
        set(mh3.handle3,'xdata',position_data(1,round(5.3760*i)),'ydata',position_data(2,round(5.3760*i)));
        set(mh3.handle5,'xdata',ooi_point_x_02,'ydata',ooi_point_y_02,'Color','green','MarkerSize',17);
        set(mh3.handle6,'xdata',position_data(1,round(5.3760*(i)))+l_x,'ydata',position_data(2,round(5.3760*(i)))+l_y);
        set(mh3.handle7,'xdata',position_data(1,round(5.3760*(i)))+l_yy,'ydata',position_data(2,round(5.3760*(i)))+l_xx);
        set(mh3.handle8,'xdata',position_data(1,round(5.3760*(i)))-l_yy,'ydata',position_data(2,round(5.3760*(i)))-l_xx);
        set(mh3.handle9,'xdata',position_data(1,round(5.3760*(i)))+l_x2,'ydata',position_data(2,round(5.3760*(i)))+l_y2);
        set(mh3.handle10,'xdata',position_data(1,round(5.3760*(i)))-l_x2,'ydata',position_data(2,round(5.3760*(i)))-l_y2);


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
end
% ---------------------------------------
% Callback function. I defined it, and associated it to certain GUI button,
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
end
