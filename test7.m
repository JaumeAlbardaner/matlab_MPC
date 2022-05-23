clear all; clf; close all;

%%%%%%%%%%%%%%% COMMENTS %%%%%%%%%%%%%
%Should be pretty much like the paper
%Changed cost so that we swipe the values instead of moving an iterator
%And we also draw the weighted controls

%%%%%%%%%%%%%%% GLOBAL VARIABLES %%%%%%%%%%%%%%%%

%Target speed (m/s)
u = 5;

%Number of seconds between updates
deltaT = 0.1; 

%%
%%%%%%%%%%%%%%%%% PATH GENERATION %%%%%%%%%%%%%%%%
pathPoints=1000;
th = linspace( 0, pi/2, pathPoints);
x = 50*cos(th);
y = 50*sin(th);

%%
%%%%%%%%%%%%%%%% COST MAP %%%%%%%%%%%%%%%%%%%%%%%%

width = 60; %Width in meters
height = 60; %Height in meters
origin  = [min(x(:)) min(y(:))];
resolution = 10; %Each meter is divided in 10 blocks


costMap = zeros([width*resolution height*resolution]);

for k=1:pathPoints
    x_i = round(x(k),resolution/10)*resolution+1;
    y_i = round(y(k),resolution/10)*resolution+1;
    costMap(x_i,y_i) = 1;
end

pathWidth = 15;
EE = strel('diamond',2);
for i=1:pathWidth
    costMap = costMap + imdilate(costMap,EE);
end

costMap = costMap./max(costMap(:));
costMap = imcomplement(costMap);
imshow(costMap)


%%
%%%%%%%%%% PATH POSE CALCULATION %%%%%%%%%%%%%%%%%
Initial_pose = transl(40,0,0)*trotz(pi/2);
Path(:,:,1) = Initial_pose;
Position(:,1) = transl(Initial_pose);
Orientation(:,1) = tr2rpy(Initial_pose);

for i=1:length(th)
    Path(:,:,i) = transl(x(i),y(i),0)*trotz(th(i));
    Position(:,i+1)=transl(Path(:,:,i));
    Orientation(:,i+1)=tr2rpy(Path(:,:,i));
end

%%
%%%%%%%%%%%%%%%%%%%% VIDEO PATHS %%%%%%%%%%%%%%%%%%%%%%

N = 1000; %Number of samples
Thorizon = 2.5; %Seconds into the future


close all
fig = figure(3);
xaxis([44 56]);
yaxis([0 12]);
plot(x,y); 
hold on
% v = VideoWriter('paths.avi');
% v.FrameRate = 1;
% open(v);

meanVec = zeros([1 Thorizon/deltaT+1]); %Init vector of means
meanSpeed = ones([1 Thorizon/deltaT+1])*u; %Init vector of means
cost = zeros([N 1]);

followedPath(:,:,1) = Initial_pose;
for iter = 1:120
    fprintf("Iter %d/%d\n",iter,200);

    control = zeros([N Thorizon/deltaT]);
    speed = zeros([N Thorizon/deltaT]);
    
    for i=1:Thorizon/deltaT+1
        control(:,i) = normrnd(meanVec(i),0.01*exp(i*deltaT/Thorizon),[N 1]);
        speed(:,i) = normrnd(meanSpeed(i),0.5*exp(i*deltaT/Thorizon),[N 1]);
    end

    for k = 1:N
        Position = followedPath(:,:,iter);
        for t=1:Thorizon/deltaT+1
            Position = Position*trotz(control(k,t))*transl(speed(k,t)*deltaT,0,0);
            pos = transl(Position);

            cost(k) = cost(k) + calcCost(u,speed(k,t),costMap,resolution,pos,control(k,t));
        end
    end

    lambda = 10;
    cost = log((cost-min(cost))/max(cost)+1)*500; %Rnd variable
    totalSum = sum(exp(-1/lambda*cost));

    avgControl = zeros([1 Thorizon/deltaT+1]);
    avgSpeed = zeros([1 Thorizon/deltaT+1]);

    for i = 1:N
    avgControl = avgControl + exp(-1/lambda*cost(i))*control(i,:)/totalSum;
    avgSpeed = avgSpeed + exp(-1/lambda*cost(i))*speed(i,:)/totalSum;
    end

    meanVec(1:Thorizon/deltaT) = avgControl(2:end);
    meanSpeed(1:Thorizon/deltaT) = avgSpeed(2:end); 
    
    meanVec(end) = 0;
    meanSpeed(end) = 0;

    %WE ONLY PLOT THE REAL MEANS
    simulatedPose = followedPath(:,:,iter);

    for i = 1:length(avgSpeed)
       simulatedPose =  simulatedPose*trotz(avgControl(i))*transl(avgSpeed(i)*deltaT,0,0);
       pos = transl(simulatedPose);
       xResults(i) = pos(1);
       yResults(i) = pos(2);
    end
    
    plot(xResults(:),yResults(:)); 

    followedPath(:,:,iter+1) =  followedPath(:,:,iter)*trotz(avgControl(1))*transl(avgSpeed(1)*deltaT,0,0);
    %f = getframe(fig);
    %writeVideo(v,f);
end
% close(v);

%}

function totalCost = calcCost(targetSpeed,currentSpeed,costMap,resolution,position,angle)
            speedCost = 2.5 * (targetSpeed-currentSpeed)^2;
            
            width = size(costMap,1);
            height = size(costMap,2);
            x = position(1);
            y = position(2);
            roundX = round(x,1)*resolution+1;
            roundY = round(y,1)*resolution+1;
            if roundY > height || roundX > width || roundY < 1 || roundX <1
                positionCost = 1;
            else
                positionCost = 50*costMap(roundX,roundY)^2;
            end

            steerCost = 10 * angle^2;
            
            totalCost = speedCost + positionCost + steerCost;
end


