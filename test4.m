clear all; clf; close all;

%%%%%%%%%%%%%%% COMMENTS %%%%%%%%%%%%%
%Generated Cost function

%Next step is to add weight to controls and not iterate from 1 to
%length(th)

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
%plot(x,y); axis equal;

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
Initial_pose = transl(51,0,0)*trotz(pi/2);
Path(:,:,1) = Initial_pose;
Position(:,1) = transl(Initial_pose);
Orientation(:,1) = tr2rpy(Initial_pose);

for i=1:length(th)
    Path(:,:,i) = transl(x(i),y(i),0)*trotz(th(i));
    Position(:,i+1)=transl(Path(:,:,i));
    Orientation(:,i+1)=tr2rpy(Path(:,:,i));
end

%%
%%%%%%%%%%%%%%% ROBOT INITIALIZATION %%%%%%%%%%%%%
F = [1 2 3 1];
Robot= [0 -0.025 0 1;0.05 0 0 1;0 0.025 0 1];
Robot_theoric= Initial_pose*Robot';
patch('Vertices',Robot_theoric(1:3,:)','Faces',F,'FaceColor', 'r');

%%
%%%%%%%%%%%% STEP 0 AND PARAM SETUP %%%%%%%%%%%%%%

N = 1000; %Number of samples
Thorizon = 2.5; %Seconds into the future

meanVec = zeros([1 length(th)]);
cost = zeros([N 1]);

control = zeros([N Thorizon/deltaT]);
speed = zeros([N Thorizon/deltaT]);

for i=1:Thorizon/deltaT+1
    control(:,i) = normrnd(0,0.01*exp(i*deltaT/Thorizon),[N 1]);
    speed(:,i) = normrnd(u,0.5*exp(i*deltaT/Thorizon),[N 1]);
end

figure(1); clf;
for k = 1:N
    Position = Initial_pose;
    for t=1:Thorizon/deltaT+1
        Position = Position*trotz(control(k,t))*transl(speed(k,t)*deltaT,0,0);
        p2(:,t) = transl(Position);
        roundX = round(p2(1,t),1)*resolution+1;
        roundY = round(p2(2,t),1)*resolution+1;
        
        cost(k) = cost(k) +  ...
                2.5 * (u-speed(k,t))^2 + ...
                + 50*costMap(roundX,roundY)^2;
    end
    plot(p2(1,:),p2(2,:)); hold on;
    xResults(k,:) = p2(1,:);
    yResults(k,:) = p2(2,:);
end
axis equal;
plot(x,y);
xaxis([44 56]);
yaxis([0 12]);
figure;
histogram(cost);

figure(1);
[~, index] = min(cost(:));
X = xResults(index,:);
Y = yResults(index,:);

plot(X(:),Y(:),'g','LineWidth',8); 



%%%%%%%%%%%%%%%%%%%% VIDEO PATHS %%%%%%%%%%%%%%%%%%%%%%
%{
close all
fig = figure(3);
xaxis([44 56]);
yaxis([0 12]);
plot(x,y); 
hold on
% v = VideoWriter('paths.avi');
% v.FrameRate = 1;
% open(v);

meanVec = zeros([1 length(th)]); %Init vector of means
meanSpeed = ones([1 length(th)])*u; %Init vector of means
cost = zeros([N 1]);

followedPath(:,:,1) = Initial_pose;

for iter = 1:length(th)
    fprintf("Iter %d/%d\n",iter,length(th));

    control = zeros([N Thorizon/deltaT]);
    speed = zeros([N Thorizon/deltaT]);
    
    for i=1:Thorizon/deltaT+1
        control(:,i) = normrnd(meanVec(iter+i),0.01*exp(i*deltaT/Thorizon),[N 1]);
        speed(:,i) = normrnd(meanSpeed(iter+i),0.5*exp(i*deltaT/Thorizon),[N 1]);
    end

    for k = 1:N
        Position = followedPath(:,:,iter);
        for t=1:Thorizon/deltaT+1
            Position = Position*trotz(control(k,t))*transl(speed(k,t)*deltaT,0,0);
            p2(:,t) = transl(Position);

            cost(k) = cost(k) + calcCost(u,speed(k,t),costMap,resolution,p2(:,t));
        end
        xResults(k,:) = p2(1,:);
        yResults(k,:) = p2(2,:);

    end
    %TODO: SWAP MIN with REAL FUNCTION
    [~, index] = min(cost(:));
    meanVec(iter:iter+Thorizon/deltaT) = control(index,:);
    meanSpeed(iter:iter+Thorizon/deltaT) = speed(index,:); 

    X = xResults(index,:);
    Y = yResults(index,:);

    plot(X(:),Y(:)); 

    followedPath(:,:,iter+1) =  followedPath(:,:,iter)*trotz(meanVec(iter))*transl(meanSpeed(iter)*deltaT,0,0);
    %f = getframe(fig);
    %writeVideo(v,f);
end
% close(v);

%}

function totalCost = calcCost(targetSpeed,currentSpeed,costMap,resolution,position)
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
            
            totalCost = speedCost + positionCost;
end


