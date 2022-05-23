clear all; clf; close all;


%Four possible approaches: 
%   1-Search for closest path point (CPU exhaustive)
%   2-Generate best option in each point (MEM exhaustive)
%   3-Moving point in the path (Very precise speed + Bezier curvature)
%   4-Should use acceleration instead of velocity

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

% for i = 1:width*resolution
%     for j = 1:height*resolution
%         dist = width*resolution;
%         for k=1:pathPoints
%             x_i = x(k);
%             y_i = y(k);
%             P1 = [x_i y_i];
%             P2 = [(i-1)/resolution (j-1)/resolution];
%             tmpDist = norm(P1-P2);
%             dist = min(dist,tmpDist);
%         end
%         costMap(i,j) = dist;
%     end
% end

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
%%%%%%%%%%%%%%%% IDEAL ORIENTATION %%%%%%%%%%%%%%%

%sideSlip angle is still TBD if useful



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
%%%%%%%%%%%%%%%%% SPEED IN PATH %%%%%%%%%%%%%%%%%%

fig = figure(1); hold on;
v = VideoWriter('map.avi');
v.FrameRate = 1;
open(v);

sumL = 0;
for i = 2:length(th)
    sumL = sumL + norm(Position(:,i) - Position(:,i-1));
    if sumL >= u
        plot(x,y,'b',Position(1,i),Position(2,i),'g--o'); axis equal; 
        f = getframe(fig);
        writeVideo(v,f);
        
        sumL = sumL - u;
    end
end
close(v); 

%%
%%%%%%%%%%%%%%% ROBOT INITIALIZATION %%%%%%%%%%%%%
F = [1 2 3 1];
Robot= [0 -0.025 0 1;0.05 0 0 1;0 0.025 0 1];
Robot_theoric= Initial_pose*Robot';
patch('Vertices',Robot_theoric(1:3,:)','Faces',F,'FaceColor', 'r');

%%
%%%%%%%%%%%% STEP 0 AND PARAM SETUP %%%%%%%%%%%%%%

N = 1000; %Number of samples
Thorizon = 2; %Seconds into the future


meanVec = zeros([1 pathPoints]); %Init vector of means
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
%figure;
%histogram(cost);

figure(1);
[~, index] = min(cost(:));
X = xResults(index,:);
Y = yResults(index,:);

plot(X(:),Y(:),'g','LineWidth',8); 



%%%%%%%%%%%%%%%%%%%% VIDEO PATHS %%%%%%%%%%%%%%%%%%%%%%

close all
fig = figure(3);
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
        control(:,i) = normrnd(meanVec(iter),0.01*exp(i*deltaT/Thorizon),[N 1]);
        speed(:,i) = normrnd(meanSpeed(iter),0.5*exp(i*deltaT/Thorizon),[N 1]);
    end

    for k = 1:N
        Position = followedPath(:,:,iter);
        for t=1:Thorizon/deltaT+1
            Position = Position*trotz(control(k,t))*transl(speed(k,t)*deltaT,0,0);
            p2(:,t) = transl(Position);
            roundX = round(p2(1,t),1)*resolution+1;
            roundY = round(p2(2,t),1)*resolution+1;
            
            cost(k) = cost(k) + ...
                2.5 * (u-speed(k,t))^2 + ...
                + 50*costMap(roundX,roundY)^2;
        end
        xResults(k,:) = p2(1,:);
        yResults(k,:) = p2(2,:);

    end
    %TODO: SWAP MIN with REAL FUNCTION
    [~, index] = min(cost(:));
    meanVec(iter:iter+Thorizon/deltaT) = control(index,:);
    %meanSpeed(iter:iter+Thorizon/deltaT) = speed(index,:); 

    X = xResults(index,:);
    Y = yResults(index,:);

    plot(X(:),Y(:)); 

    followedPath(:,:,iter+1) =  followedPath(:,:,iter)*trotz(meanVec(iter))*transl(meanSpeed(index)*deltaT,0,0);
    %f = getframe(fig);
    %writeVideo(v,f);
end
% close(v);

%}

function cost = calcCost()
    cost = 0;
end




