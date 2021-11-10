
%filename = '50_all_move_2021-10-12-14-43-47.bag';
%filename = "150_all_start_2021-10-12-14-24-01.bag";
filename = '50_all_start_2021-10-12-14-38-49.bag';
%filename ="100_all_start_2021-10-12-14-29-56.bag";
bag = rosbag(filename);
%emptyveloScan = rosmessage("velodyne_msgs/VelodyneScan","DataFormat","struct")
%emptyveloPkt = rosmessage("velodyne_msgs/VelodynePacket","DataFormat","struct")

bSel = select(bag,'Topic','/velodyne_packets');
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1}

%xyz = rosReadXYZ(ptcloud)

% emptyveloScan = rosmessage("velodyne_msgs/VelodyneScan","DataFormat","struct")
% emptyveloPkt = rosmessage("velodyne_msgs/VelodynePacket","DataFormat","struct")

veloReader = velodyneROSMessageReader(msgStructs,"VLP16")
timeDuration = veloReader.StartTime + duration(0,0,1,'Format','s');
ptCloudObj = readFrame(veloReader,timeDuration);
ptCloudLoc = ptCloudObj.Location;
%pcwrite(ptCloudObj, "objectAll3.pcd");
% xlimits = ptCloudObj.XLimits;
% ylimits = ptCloudObj.YLimits;
% zlimits = ptCloudObj.ZLimits;
% xlimits = [-0.5 0];
% ylimits = [1 1.5];
% zlimits = [-0.5 0.5];

% xlimits = [-1 0];
% ylimits = [-0.5 0.5];
% zlimits = [-1 1];

xlimits = [-5 5];
ylimits = [-5 5];
zlimits = [-5 5];
% roi = [-0.5 0 1 1.5 -0.5 0.5];
% indices = findPointsInROI(ptCloudObj,roi);
% cropped = select(ptCloudObj,indices);
% pcwrite(cropped, "cropped.pcd");
% minDistance = 0.5;
% [labels,numClusters] = pcsegdist(ptCloudObj,minDistance);
% pcshow(ptCloudObj.Location,labels);
% colormap(hsv(numClusters));
% title('Point Cloud Clusters');

% xwide = [-5 5];
% ywide = [-5 5];
% zwide = [-5 5];
reset(veloReader)

%player = pcplayer(xwide,ywide,zwide);
player = pcplayer(xlimits,ylimits,zlimits);
count = 0;
ptCloudObj = readFrame(veloReader);
%figure(player,ptCloudObj.Location,ptCloudObj.Intensity);
while(hasFrame(veloReader) && isOpen(player))
     ptCloudObj = readFrame(veloReader);
     view(player,ptCloudObj.Location,ptCloudObj.Intensity);
     count = count + 1;
     %fname = "tester" + int2str(count) + ".fig";
     %savefig(fname);
     pause(0.1);
 end
%bagInfo = rosbag('info','test.bag')
%rosbag info filename
%https://nl.mathworks.com/h
%
% 
% elp/ros/ug/work-with-velodyne-ros-messages.html
%%
% patient.name = 'John Doe';
% patient.billing = 127;
% patient.test = [79 75 73; 180 178 177.5; 220 210 205];
% patient.cloud = readFrame(veloReader);
% 
% patient(2).name = 'Jane Doe';
% patient(2).billing = 130;
% patient(2).test = [479 75 73; 180 178 177.5; 220 210 205];
% patient(2).cloud = ptCloudObj;
% 
% roi = [-1 1 0 0.1 -1 1];
% indices = findPointsInROI(ptCloudObj,roi);
% ptCloudObj = select(ptCloudObj,indices);

s = struct('a',{},'b',{},'c',{});

s(1).a = ptCloudObj;
s(1).b = 'hi';
s(1).c = 123;

s(2).a = ptCloudObj;
s(2).b = 'bye';
s(2).c = 234;

s(2).boop = 234;
%%
dataset = struct('distance', {}, 'Cloud',{}, 'filename', {},'croppedCloud', {});

list = dir('dataset/*.bag');
filenames = {list.name}
names = string(filenames)
sz = length(filenames)
for i=1:sz
    name = 'dataset/' + names(i);
    bag = rosbag(name);
    bSel = select(bag,'Topic','/velodyne_packets');
    msgStructs = readMessages(bSel,'DataFormat','struct');


    veloReader = velodyneROSMessageReader(msgStructs,"VLP16");
    timeDuration = veloReader.StartTime + duration(0,0,1,'Format','s');
    dataset(i).filename = name;
    dataset(i).Cloud = readFrame(veloReader,timeDuration);
    cloud = dataset(i).Cloud;
    
    roi = [-inf,inf;-0.4,0.2;-inf,inf];
    indices = findPointsInROI(cloud,roi);
    %croppedCloud=select(cloud,indices);
    dataset(i).croppedCloud = select(dataset(i).Cloud, indices);
    
    reset(veloReader)

end
%%
for i=1:sz 
    figure
    pcshow(dataset(i).croppedCloud);
    title(dataset(i).filename);
%%
end

%pcshow(dataset(1).Cloud);
%% Downsample

for i=1:sz 
    dataset(i).downSample = pcdownsample(dataset(i).croppedCloud,'gridAverage',0.01);
end
%% Denoise

for i=1:sz 
    dataset(i).denoise = pcdenoise(dataset(i).downSample);
end
%%
% 
% pc50 = pcdownsample(dataset(4).croppedCloud,'gridAverage',0.01);
% pc100 = pcdownsample(dataset(1).croppedCloud,'gridAverage',0.01);
% 
% pcshowpair(pc50, pc100);
% tform = pcregistericp(pc100,pc50);
% movingReg = pctransform(pc100,tform);
%%
% figure
% pcshowpair(movingReg,pc50,'MarkerSize',50)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% title('Point clouds after registration')
% legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
% legend('Location','southoutside')
%% Separate tiles from board

% 
% for i=1:sz 
%     dataset(i).tileIndex = find(dataset(i).denoise.Intensity < 60);
%     dataset(i).boardIndex = find(dataset(i).denoise.Intensity >= 60);
%     
%     dataset(i).tilepoints = dataset(i).denoise.Location(dataset(i).tileIndex,:);
%     dataset(i).boardpoints = dataset(i).denoise.Location(dataset(i).boardIndex,:);
% 
% end
% 
% pcshow(dataset(4).boardpoints)
%     for k = 1:length(dataset(i).Cloud.Intensity)
%         
%         if (MorecroppedCloud.Intensity(i) < 60)
%             
%         end
%         
%      end
%% pcfitplane function

maxDistance = 0.08; %large distance
referenceVector = [1,0,0];
maxAngularDistance = 5;
for i=1:sz 
    [dataset(i).model,dataset(i).inlier,dataset(i).outlier, dataset(i).planeError] = pcfitplane(dataset(i).denoise,maxDistance,referenceVector,maxAngularDistance);
    dataset(i).plane = select(dataset(i).denoise, dataset(i).inlier);
    figure
    pcshow(dataset(i).plane);
    title(dataset(i).filename);
end
% plane1 = select(pc50,inlierIndices);
%remainPtCloud = select(pc50,outlierIndices);

% [model2,inlierIndices2,outlierIndices2] = pcfitplane(pc100,maxDistance,referenceVector,maxAngularDistance);
% plane2 = select(pc100,inlierIndices2);
% %remainPtCloud2 = select(pc50,outlierIndices2);
% figure
% pcshow(plane2)
% title('100 Plane')
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold on
% plot(model2)
%%
for i=1:sz 
    dataset(i).tileIndex = find(dataset(i).plane.Intensity < 60);
    dataset(i).boardIndex = find(dataset(i).plane.Intensity >= 60);
    
    dataset(i).tilepoints = dataset(i).plane.Location(dataset(i).tileIndex,:);
    dataset(i).boardpoints = dataset(i).plane.Location(dataset(i).boardIndex,:);
    
    dataset(i).tileCloud = pointCloud(dataset(i).tilepoints);
    dataset(i).boardCloud = pointCloud(dataset(i).boardpoints);
    
    figure
    pcshow(dataset(i).boardCloud);
    title(dataset(i).filename + " Board");
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    figure
    pcshow(dataset(i).tileCloud);
    title(dataset(i).filename + " Tile");
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
end
%%

%%
for i=1:sz 
     roi = [-inf,inf;-inf,inf;dataset(i).tileCloud.ZLimits(1),dataset(i).tileCloud.ZLimits(2)];
     indices = findPointsInROI(dataset(i).boardCloud,roi);
%    croppedCloud=select(cloud,indices);
%     dataset(i).croppedCloud = select(dataset(i).Cloud, indices);
     dataset(i).croppedBoard = select(dataset(i).boardCloud, indices);
end
maxDistance = 0.05; %large distance
referenceVector = [1,0,0];
maxAngularDistance = 5;
for i=1:sz 
    [dataset(i).modelRefine,dataset(i).inlierRefine,dataset(i).outlierRefine, dataset(i).planeErrorRefine] = pcfitplane(dataset(i).croppedBoard,maxDistance,referenceVector,maxAngularDistance);
    dataset(i).planeRefine = select(dataset(i).croppedBoard, dataset(i).inlierRefine);
    figure
    pcshow(dataset(i).planeRefine);
    title(dataset(i).filename);
    hold on
    plot(dataset(i).modelRefine)
end
%% Align

%referenceCloud = dataset(4).boardCloud; %Select closest as reference
referenceCloud = dataset(4).boardCloud; %Select closest as reference
for i=1:sz 
    [dataset(i).transform,dataset(i).aligned,dataset(i).rmsError] = pcregistericp(referenceCloud,dataset(i).croppedBoard);
    %dataset(i).transform = pcregistericp(referenceCloud, dataset(i).boardCloud);
    %dataset(i).aligned = pctransform(dataset(i).boardCloud,dataset(i).transform);
    
    figure 
    %pcshowpair(dataset(i).aligned,referenceCloud,'MarkerSize',50)
    pcshowpair(dataset(i).aligned,referenceCloud);
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    heading = "aligned " + dataset(i).filename;
    title(heading);
    legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
    legend('Location','southoutside')
end
% pcshowpair(pc50, pc100);
% tform = pcregistericp(pc100,pc50);
% movingReg = pctransform(pc100,tform);
%% Tabulation of transform

for i=1:sz 
    tabledata(i).deltax = dataset(i).transform.Translation(1);
    tabledata(i).deltay = dataset(i).transform.Translation(2);
    tabledata(i).deltaz = dataset(i).transform.Translation(3);
end
    deltay =[tabledata.deltay];
    deltax =[tabledata.deltax];
    deltaz =[tabledata.deltaz];
    
testtable.deltax = dataset(1).transform.Translation(1);

[xData, yData] = prepareCurveData( deltax, deltaz );

% Set up fittype and options.
ft = fittype( 'poly1' );

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft );

% Plot fit with data.
figure( 'Name', 'z offset graph' );
h = plot( fitresult, xData, yData );
legend( h, 'deltaz vs. deltax', 'z offset graph', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'deltax', 'Interpreter', 'none' );
ylabel( 'deltaz', 'Interpreter', 'none' );
grid on

%% Translation analysis
% deltax values are as expected 
% 
% deltay values are near zero
% 
% deltaz values have a slight increase - need to adjust with a rotation

for i=1:sz 
    dataset(i).rot = dataset(i).transform.Rotation;
    trans = [0, 0, 0];
    tform = rigid3d(dataset(i).rot,trans);
    dataset(i).rotatedCloud = pctransform(dataset(i).croppedBoard, tform);
    %dataset(i).rotatedPlane = pctransform(dataset(i).modelRefine);
    figure
    pcshow(dataset(i).croppedBoard);
    title(dataset(i).filename + " cloud before transform")
    
    figure
    pcshow(dataset(i).rotatedCloud);
    title(dataset(i).filename + " after rotation")
% rot_edit = rot;
% rot_edit(1,1)=1;
% rot_edit(2,1)=0;
% rot_edit(2,2)=1;
% rot_edit(2,3)=0;
% rot_edit(3,3)=1;
end
%%
for i=1:sz 

    
    figure
    pcshow(dataset(i).tileCloud);
    title(dataset(i).filename + " Tile");
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
end
bottom100z = 0;
divz = 0.02;
divy_2 = -0.13;
divy_3 = 0;
divy_1 = -0.23;

board100(1).Block_roi = [-inf inf -inf divy_1 divz inf];
board100(2).Block_roi  = [-inf inf divy_1 divy_2 divz inf];
board100(3).Block_roi  = [-inf inf divy_2 divy_3 divz inf];
board100(4).Block_roi  = [-inf inf divy_3 inf divz inf];

board100(5).Block_roi  = [-inf inf -inf divy_1 -inf divz];
board100(6).Block_roi  = [-inf inf divy_1 divy_2 -inf divz];
board100(7).Block_roi  = [-inf inf divy_2 divy_3 -inf divz];
board100(8).Block_roi  = [-inf inf divy_3 inf -inf divz];

tileCloud = dataset(1).tileCloud;
for i=1:8
    indices = findPointsInROI(tileCloud,board100(i).Block_roi);
    board100(i).BlockCloud = select(tileCloud,indices);
    %Arithmetic mean
    board100(i).net_mean = mean(board100(i).BlockCloud.Location(:,1))
    board100(i).min = min(board100(i).BlockCloud.Location(:,1));
    N=board100(i).BlockCloud.Count;
    %N=57;
    %Compute sum expressions
    square_sum = 0;
    cube_sum = 0;
    power4_sum =0;
    for x=1:N
        xvalues = board100(i).BlockCloud.Location(:,1);
        square_sum = square_sum + (xvalues(x,1)^2);
        cube_sum = cube_sum + (xvalues(x,1)^3);
        power4_sum = power4_sum + (xvalues(x,1)^4);
        xtest = xvalues(x,1);
    end
    %RMS mean
    board100(i).net_rms=sqrt((1/N)*square_sum);
    
    %Skewness
    board100(i).skewness = (1/(N*N*(board100(i).net_rms)^3))*cube_sum;
    
    %Kurtosis
    board100(i).Kurtosis = (1/(N*N*(board100(i).net_rms)^4))*power4_sum;
    
    figure
    pcshow(board100(i).BlockCloud);
    title(i);
end
%%
for i=1:8
    zvals = board100(i).BlockCloud.Location(:,3);
    first_val = zvals(1);
    prev_val = first_val;
    n_lines = 1;
    line_size = 0;
    jump_val = 0.01;
    for k=1:board100(i).BlockCloud.Count
        board100(i).diff(k,1) = abs(zvals(k)-prev_val);
        prev_val = zvals(k);
        
        if board100(i).diff(k,1) >= jump_val
            n_lines = n_lines +1;
            line_size =1;
           % board100(i).linesArr(n_lines)=[];
        else
            line_size = line_size +1;
        end 
        
        %board100(i).lines(n_lines,line_size) = board100(i).BlockCloud.Location(k,1);
        board100(i).lines(n_lines,line_size) = k;
        
%         myArr = [];
%         myArr = board100(i).linesArr(n_lines);
%         myArr(end+1)=k;
%         board100(i).linesArr(n_lines) = myArr;
    end
    board100(i).nLines =n_lines;
end
%%
%board100(1).lineArr = [];
% for i=1:3
%     board100(1).lineArr(i)=[];
% end
board100(1).lineArr = struct('x' , {}, 'y', {}, 'z', {})
board100(1).lineArr(1).x =5;
board100(1).lineArr(1).x(end+1) =9;
board100(1).lineArr(2).x =7;

%board100(1).lineCell(1,1)=4;
% board100(1).lineCell(2,1)=77;
% board100(1).lineCell(1,2)=444;


%arrPointClouds
board100(1).pointLine=struct('ptCloud', {}, 'rms', {}, 'mean', {}, 'kurtosis', {}, 'skewness', {});
board100(1).pointLine(1).ptCloud = board100(1).BlockCloud;
board100(1).pointLine(2).ptCloud = board100(2).BlockCloud;
%board100(1).lineArr(end+1)=[69,69,420];
%board100(1).lineArr(end+1)=88;
%%
Viewer = matlab.diagram.ClassViewer('dataset','matlab.net.http.RequestMessage')

%%
% 
% trans = [0, 0, 0];
% tform = rigid3d(rot,trans);
% rotatedCloud = pctransform(dataset(1).boardCloud, tform);
% similarCloud = pcregistericp(dataset(1).boardCloud, rotatedCloud)
% figure
% pcshow(rotatedCloud);
% title("rotated");
% 
% figure
% pcshow(dataset(1).boardCloud)
% title("Original")


%%
% figure
% pcshowpair(movingReg,pc50,'MarkerSize',50)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% title('Point clouds after registration')
% legend({'Moving point cloud','Fixed point cloud'},'TextColor','w')
% legend('Location','southoutside')
%%
% roi = [-inf,inf;-1,1;-inf,inf];
% indices = findPointsInROI(plane2,roi);
% croppedCloud=select(plane2,indices);
% pcshow(pcdenoise(croppedCloud));
%%
% plot(model1);

a=cell(1,2);
for i=1:2
 a(i)=mat2cell(zeros(8,8,2),8,8,2);
 a{i}(:,:,1)=randi(8,8);
end