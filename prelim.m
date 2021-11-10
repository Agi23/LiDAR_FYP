list = dir('agoritsa_ice_lidar/*.bag');
filenames = {list.name}

filename = 'test.bag';
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
xlimits = [-0.5 0];
ylimits = [1 1.5];
zlimits = [-0.5 0.5];
roi = [-0.5 0 1 1.5 -0.5 0.5];
indices = findPointsInROI(ptCloudObj,roi);
cropped = select(ptCloudObj,indices);
pcwrite(cropped, "cropped.pcd");
minDistance = 0.5;
[labels,numClusters] = pcsegdist(ptCloudObj,minDistance);
pcshow(ptCloud.Location,labels);
colormap(hsv(numClusters));
title('Point Cloud Clusters');

% xwide = [-5 5];
% ywide = [-5 5];
% zwide = [-5 5];
reset(veloReader)

%player = pcplayer(xwide,ywide,zwide);
player = pcplayer(xlimits,ylimits,zlimits);
count = 0;
ptCloudObj = readFrame(veloReader);
%figure(player,ptCloudObj.Location,ptCloudObj.Intensity);
while(hasFrame(veloReader) && isOpen(player) && (veloReader.CurrentTime < veloReader.StartTime + seconds(2)))
     ptCloudObj = readFrame(veloReader);
     view(player,ptCloudObj.Location,ptCloudObj.Intensity);
     count = count + 1;
     %fname = "tester" + int2str(count) + ".fig";
     %savefig(fname);
     pause(1);
 end
%bagInfo = rosbag('info','test.bag')
%rosbag info filename
%https://nl.mathworks.com/help/ros/ug/work-with-velodyne-ros-messages.html