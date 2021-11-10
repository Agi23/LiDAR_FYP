list = dir('agoritsa_ice_lidar_all/*.bag')
filenames = {list.name}

%fname = 'test.bag';
%fname = 'Sine_540.bag';
fname1 = 'All3_540.bag';
fname2 = 'closest.bag';
%fname = 'Pyramid_540.bag';
processFile(fname1);
processFile(fname2);
%processFile('test2.bag');
% for i=1:length(filenames)
%   processFile(num2str(filenames(i)));
% end
function processFile(filename)
    bag = rosbag(filename);
    bSel = select(bag,'Topic','/velodyne_packets');
    msgStructs = readMessages(bSel,'DataFormat','struct');
    msgStructs{1}


    veloReader = velodyneROSMessageReader(msgStructs,"VLP16")
    timeDuration = veloReader.StartTime + duration(0,0,1,'Format','s');
    ptCloudObj = readFrame(veloReader,timeDuration);
    ptCloudLoc = ptCloudObj.Location;
    reset(veloReader)
    %pcshow(ptCloudObj);

     %xlimits = ptCloudObj.XLimits;
     %ylimits = ptCloudObj.YLimits;
     %zlimits = ptCloudObj.ZLimits;
%test
%     xlimits = [-0.5 0];
%     ylimits = [1 1.5];
%     zlimits = [-0.5 0.5];

%sine
    xlimits = [-1.5 -0.5];
    ylimits = [-0.5 0.5];
    zlimits = [-1 1];
    xwide = [-5 5];
    ywide = [-5 5];
    zwide = [-5 5];
    reset(veloReader)

    player = pcplayer(xwide,ywide,zwide);
    %player = pcplayer(xlimits,ylimits,zlimits);
    count = 0;
    
    ptCloudObj = readFrame(veloReader);
    ptCloudDenoise = pcdenoise(ptCloudObj);
    while isOpen(player)
    %figure
        view(player,ptCloudDenoise.Location,ptCloudDenoise.Intensity);
    end
    %while isOpen(player)
    %figure
        view(player,ptCloudObj.Location,ptCloudObj.Intensity);
    %end
    %(filename + ".fig");
end
%bagInfo = rosbag('info','test.bag')
%rosbag info filename
%https://nl.mathworks.com/help/ros/ug/work-with-velodyne-ros-messages.html