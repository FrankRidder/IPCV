%% Amazing code here
clear variables
close all

%% Calibration images
imagesCameraCalibrationLeft = imageSet('pictures/calibration1/calibrationLeft');
imagesCameraCalibrationMiddle = imageSet('pictures/calibration1/calibrationMiddle');
imagesCameraCalibrationRight = imageSet('pictures/calibration1/calibrationRight');
%% Subject 1
imagesSubject = imageSet('pictures/subject');
subjectLeft = read(imagesSubject,1);
subjectMiddle = read(imagesSubject,2);
subjectRight = read(imagesSubject,3);
squareSize = 10;
I = readimage(imagesCameraCalibrationMiddle,1);
imageSize = [size(I,1),size(I,2)];
% [params, pairsUsed, worldPoints] = CalibrationMiddleRight();
% save params params
% save worldPoints worldPoints
% [params2, pairsUsed2, worldPoints2] = CalibrationRightMiddle();
% save params2 params2
% save worldPoints2 worldPoints2
% [params3, pairsUsed3, worldPoints3] = CalibrationMiddleLeft();
% save params3 params3
% save worldPoints3 worldPoints3

load worldPoints
load params
load worldPoints2
load params2
load worldPoints3
load params3

I1 = subjectMiddle;
I2 = subjectRight;
I3 = subjectLeft;

ptCloud = pcdenoise(createPointcloud(I1, I2,params,222, 350));
ptCloud2 =createPointcloud(I3, I1, params3,222, 350);
pcshow(ptCloud);
pcshowpair(ptCloud, ptCloud2)

gridSize = 0.0001;
% ptCloud = pcdownsample(ptCloud, 'gridAverage', gridSize);
x= double(ptCloud.Location(:, 1));
y= double(ptCloud.Location(:, 2));
%     z= double(ptCloud.Location(:, 3).');
z= double(ptCloud.Location(:, 3));
% shp = boundary(x,y,z,1);
% trisurf(shp)
%     tri = delaunay(x, y);
%     trimesh(tri, x, y, z);
% gridSize = 0.1;
% fixed = pcdownsample(ptCloud1, 'gridAverage', gridSize);
% moving = pcdownsample(ptCloud2, 'gridAverage', gridSize);
% tform = pcregistericp(moving, fixed,'Extrapolate', true);
% 
% ptCloudAligned = pctransform(ptCloud2,tform);
% pcshowpair(ptCloudAligned, ptCloud1)

%% Create point cloud
function [ptCloud] = createPointcloud(Z1,Z2,stereoParams,min,max )
    Z1 = undistortImage(Z1,stereoParams.CameraParameters1);
    Z2 = undistortImage(Z2,stereoParams.CameraParameters2);
    [J1,J2] = rectifyStereoImages(Z1,Z2,stereoParams, ...
      'OutputView','full');
    J1Gray=rgb2gray(J1);
    J2Gray=rgb2gray(J2);
%   imtool(stereoAnaglyph(J1,J2));
    disparityMap = disparitySGM(imgaussfilt(histeq(J1Gray)),imgaussfilt(histeq(J2Gray)),'DisparityRange',[min max],'UniquenessThreshold',10);
    points3D = reconstructScene(disparityMap, stereoParams);
    ptCloud = pcdenoise(removeInvalidPoints(pointCloud(points3D, 'Color', J1)));
 
  

% 
% 
% z
% k =boundary(real(x),real(y),real(z));


end
%% Calibrate Camera
function [params, tform, estimationErrors] = calibrateCamera(images1,images2,squareSize)
    I = readimage(images1,1);
    imageSize = [size(I,1),size(I,2)];
    [imagePoints,boardSize] = ...
    detectCheckerboardPoints(images1.Files,images2.Files);
    worldPoints = generateCheckerboardPoints(boardSize,squareSize);
    [params,~, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

end
%% Remove background
function [removedBgImage] = removeBg(image, right)
image = im2double(image);
imageNorm = (image - mean2(image))./std2(image);
imageGrey = rgb2gray(imageNorm);
se = strel('diamond',1);
utCanny = ut_edge(imageGrey, 'canny', 'sigma', 3, 'hysteresis', [0.06 0.005]);
utCanny = imdilate(utCanny,se);
if right == 1
    utCanny = padarray(utCanny,[1 1],1,'post');
else 
    utCanny = padarray(padarray(utCanny,[1 1],1,'post'),[0 1],1,'pre');
end
filled = imfill(utCanny,'holes');
if right == 1
    filled = filled(1:end-1,1:end-1);
else 
    filled = filled(1:end-1,2:end-1);
end
removedBgImage =times(image, filled);
end
%% Find features
function [matchedPoints1, matchedPoints2] = findFeatures(I1, I2)
    points1 = detectHarrisFeatures(I1);
    points2 = detectHarrisFeatures(I2);
    [features1,valid_points1] = extractFeatures(I1,points1);
    [features2,valid_points2] = extractFeatures(I2,points2);
    indexPairs = matchFeatures(features1,features2);
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
end

%%
function [stereoParams, pairsUsed, worldPoints] = CalibrationMiddleRight()
imageFileNames1 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1633.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1769.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2041.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2177.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2449.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    };
imageFileNames2 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1633.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1769.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2041.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2177.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2449.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_273.jpg',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
squareSize = 10;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Read one of the images from the first stereo pair
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

end


%%
function [stereoParams, pairsUsed, worldPoints] = CalibrationRightMiddle()
imageFileNames1 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1089.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1633.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_1769.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2041.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2177.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2313.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_2449.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_273.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationRight\Calibratie 1_R_409.jpg',...
    };
imageFileNames2 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1089.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1633.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1769.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2041.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2177.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2313.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2449.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_409.jpg',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
squareSize = 10;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Read one of the images from the first stereo pair
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);
end


function [stereoParams, pairsUsed, worldPoints] = CalibrationMiddleLeft()
imageFileNames1 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1089.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1225.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_137.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1497.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_1905.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_2585.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_273.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_409.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_545.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_681.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_817.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationLeft\Calibratie 1_L_953.jpg',...
    };
imageFileNames2 = {'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1089.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1225.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_137.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1497.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_1905.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_2585.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_409.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_545.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_681.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_817.jpg',...
    'C:\Users\kopop\Documents\MATLAB\IPCV\pictures\calibration1\calibrationMiddle\Calibratie 1_M_953.jpg',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
squareSize = 10;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Read one of the images from the first stereo pair
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);
end