clear variables;
close all;
clc;

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
%[params4, pairsUsed4, worldPoints4] = CalibrationLeftMiddle();
%save params4 params4
%save worldPoints4 worldPoints4

load worldPoints
load params
load worldPoints2
load params2
load worldPoints3
load params3
load worldPoints4
load params4

%showExtrinsics(params3);
%Color normalize over all images
[subjectLeft,subjectMiddle,subjectRight] = colorNormalization(subjectLeft,subjectMiddle,subjectRight);

%Rectify image with background
[rectMiddleRight , rectRight] = rectifyStereoImages(subjectMiddle,subjectRight,params,'OutputView','full');
[rectLeft, rectMiddleLeft] = rectifyStereoImages(subjectLeft,subjectMiddle,params3, 'OutputView','full');


%Rectify image without with background used a mask 
[NoBGRectMiddleRight , NoBGRectRight] = rectifyStereoImages(removeBG(subjectMiddle, 0),removeBG(subjectRight, 1),params,'OutputView','full');
[NoBGRectLeft, NoBGRectMiddleLeft] = rectifyStereoImages(removeBG(subjectLeft, 0),removeBG(subjectMiddle, 0),params3, 'OutputView','full');

%Create the pointcloud using a disparity map and return unreliable points
[ptCloud1, unreliables1]  = createPointcloud(rectMiddleRight, rectRight,params,240, 368, NoBGRectMiddleRight);
[ptCloud2, unreliables2]  = createPointcloud(rectLeft,rectMiddleLeft,params3, 240, 368, NoBGRectLeft);

% figure; imshow(unreliables1);
% figure; imshow(unreliables2);

%figure; pcshow(ptCloud1);
%figure; pcshow(ptCloud2);

% Tranform according to the camera parameters to make it easier of the
% algorithm to line up
trans = params2.TranslationOfCamera2;
rot = params2.RotationOfCamera2;
tform = rigid3d(rot,trans);

trans = params.TranslationOfCamera2;
rot = params.RotationOfCamera2;
tform2 = rigid3d(rot,trans);

ptCloudRef = pctransform(ptCloud1,tform);
ptCloudCurrent =pctransform(ptCloud2,tform2);

%Align pointclouds using ICP
gridSize = 10;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
tform = pcregistericp(moving, fixed, 'Metric','pointToPoint','Extrapolate', true);

%Adjust pointcloud using tform gotten using ICP
ptCloudAligned = pctransform(ptCloudCurrent,tform);
figure;pcshowpair(ptCloudAligned, ptCloudRef);

%Merge pointclouds
ptCloudOut = pcmerge(ptCloudAligned, ptCloudRef, 1);
figure;pcshow(ptCloudOut);

% Start making mesh beginning
ptCloudOut = pcdownsample(ptCloudOut, 'gridAverage', gridSize);
x = double(ptCloudOut.Location(:,1));
y = double(ptCloudOut.Location(:,2));
z = double(ptCloudOut.Location(:,3));
trimesh = delaunay(x,y);
trisurf(trimesh,x,y,z);

%% Create point cloud
function [ptCloud, unreliables] = createPointcloud(J1,J2,stereoParams,min,max, mask)
    J1Gray=rgb2gray(J1);
    J2Gray=rgb2gray(J2);
    %imtool(stereoAnaglyph(J1,J2));
    
    %Create disparity map
    disparityMap = disparitySGM(J1Gray,J2Gray,'DisparityRange',[min max],'UniquenessThreshold',5);
    
    %Make the mask logical to remove it from the disparity map
    mask = rgb2gray(mask);
    mask = imbinarize(mask, 0);
    disparityMap = times(disparityMap, mask);
    
    % Create array size of the disparity map and set all values to one
    unreliables = ones(size(disparityMap));
    %Set the usefull point to zero
    unreliables(find(disparityMap~=0)) = 0;
    %unreliables are set to nan by disparitySGM
    unreliables(find(isnan(disparityMap))) = 1;
   
    % Replace NaN values to remove holes in the disparity map
    disparityMap(isnan(disparityMap))= -realmax('single');
    disparityMap = imfill(disparityMap,'holes');
    
    %remove outliers using median filter
    disparityMap = medfilt2(disparityMap, [75,75]);
    
    %figure;
    %imshow(disparityMap,[min max]);
    %title('Disparity Map');
    %colormap jet;
    %colorbar;
     
    points3D = reconstructScene(disparityMap, stereoParams);
    
    ptCloud = pcdenoise(pointCloud(points3D, 'Color',  J1));
end
%% Calibrate Camera unused function
function [params, tform, estimationErrors] = calibrateCamera(images1,images2,squareSize)
    I = readimage(images1,1);
    imageSize = [size(I,1),size(I,2)];
    [imagePoints,boardSize] = ...
    detectCheckerboardPoints(images1.Files,images2.Files);
    worldPoints = generateCheckerboardPoints(boardSize,squareSize);
    [params,~, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);

end

% This function returns a image without the background
function [removedBgImage] = removeBG(image, right)
    image = im2double(image);

    %Normalize the image
    imageNorm = (image - mean2(image))./std2(image);
    %Set image to grey scale
    imageGrey = rgb2gray(imageNorm);
    
    %Use canny edge detection to get the edges of the person
    utCanny = ut_edge(imageGrey, 'canny', 'sigma', 3, 'hysteresis', [0.06 0.005]);
    
    %Use dilation to connect the edges
    se = strel('diamond',1);
    utCanny = imdilate(utCanny,se);

    %Pad the image according to how the image looks
    %This is used to be able to fill the image, imfill does not fill if the
    %edge is not fully connected. This also means that it does not fill if
    %it is connected to the image border
    if right == 1
        utCanny = padarray(utCanny,[1 1],1,'post');
    else 
        utCanny = padarray(padarray(utCanny,[1 1],1,'post'),[0 1],1,'pre');
    end
    %Fill the padded image
    filled = imfill(utCanny,'holes');
    
    %Remove the padding
    if right == 1
        filled = filled(1:end-1,1:end-1);
    else 
        filled = filled(1:end-1,2:end-1);
    end
    
    removedBgImage = times(image,filled);
end


%This function get the mean of every color channel and calculates the over
%3 images and calculates a normalized image using these mains
function [I1,I2,I3] = colorNormalization(I1,I2,I3)

    %Get the color channels
    redChannel1 = I1(:, :, 1);
    greenChannel1 = I1(:, :, 2);
    blueChannel1 = I1(:, :, 3);
    redChannel2 = I2(:, :, 1);
    greenChannel2 = I2(:, :, 2);
    blueChannel2 = I2(:, :, 3);
    redChannel3 = I3(:, :, 1);
    greenChannel3 = I3(:, :, 2);
    blueChannel3 = I3(:, :, 3);
    %Get the means of color channels
    meanR1 = mean2(redChannel1);
    meanG1 = mean2(greenChannel1);
    meanB1 = mean2(blueChannel1);
    meanR2 = mean2(redChannel2);
    meanG2 = mean2(greenChannel2);
    meanB2 = mean2(blueChannel2);
    meanR3 = mean2(redChannel3);
    meanG3 = mean2(greenChannel3);
    meanB3 = mean2(blueChannel3);

    %Calc the means of color channels across the images
    desiredMeanR = mean([meanR1, meanR2, meanR3]);
    desiredMeanG = mean([meanG1, meanG2, meanG3]);
    desiredMeanB = mean([meanB1, meanB2, meanB3]);

    %Calc a factor for every image to normalize the color
    correctionFactorR1 = desiredMeanR / meanR1;
    correctionFactorG1 = desiredMeanG / meanG1;
    correctionFactorB1 = desiredMeanB / meanB1;

    correctionFactorR2 = desiredMeanR / meanR2;
    correctionFactorG2 = desiredMeanG / meanG2;
    correctionFactorB2 = desiredMeanB / meanB2;

    correctionFactorR3 = desiredMeanR / meanR3;
    correctionFactorG3 = desiredMeanG / meanG3;
    correctionFactorB3 = desiredMeanB / meanB3;

    %Normalize every color channel of every image and combine
    redChannel = uint8(single(redChannel1) * correctionFactorR1);
    greenChannel = uint8(single(greenChannel1) * correctionFactorG1);
    blueChannel = uint8(single(blueChannel1) * correctionFactorB1);
    % Recombine into an RGB image
    I1 = cat(3, redChannel, greenChannel, blueChannel);

    redChannel = uint8(single(redChannel2) * correctionFactorR2);
    greenChannel = uint8(single(greenChannel2) * correctionFactorG2);
    blueChannel = uint8(single(blueChannel2) * correctionFactorB2);
    % Recombine into an RGB image
    I2 = cat(3, redChannel, greenChannel, blueChannel);

    redChannel = uint8(single(redChannel3) * correctionFactorR3);
    greenChannel = uint8(single(greenChannel3) * correctionFactorG3);
    blueChannel = uint8(single(blueChannel3) * correctionFactorB3);
    % Recombine into an RGB image
    I3 = cat(3, redChannel, greenChannel, blueChannel);

end

function [stereoParams, pairsUsed, worldPoints] = CalibrationLeftMiddle()
% Define images to process
imageFileNames1 = {'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1089.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1225.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1361.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_137.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1497.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1633.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_1905.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_2041.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_2449.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_2585.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_273.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_409.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_545.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_681.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_817.jpg',...
    'pictures/calibration1/calibrationMiddle/Calibratie 1_M_953.jpg',...
    };
imageFileNames2 = {'pictures/calibration1/calibrationLeft/Calibratie 1_L_1.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1089.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1225.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1361.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_137.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1497.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1633.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_1905.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_2041.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_2449.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_2585.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_273.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_409.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_545.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_681.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_817.jpg',...
    'pictures/calibration1/calibrationLeft/Calibratie 1_L_953.jpg',...
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
function [stereoParams, pairsUsed, worldPoints] = CalibrationMiddleRight()
imageFileNames1 = {'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1633.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1769.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2041.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2177.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2449.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    };
imageFileNames2 = {'pictures\calibration1\calibrationRight\Calibratie 1_R_1361.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_1633.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_1769.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2041.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2177.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2449.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_273.jpg',...
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
imageFileNames1 = {'pictures\calibration1\calibrationRight\Calibratie 1_R_1089.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_1361.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_1633.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_1769.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2041.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2177.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2313.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_2449.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_273.jpg',...
    'pictures\calibration1\calibrationRight\Calibratie 1_R_409.jpg',...
    };
imageFileNames2 = {'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1089.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1633.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1769.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2041.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2177.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2313.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2449.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_409.jpg',...
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
imageFileNames1 = {'pictures\calibration1\calibrationLeft\Calibratie 1_L_1.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_1089.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_1225.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_1361.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_137.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_1497.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_1905.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_2585.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_273.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_409.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_545.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_681.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_817.jpg',...
    'pictures\calibration1\calibrationLeft\Calibratie 1_L_953.jpg',...
    };
imageFileNames2 = {'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1089.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1225.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1361.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_137.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1497.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_1905.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_2585.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_273.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_409.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_545.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_681.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_817.jpg',...
    'pictures\calibration1\calibrationMiddle\Calibratie 1_M_953.jpg',...
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