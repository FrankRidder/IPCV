%% Amazing code here
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

%Color normalize over all images
[subjectLeft,subjectMiddle,subjectRight] = colorNormalization(subjectLeft,subjectMiddle,subjectRight);

subjectLeft = im2double(subjectLeft);
subjectMiddle = im2double(subjectMiddle);
subjectRight = im2double(subjectRight);

maskLeft = getBG(subjectLeft, 0);
maskMiddel = getBG(subjectMiddle, 0);
maskRight = getBG(subjectRight, 1);

noBGsubjectLeft = times(subjectLeft, maskLeft);
noBGsubjectMiddle = times(subjectMiddle, maskMiddel);
noBGsubjectRight = times(subjectRight, maskRight);

[rectMiddleRight , rectRight] = rectifyStereoImages(subjectMiddle,subjectRight,params, 'OutputView','full');
[rectLeft, rectMiddleLeft] = rectifyStereoImages(subjectLeft,subjectMiddle,params3, 'OutputView','full'); 


[NoBGRectMiddleRight , NoBGRectRight] = rectifyStereoImages(noBGsubjectMiddle,noBGsubjectRight,params, 'OutputView','full');
[NoBGRectLeft, NoBGRectMiddleLeft] = rectifyStereoImages(noBGsubjectLeft,noBGsubjectMiddle,params3, 'OutputView','full');

ptCloud1 = createPointcloud(rectMiddleRight, rectRight,params,222, 350, NoBGRectMiddleRight);
ptCloud2 = createPointcloud(rectLeft,rectMiddleLeft,params3, 222, 350, NoBGRectLeft);

figure; pcshow(ptCloud1);
figure; pcshow(ptCloud2);


gridSize = 0.1;
fixed = pcdownsample(ptCloud1, 'gridAverage', gridSize);
moving = pcdownsample(ptCloud2, 'gridAverage', gridSize);
tform = pcregistericp(moving, fixed,'Extrapolate', true);

ptCloudAligned = pctransform(ptCloud2,tform);
pcshowpair(ptCloudAligned, ptCloud1)

%% Create point cloud
function [ptCloud] = createPointcloud(J1,J2,stereoParams,min,max, mask)
    J1Gray=rgb2gray(J1);
    J2Gray=rgb2gray(J2);
%   imtool(stereoAnaglyph(J1,J2));
    disparityMap = disparitySGM(J1Gray,J2Gray,'DisparityRange',[min max],'UniquenessThreshold',5);
    
    mask = rgb2gray(mask);
    mask = imbinarize(mask, 0);
    disparityMap = times(disparityMap, mask);
    
    figure;
    imshow(disparityMap,[min max]);
    title('Disparity Map');
    colormap jet;
    colorbar;
    
    points3D = reconstructScene(disparityMap, stereoParams);
    ptCloud = pcdenoise(pointCloud(points3D, 'Color',  J1));


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
function [filled] = getBG(image, right)
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
end

function [I1,I2,I3] = colorNormalization(I1,I2,I3)

redChannel1 = I1(:, :, 1);
greenChannel1 = I1(:, :, 2);
blueChannel1 = I1(:, :, 3);
redChannel2 = I2(:, :, 1);
greenChannel2 = I2(:, :, 2);
blueChannel2 = I2(:, :, 3);
redChannel3 = I3(:, :, 1);
greenChannel3 = I3(:, :, 2);
blueChannel3 = I3(:, :, 3);

meanR1 = mean2(redChannel1);
meanG1 = mean2(greenChannel1);
meanB1 = mean2(blueChannel1);
meanR2 = mean2(redChannel2);
meanG2 = mean2(greenChannel2);
meanB2 = mean2(blueChannel2);
meanR3 = mean2(redChannel3);
meanG3 = mean2(greenChannel3);
meanB3 = mean2(blueChannel3);

desiredMeanR = mean([meanR1, meanR2, meanR3]);
desiredMeanG = mean([meanG1, meanG2, meanG3]);
desiredMeanB = mean([meanB1, meanB2, meanB3]);

correctionFactorR1 = desiredMeanR / meanR1;
correctionFactorG1 = desiredMeanG / meanG1;
correctionFactorB1 = desiredMeanB / meanB1;

correctionFactorR2 = desiredMeanR / meanR2;
correctionFactorG2 = desiredMeanG / meanG2;
correctionFactorB2 = desiredMeanB / meanB2;

correctionFactorR3 = desiredMeanR / meanR3;
correctionFactorG3 = desiredMeanG / meanG3;
correctionFactorB3 = desiredMeanB / meanB3;

redChannel = uint8(single(redChannel1) * correctionFactorR1);
greenChannel = uint8(single(greenChannel1) * correctionFactorG1);
blueChannel = uint8(single(blueChannel1) * correctionFactorB1);
% Recombine into an RGB image
% Recombine separate color channels into a single, true color RGB image.
I1 = cat(3, redChannel, greenChannel, blueChannel);

redChannel = uint8(single(redChannel2) * correctionFactorR2);
greenChannel = uint8(single(greenChannel2) * correctionFactorG2);
blueChannel = uint8(single(blueChannel2) * correctionFactorB2);
% Recombine into an RGB image
% Recombine separate color channels into a single, true color RGB image.
I2 = cat(3, redChannel, greenChannel, blueChannel);

redChannel = uint8(single(redChannel3) * correctionFactorR3);
greenChannel = uint8(single(greenChannel3) * correctionFactorG3);
blueChannel = uint8(single(blueChannel3) * correctionFactorB3);
% Recombine into an RGB image
% Recombine separate color channels into a single, true color RGB image.
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