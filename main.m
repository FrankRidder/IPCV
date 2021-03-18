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
% [params, worldPoints] = calibrateCamera(imagesCameraCalibrationMiddle,imagesCameraCalibrationRight,10);
% save params params
% save worldPoints worldPoints
load worldPoints
load params
% [params2, worldPoints2] = calibrateCamera(imagesCameraCalibrationMiddle,imagesCameraCalibrationLeft,10);  
% figure;
% showExtrinsics(params);
% % Plot parameters
% [J,newOrigin] = undistortImage(subjectLeft,params.CameraParameters2);
% figure; imshowpair(subjectLeft,J,'montage');

% imshow(removeBg(subjectMiddle));
% 
% imshow(removeBg(subjectLeft));
% 
% imshow(removeBg(subjectRight));
% im = im2double(subjectMiddle);
% im = (im - mean2(im))./std2(im);
% imshow(im)
subjectMiddle = removeBg(subjectMiddle);
subjectRight = removeBg(subjectRight);
imshow(subjectRight);


[frameLeftRect, frameRightRect] = ...
    rectifyStereoImages(subjectMiddle, subjectRight, params);


disparityMap = disparitySGM(rgb2gray(frameLeftRect), rgb2gray(frameRightRect));
points3D = reconstructScene(disparityMap,params);
% Convert to meters and create a pointCloud object
points3D = points3D./300;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');

% Visualize the point cloud
view(player3D, ptCloud);


function [params, worldPoints] = calibrateCamera(images1,images2,squareSize)
    I = readimage(images1,1);
    imageSize = [size(I,1),size(I,2)];
    [imagePoints,boardSize] = ...
    detectCheckerboardPoints(images1.Files,images2.Files);
    worldPoints = generateCheckerboardPoints(boardSize,squareSize);
    params = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);
end
function [removedBgImage] = removeBg(image)
    lab_he = rgb2lab(image);
    ab = lab_he(:,:,2:3);
    ab = im2single(ab);
    nColors = 10;
    % repeat the clustering 3 times to avoid local minima
    pixel_labels = imsegkmeans(ab,nColors,'NumAttempts',3);
    mask = pixel_labels==2;
    cluster = image .*uint8(mask);
    removedBgImage=image-cluster;
end