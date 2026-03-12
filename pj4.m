%% blg
% 相机移速900px/5s，顶部相机缩放比例1060px/30mm，侧边相机缩放比例2427px/30mm
clear; clc; close all;

%% 多图旋转切片与拼接
% --- 1. 参数设置 ---
imageDir = './GlassTubeData/';        % 图像文件夹
imageFormat = '*.bmp';         % 图像格式
nominalStep = 900;             % 理论步长
searchRange = 40;              % 搜寻范围 (允许左右误差 +/- 40px)
debugMode = false;              % 是否打印调试信息

% 预设逆时针旋转0.39°
preRotationAngle = -0.39;      

filePattern = fullfile(imageDir, imageFormat);
dirInfo = dir(filePattern);
% 确保按拍摄顺序依次拼接
[~, index] = sort({dirInfo.name});
dirInfo = dirInfo(index);
numImages = length(dirInfo);

if numImages < 2, error('需要至少两张图片'); end

% --- 2. 辅助函数 ---
% 定义旋转函数：保持原图大小 ('crop')，使用双三次插值 ('bicubic') 保证画质
doRotate = @(img) imrotate(img, preRotationAngle, 'bicubic', 'crop');

% --- 3. 首图处理 ---
fprintf('开始处理，预设旋转角度: %.2f°\n', preRotationAngle);

% 读取并立即旋转第一张图
imgFirstRaw = imread(fullfile(imageDir, dirInfo(1).name));
imgFirstRot = doRotate(imgFirstRaw); 

[H, W, C] = size(imgFirstRot);

% 第一张图：直接取中心部分作为基准
startCropWidth = nominalStep; 
xStart_1 = floor((W - startCropWidth)/2) + 1;
fullStrip = imgFirstRot(:, xStart_1 : xStart_1 + startCropWidth - 1, :);

% 缓存上一张图的灰度数据（已旋转），用于下一轮的互相关匹配
if C == 3
    grayPrev = rgb2gray(imgFirstRot);
else
    grayPrev = imgFirstRot;
end

% 记录步长
actualSteps = zeros(numImages-1, 1);

% --- 4. 循环拼接 (从右向左) ---
for i = 2:numImages
    % A. 读取原始图像并立即旋转
    imgCurrRaw = imread(fullfile(imageDir, dirInfo(i).name));
    imgCurrRot = doRotate(imgCurrRaw); % 对全图进行旋转
    
    % 转灰度用于计算
    if C == 3
        grayCurr = rgb2gray(imgCurrRot);
    else
        grayCurr = imgCurrRot;
    end
    
    % B. 抗抖动计算 (在已旋转的图像之间进行匹配)
    % 1. 在上一张图(Prev)中心偏左取模板
    roiH = floor(H/3) : floor(2*H/3); 
    tmplX = floor(W/2) - 200;         
    tmplW = 150;                      
    template = grayPrev(roiH, tmplX : tmplX + tmplW);
    
    % 2. 在当前图(Curr)预期位置搜索
    expectedX = tmplX + nominalStep; 
    searchX_Start = max(1, expectedX - searchRange);
    searchX_End = min(W, expectedX + tmplW + searchRange);
    searchRegion = grayCurr(roiH, searchX_Start : searchX_End);
    
    % 3. 执行匹配
    c = normxcorr2(template, searchRegion);
    [ypeak, xpeak] = find(c == max(c(:)));
    
    % 4. 坐标转换求实际步长 dx 和垂直偏差 dy
    xMatchInSearch = xpeak - size(template, 2) + 1; 
    xMatchInFull = searchX_Start + xMatchInSearch - 1;
    dx = xMatchInFull - tmplX;
    dy = ypeak - size(template, 1); 
    
    % 异常保护
    if abs(dx - nominalStep) > searchRange + 10
        dx = nominalStep; dy = 0;
        warning('Img %d 匹配失败，使用默认步长', i);
    end
    actualSteps(i-1) = dx;
    
    % C. 切片与拼接
    % 1. 动态裁剪：在【已旋转】的当前图中，取【正中心】宽度为 dx 的切片
    sliceWidth = round(dx);
    xSliceStart = floor((W - sliceWidth)/2) + 1;
    xSliceEnd = xSliceStart + sliceWidth - 1;
    
    newStrip = imgCurrRot(:, xSliceStart:xSliceEnd, :);
    
    % 2. 垂直对齐微调 (imtranslate)
    % 虽然已经统一旋转了，但机械传动仍可能有微小的上下抖动
    % dy 是互相关算出来的垂直像素差，这里进行补偿
    if abs(dy) > 0
        newStrip = imtranslate(newStrip, [0, -dy], 'FillValues', 0);
    end
    
    % 3. 缝合 (从右向左：[新, 旧])
    fullStrip = [newStrip, fullStrip];
    
    % D. 更新缓存
    grayPrev = grayCurr; % 当前图变为下一轮的"上一张"
    
    if debugMode
        fprintf('Img %02d: 预旋0.39°, 实际步长=%dpx, 垂直补偿=%+d\n', ...
            i, sliceWidth, round(dy));
    end
end

% --- 5. 结果输出 ---
figure('Name', '拼接结果', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.6]);

subplot(2,1,1);
imshow(fullStrip);
title(sprintf('拼接结果'));
xlabel('像素');

subplot(2,1,2);
plot(actualSteps, '-o');
yline(nominalStep, '--r');
title('实际检测到的步长 (px)');
xlabel('图片序号'); ylabel('像素宽度');
grid on;

imwrite(fullStrip, 'stitched_glass_tube_prerotated.png');
fprintf('处理完成，结果已保存为 stitched_glass_tube_prerotated.png\n');

%% 上下包络Canny提取
% --- 1. 读取之前的拼接结果 ---
imageFile = 'stitched_glass_tube_prerotated.png';
if ~exist(imageFile, 'file')
    error('未找到文件: %s，请先运行上一段拼接代码。', imageFile);
end

fullImg = imread(imageFile);

% --- 2. 交互式选取区域 (ROI) ---
f = figure('Name', 'ROI选择', 'NumberTitle', 'off');
imshow(fullImg);
title('框选提取区域，双击选框内部确认', 'Color', 'r', 'FontSize', 12);

% 等待用户框选
[croppedImg, rect] = imcrop; 
close(f);

if isempty(croppedImg)
    error('未选择区域');
end

% 转灰度
if size(croppedImg, 3) == 3
    grayRoi = rgb2gray(croppedImg);
else
    grayRoi = croppedImg;
end

% --- 3. Canny 边缘检测与预处理 ---

% A. 高斯滤波 (平滑细微噪点)
grayRoiFiltered = imgaussfilt(grayRoi, 1.5); 

% B. Canny 检测
% 阈值可调
bwEdges = edge(grayRoiFiltered, 'Canny', [0.15,0.25]);
bwClean = bwEdges;

% --- 4. 提取上下包络 ---
[rows, cols] = size(bwClean);

% 初始化两张空图，分别存上边缘和下边缘
topEdgeMap = false(rows, cols);
bottomEdgeMap = false(rows, cols);

% 逐列扫描
for c = 1:cols
    % 找到当前列 c 中，所有边缘点的行索引 (row indices)
    edgePixels = find(bwClean(:, c));
    
    if ~isempty(edgePixels)
        % 1. 提取最上边缘 (Row index 最小)
        minRow = min(edgePixels);
        topEdgeMap(minRow, c) = 1;
        
        % 2. 提取最下边缘 (Row index 最大)
        maxRow = max(edgePixels);
        bottomEdgeMap(maxRow, c) = 1;
    end
end

% 移除小于 30 像素的孤立短线 (去除灰尘和不连续的杂讯)
topEdgeMap = bwareaopen(topEdgeMap,30);
bottomEdgeMap = bwareaopen(bottomEdgeMap,30);

% 合并结果
finalResult = topEdgeMap | bottomEdgeMap;


% --- 5. 结果可视化 ---
figure('Name', '包络提取结果', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.5]);

% 原始 ROI
subplot(1, 3, 1);
imshow(grayRoi);
title('原始框选区域');

% Canny 原始结果 (包含内部线条)
subplot(1, 3, 2);
imshow(bwClean);
title('Canny 提取可视化');

% 最终筛选结果 (叠加显示)
subplot(1, 3, 3);
imshow(grayRoi); hold on;
% 用绿色绘制上边缘，蓝色绘制下边缘
[py_top, px_top] = find(topEdgeMap);
[py_bot, px_bot] = find(bottomEdgeMap);

% 注意 plot 的坐标是 (x, y)，而 find 返回是 (row, col) 即 (y, x)
plot(px_top, py_top, 'g.', 'MarkerSize', 1); 
plot(px_bot, py_bot, 'b.', 'MarkerSize', 1);
title('包络提取结果');
legend('上边缘', '下边缘');

% --- 6. 保存数据 ---
% 保存二值图
imwrite(finalResult, 'glass_tube_outer_edges.png');

% 保存坐标数据用于测量
edgeData = table;
edgeData.X = (1:cols)';
% 初始化 Y 坐标 (NaN 表示该列没检测到边缘)
edgeData.TopY = nan(cols, 1);
edgeData.BottomY = nan(cols, 1);

for c = 1:cols
    idxTop = find(topEdgeMap(:, c), 1);
    if ~isempty(idxTop), edgeData.TopY(c) = idxTop; end
    
    idxBot = find(bottomEdgeMap(:, c), 1);
    if ~isempty(idxBot), edgeData.BottomY(c) = idxBot; end
end

% 计算粗略直径
edgeData.Diameter = edgeData.BottomY - edgeData.TopY;
writetable(edgeData, 'edge_coordinates.csv');

fprintf('处理完成！\n图片已保存为 glass_tube_outer_edges.png\n坐标数据已保存为 edge_coordinates.csv\n');

%% 成功，阳光普照法获得轮廓，最小二乘法拟合圆，准确且快速，适用于图中有两个近似同心圆的情况，方案为用阳光普照法获得外圆轮廓，最小二乘法拟合外圆圆心半径，再根据外圆圆心用阳光普照法获得内圆轮廓，最小二乘法拟合内圆圆心半径
%%%%%%%%%用射线法获得壁厚
I=imread('C:\Users\11603\Documents\MATLAB\基于Canny算子和互相关拼接的玻璃管弯曲度检测\biaozhun.bmp');
I=im2gray(I);
%%%%%%%%%%%二值化方案
% I=im2bw(I);%%%二值化
% I=double(I);
%%%%%%%%%%开运算方案
a=strel('disk',8);
I=imopen(I,a);
figure(5)
imshow(I)
%%%%%%%%%%系数设定：t1为低阈值，t2为高阈值，迭代次数为k,具体参数需要根据图像不同而设定
%%%%%%%%%低阈值设定较高会导致噪声增加，设置较低会导致伪边缘变多
%%%%%%%%高阈值设定较高会导致迭代次数增加，设定较低会导致伪边缘变多
t1=0.1;t2=0.2;k=60;
%%%%%%%%%%设定高斯平滑算子和高斯平滑
g=[1/16 2/16 1/16;2/16 4/16 2/16;1/16 2/16 1/16];
If=conv2(I,g,'same');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%差分法求解梯度矩阵
sx=[-1 0 1;-1 0 1;-1 0 1];sy=[-1 -1 -1;0 0 0;1 1 1];
Ix=conv2(If,sx,'same');Iy=conv2(If,sy,'same');
Ig=sqrt(Ix.^2+Iy.^2);
Tg=atan2(Iy,Ix);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%非极大值抑制
[m,n]=size(Tg);
for i=1:m
    for j=1:n
    if Tg(i,j)>-pi/8&&Tg(i,j)<=pi/8
        Tg(i,j)=0;
    elseif Tg(i,j)>pi/8&&Tg(i,j)<=3*pi/8
        Tg(i,j)=1;
    elseif Tg(i,j)>3*pi/8&&Tg(i,j)<=5*pi/8
        Tg(i,j)=2; 
    elseif Tg(i,j)>5*pi/8&&Tg(i,j)<=7*pi/8
        Tg(i,j)=3;
    elseif Tg(i,j)>7*pi/8||Tg(i,j)<=-7*pi/8
        Tg(i,j)=0;
    elseif Tg(i,j)>-7*pi/8&&Tg(i,j)<=-5*pi/8
        Tg(i,j)=1; 
    elseif Tg(i,j)>-5*pi/8&&Tg(i,j)<=-3*pi/8
        Tg(i,j)=2; 
    elseif Tg(i,j)>-3*pi/8&&Tg(i,j)<=-pi/8
        Tg(i,j)=3;         
    end
    end
end
for i=2:m-1
    for j=2:n-1
    if Tg(i,j)==2
        if Ig(i,j)>Ig(i+1,j)&&Ig(i,j)>Ig(i-1,j)
            Ig(i,j)=Ig(i,j);
        else
            Ig(i,j)=0;
        end
    elseif Tg(i,j)==3
        if Ig(i,j)>Ig(i+1,j-1)&&Ig(i,j)>Ig(i-1,j+1)
            Ig(i,j)=Ig(i,j);
        else
            Ig(i,j)=0;
        end
     elseif Tg(i,j)==0
        if Ig(i,j)>Ig(i,j-1)&&Ig(i,j)>Ig(i,j+1)
            Ig(i,j)=Ig(i,j);
        else
            Ig(i,j)=0;        
        end        
     elseif Tg(i,j)==1   
         if Ig(i,j)>Ig(i-1,j-1)&&Ig(i,j)>Ig(i+1,j+1)
            Ig(i,j)=Ig(i,j);
        else
            Ig(i,j)=0;        
         end
    end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%双阈值检测
Ig1=Ig*0;Ig2=Ig*0;
th1=max(max(Ig(2:m-1,2:n-1)))*t1;
th2=max(max(Ig(2:m-1,2:n-1)))*t2;
for i=1:m
    for j=1:n
    if Ig(i,j)>=th2
       Ig2(i,j)=Ig(i,j);
    elseif Ig(i,j)>=th1&&Ig(i,j)<=th2
        Ig1(i,j)=Ig(i,j);
    end
    end
end
for i=1:m
    for j=1:n
        if Ig1(i,j)~=0
        Ig1(i,j)=Ig1(i,j)/Ig1(i,j);
        end
        if Ig2(i,j)~=0
        Ig2(i,j)=Ig2(i,j)/Ig2(i,j);
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%图像互补
for k1=1:k
for i=2:m-1
    for j=2:n-1
    if Ig1(i,j)~=0
       if Ig2(i-1,j-1)~=0||Ig2(i-1,j)~=0||Ig2(i-1,j+1)~=0||Ig2(i,j-1)~=0||Ig2(i,j+1)~=0||Ig2(i+1,j-1)~=0||Ig2(i+1,j)~=0||Ig2(i+1,j+1)~=0
       Ig2(i,j)=Ig1(i,j);
       Ig1(i,j)=0;
           else
       end
    else
    end
    end
end
end
figure(6)
imshow(Ig2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%拆解图像获得内外圆轮廓
Ig3=Ig2*0;Ig4=Ig2*0;%Ig3为内圆轮廓，Ig4为外圆轮廓
%%%%%%%%%%%%%%%%%%%%%%%%%%%%选择内圆内任意一个点（本程序选择图像中央点（m/2,n/2），通常肯定在内圆内部）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%以图像中央点为中心向上下左右四个方向引四条射线，交内圆轮廓于四个点，为阳光普照边界点
for i=round(m/2):m-10
    if Ig2(i,round(n/2))~=0
        m2=i;Ig3(i,round(n/2))=1;
        break
    end
end
for i=round(m/2):-1:10
    if Ig2(i,round(n/2))~=0
        m1=i;Ig3(i,round(n/2))=1;
        break
    end
end
for j=round(n/2):n-10
    if Ig2(round(m/2),j)~=0
        n2=j;Ig3(round(m/2),j)=1;
        break
    end
end
for j=round(n/2):-1:10
    if Ig2(round(m/2),j)~=0
        n1=j;Ig3(round(m/2),j)=1;
        break
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%以图像中央点和四个边界点组成的十字线为波前，向上下左右四个方向移动，遇到边界位置则停止移动，记录下坐标，即内部阳光普照法获得内圆轮廓
for i=m1+1:m2-1
    for j=round(n/2):n-10
    if Ig2(i,j)~=0
     Ig3(i,j)=1;
     break
 else 
    end
    end
end
for i=m1+1:m2-1
    for j=round(n/2):-1:10
    if Ig2(i,j)~=0
     Ig3(i,j)=1;
     break
 else 
    end
    end
end
for j=n1+1:n2-1
    for i=round(m/2):m-10
    if Ig2(i,j)~=0
     Ig3(i,j)=1;
     break
 else 
    end
    end
end
for j=n1+1:n2-1
    for i=round(m/2):-1:10
    if Ig2(i,j)~=0
     Ig3(i,j)=1;
     break
 else 
    end
    end
end
%%%%%%%%%%%%画内圆轮廓图
figure(7)
imshow(Ig3)
writematrix(Ig3, '2.csv');
%%%%%%%%%%%%%最小二乘法内圆
xi=0;xi2=0;xi3=0;yi=0;yi2=0;yi3=0;xiyi=0;xiyi2=0;xi2yi=0;nn=0;
for i=20:n-20
    for j=20:m-20
        if Ig3(j,i)~=0
            xi=i+xi;xi2=i^2+xi2;xi3=i^3+xi3;
            xiyi=i*j+xiyi;xiyi2=i*j^2+xiyi2;xi2yi=i^2*j+xi2yi;
            yi=j+yi;yi2=j^2+yi2;yi3=j^3+yi3;
            nn=nn+1;
        else
        end
    end
end
M=nn*xi2-(xi)^2;N=nn*xiyi-(xi*yi);H=nn*xi3+nn*xiyi2-xi*(xi2+yi2);
P=nn*yi2-(yi)^2;Q=nn*yi3+nn*xi2yi-yi*(xi2+yi2);
A=(P*H-N*Q)/(N^2-M*P);B=(Q*M-N*H)/(N^2-M*P);
C=-(xi2+yi2+(A*xi)+(B*yi))/nn;
x01=-A/2;
y01=-B/2;
r01=sqrt(A^2+B^2-4*C)/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%外部阳光普照法获得外圆轮廓
for i=10:m-10
    for j=10:n-10
    if Ig2(i,j)~=0
     Ig4(i,j)=1;
     break
 else 
    end
    end
end
for i=m-10:-1:10
    for j=n-10:-1:10
    if Ig2(i,j)~=0
     Ig4(i,j)=1;
     break
 else 
    end
    end
end
for j=10:n-10
    for i=10:m-10
    if Ig2(i,j)~=0
     Ig4(i,j)=1;
     break
 else 
    end
    end
end
for j=n-10:-1:10
    for i=m-10:-1:10
    if Ig2(i,j)~=0
     Ig4(i,j)=1;
     break
 else 
    end
    end
end
%%%%%%%%%%%%%%%%画外圆轮廓图
figure(8)
imshow(Ig4)
writematrix(Ig4, '1.csv');
%%%%%%%%%%%%最小二乘法外圆
xi=0;xi2=0;xi3=0;yi=0;yi2=0;yi3=0;xiyi=0;xiyi2=0;xi2yi=0;nn=0;
for i=20:n-20
    for j=20:m-20
        if Ig4(j,i)~=0
            xi=i+xi;xi2=i^2+xi2;xi3=i^3+xi3;
            xiyi=i*j+xiyi;xiyi2=i*j^2+xiyi2;xi2yi=i^2*j+xi2yi;
            yi=j+yi;yi2=j^2+yi2;yi3=j^3+yi3;
            nn=nn+1;
        else
        end
    end
end
M=nn*xi2-(xi)^2;N=nn*xiyi-(xi*yi);H=nn*xi3+nn*xiyi2-xi*(xi2+yi2);
P=nn*yi2-(yi)^2;Q=nn*yi3+nn*xi2yi-yi*(xi2+yi2);
A=(P*H-N*Q)/(N^2-M*P);B=(Q*M-N*H)/(N^2-M*P);
C=-(xi2+yi2+(A*xi)+(B*yi))/nn;
x02=-A/2;
y02=-B/2;
r02=sqrt(A^2+B^2-4*C)/2;
%%%%%%%%%%%%%%%%%%%%%求玻璃管端面壁厚
an=1;
dr=20;%dr为R的前进间隔
R1 = [];
R2 = [];
for theta=0:0.01:2*pi
    angle(an)=theta;
    for R=r01-dr:0.01:r01+dr%尽量缩短R的前进间隔，保证能遇到边缘点
        if Ig3(round(y01)+round(R*cos(theta)),round(x01)+round(R*sin(theta)))~=0
          R1(an)=R;
           break
        elseif R==r01+dr%如果R的值取的很细还是找不到恰好的边缘点，则该角度的内轮廓半径为所有内轮廓半径的中位数，幸好点不多
            R1(an)=median(R1);
        end
    end
       for R=r02-dr:0.01:r02+dr%尽量缩短R的前进间隔，保证能遇到边缘点
        if Ig4(round(y01)+round(R*cos(theta)),round(x01)+round(R*sin(theta)))~=0
          R2(an)=R;
           break
        elseif R==r02+dr%如果R的值取的很细还是找不到恰好的边缘点，则该角度的外轮廓半径为所有外轮廓半径的中位数，幸好点不多
            R2(an)=median(R2);
        end
       end
    an=an+1;
end
D=R2-R1;
figure(9)
plot(angle,D)
title('端面圆心角与壁厚对应关系')
xlabel('圆心角/rad');ylabel('壁厚/pixels');
set(get(gca,'XLabel'),'Fontname','Times New Roman','FontWeight','bold','Fontsize',18);
set(get(gca,'YLabel'),'Fontname','Times New Roman','FontWeight','bold','Fontsize',18);
set(gca,'Fontname','Times New Roman','FontWeight','bold','Fontsize',13)
%%%%%%%%%%%%%%%%%%%%%%%%%%求最大壁厚和最小壁厚和相应位置
bmax=max(D);bmin=min(D);%最大壁厚bmax和最小壁厚bmin
xbmax1=round(y01)+R1(D==max(D))*cos(angle(D==max(D)));%最大壁厚内轮廓纵坐标
ybmax1=round(x01)+R1(D==max(D))*sin(angle(D==max(D)));%最大壁厚内轮廓横坐标
xbmax2=round(y01)+R2(D==max(D))*cos(angle(D==max(D)));%最大壁厚外轮廓纵坐标
ybmax2=round(x01)+R2(D==max(D))*sin(angle(D==max(D)));%最大壁厚外轮廓横坐标
xbmin1=round(y01)+R1(D==min(D))*cos(angle(D==min(D)));%最小壁厚内轮廓纵坐标
ybmin1=round(x01)+R1(D==min(D))*sin(angle(D==min(D)));%最小壁厚内轮廓横坐标
xbmin2=round(y01)+R2(D==min(D))*cos(angle(D==min(D)));%最小壁厚外轮廓纵坐标
ybmin2=round(x01)+R2(D==min(D))*sin(angle(D==min(D)));%最小壁厚外轮廓横坐标
%%%%%%%%%%%%%%%%%%%%%%%%%画图
figure(10)
imshow(I)
hold on
rectangle('position',[x01-r01,y01-r01,2*r01,2*r01],'Curvature',[1,1],'EdgeColor','r','LineWidth',2)
txt=['拟合内圆圆心坐标(',num2str(x01),',',num2str(y01),')直径D=',num2str(r01*2),'pixels'];
t=text(x01,y01,txt,'FontSize',15,'Color','r');
rectangle('position',[x02-r02,y02-r02,2*r02,2*r02],'Curvature',[1,1],'EdgeColor','r','LineWidth',2)
txt=['拟合外圆圆心坐标(',num2str(x02),',',num2str(y02),')直径D=',num2str(r02*2),'pixels'];
t=text(x02,y02+150,txt,'FontSize',15,'Color','r');
line([ybmax1 ybmax2],[xbmax1 xbmax2],'LineWidth',8)
txt=['最大壁厚',num2str(bmax),'pixels'];
t=text(ybmax1,xbmax1,txt,'FontSize',15,'Color','r');
line([ybmin1 ybmin2],[xbmin1 xbmin2],'LineWidth',8)
txt=['最小壁厚',num2str(bmin),'pixels'];
t=text(ybmin1,xbmin1,txt,'FontSize',15,'Color','r');
hold off

%% 玻璃管分段三维建模
% --- 1. 参数设置 ---
OD_px = 40.12*1060/30;          % 外径 (像素)
ID_px = 30.49*1060/30;          % 内径 (像素)
R_out = OD_px / 2;   % 外半径
R_in  = ID_px / 2;   % 内半径

% =========================================================================
% 1. 数据导入与参数设置
% =========================================================================
pts_End1_Out = readmatrix('1.csv');
pts_End1_In = readmatrix('2.csv');
pts_End2_Out = readmatrix('1.csv');
pts_End2_In = readmatrix('2.csv');

% --- 路径数据 ---
pathFile = 'edge_coordinates.csv';
if exist(pathFile, 'file')
    pathData = readtable(pathFile);
    step = 5; 
    X_path = pathData.X(1:step:end); 
    raw_Y = (fillmissing(pathData.TopY,'linear') + fillmissing(pathData.BottomY,'linear')) / 2;
    Y_path = smoothdata(raw_Y(1:step:end), 'rloess', 50);
else
    warning('未找到 edge_coordinates.csv，请先运行上一段提取代码。');
end

% =========================================================================
% 2. 形状与位置分离分析
% =========================================================================
num_angles = 120; % 角度分辨率

% --- 处理端面 1 ---
% 实测数据分析 (返回：自身形状, 偏心向量, 最大直径)
[R1_irr_out, R1_irr_in, Off1_irr, maxD1] = analyze_profile_separated(pts_End1_Out, pts_End1_In, num_angles);
% 完美拟合 (返回：完美椭圆形状, 实测偏心向量)
[R1_perf_out, R1_perf_in, Off1_perf] = fit_perfect_separated(pts_End1_Out, pts_End1_In, num_angles);

% --- 处理端面 2 ---
[R2_irr_out, R2_irr_in, Off2_irr, maxD2] = analyze_profile_separated(pts_End2_Out, pts_End2_In, num_angles);
[R2_perf_out, R2_perf_in, Off2_perf] = fit_perfect_separated(pts_End2_Out, pts_End2_In, num_angles);

% 定义过渡区长度 (取端面外直径/48)
L_trans1 = maxD1/48;
L_trans2 = maxD2/48;

%fprintf('端面1 偏心向量: [%.2f, %.2f]\n', Off1_irr(1), Off1_irr(2));
%fprintf('端面2 偏心向量: [%.2f, %.2f]\n', Off2_irr(1), Off2_irr(2));

% =========================================================================
% 3. 三维扫掠 (含偏心插值)
% =========================================================================
num_slices = length(X_path);
theta_grid = linspace(0, 2*pi, num_angles);

% 预分配网格
X_mesh = zeros(num_angles, num_slices);
Y_mesh_out = zeros(num_angles, num_slices);
Z_mesh_out = zeros(num_angles, num_slices);
Y_mesh_in  = zeros(num_angles, num_slices);
Z_mesh_in  = zeros(num_angles, num_slices);

% 计算路径弧长
dist_steps = sqrt([0; diff(X_path)].^2 + [0; diff(Y_path)].^2);
arc_len = cumsum(dist_steps);
total_len = arc_len(end);

for i = 1:num_slices
    current_dist = arc_len(i); % 当前位置距离起点的距离
    dist_from_end = total_len - current_dist; % 距离终点的距离
    
    % --- 初始化当前截面参数 ---
    curr_R_out = zeros(1, num_angles);
    curr_R_in  = zeros(1, num_angles);
    curr_Offset = [0, 0]; % [dY, dZ]
    
    % --- 分段逻辑 ---
    if current_dist <= L_trans1
        % 【区域1：左过渡】 实测 -> 完美
        % 归一化进度 w (0:起点 -> 1:过渡结束)
        w = current_dist / L_trans1;
        % 使用平滑的 S形曲线插值 (smoothstep)，避免转折突变
        w = w * w * (3 - 2 * w); % S形平滑
        
        % 插值形状
        curr_R_out = (1-w) * R1_irr_out + w * R1_perf_out;
        curr_R_in  = (1-w) * R1_irr_in  + w * R1_perf_in;
        % 插值偏心 (重要：偏心量也平滑过渡)
        curr_Offset = (1-w) * Off1_irr + w * Off1_perf;
        
    elseif dist_from_end <= L_trans2
        % 【区域3：右过渡】 完美 -> 实测
        % 归一化进度 w (0:进入过渡 -> 1:终点)
        w = (L_trans2 - dist_from_end) / L_trans2;
        w = w * w * (3 - 2 * w);
        
        curr_R_out = (1-w) * R2_perf_out + w * R2_irr_out;
        curr_R_in  = (1-w) * R2_perf_in  + w * R2_irr_in;
        curr_Offset = (1-w) * Off2_perf + w * Off2_irr;
        
    else
        % 【区域2：中间稳定段】 完美(左) -> 完美(右)
        % 计算在中间段的相对进度 k
        len_middle = total_len - L_trans1 - L_trans2;
        if len_middle <= 0, k = 0.5; else
            % 如果管子太短，直接取中点混合
            k = (current_dist - L_trans1) / len_middle;
        end
        
        % 形状线性过渡
        curr_R_out = (1-k) * R1_perf_out + k * R2_perf_out;
        curr_R_in  = (1-k) * R1_perf_in  + k * R2_perf_in;
        % 偏心线性过渡 (模拟内部通道的倾斜)
        curr_Offset = (1-k) * Off1_perf + k * Off2_perf;
    end
    
    % --- 坐标还原 (极坐标转笛卡尔) ---
    % 1. 外壁 (以路径中心为原点)
    local_y_out = curr_R_out .* cos(theta_grid);
    local_z_out = curr_R_out .* sin(theta_grid);
    
    % 2. 内壁 (以路径中心 + 偏心向量 为原点)
    % 这里的 Offset(1) 是 Y方向偏移, Offset(2) 是 Z方向偏移
    local_y_in  = curr_R_in .* cos(theta_grid) + curr_Offset(1);
    local_z_in  = curr_R_in .* sin(theta_grid) + curr_Offset(2);
    
    % --- 映射到全局 ---
    X_mesh(:, i) = X_path(i);
    
    % Y轴反转适应图像坐标
    Y_mesh_out(:, i) = -Y_path(i) + local_y_out; 
    Y_mesh_in(:, i)  = -Y_path(i) + local_y_in;
    
    Z_mesh_out(:, i) = local_z_out;
    Z_mesh_in(:, i)  = local_z_in;
end

% =========================================================================
% 4. 可视化
% =========================================================================
f = figure('Color', 'k', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.7]);
ax = axes('Parent', f, 'Color', 'k'); hold on; axis equal; view(3);

% 绘制外壁 (半透明)
h_out = surf(X_mesh, Z_mesh_out, Y_mesh_out, ...
    'FaceColor', [0.6 0.8 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.2, ...
    'SpecularStrength', 0.8);

% 绘制内壁 (不透明度稍高，红色，显示偏心通道)
h_in = surf(X_mesh, Z_mesh_in, Y_mesh_in, ...
    'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.6);

% 设置透明度 (Alpha Mapping)
alpha(h_out, 0.3); % 外壁较透明
alpha(h_in, 0.6);  % 内壁较不透明，增加层次感

% 绘制端面封口
fill3(X_mesh(:,1), Z_mesh_out(:,1), Y_mesh_out(:,1), [0.6 0.8 1], 'FaceAlpha', 0.3);
fill3(X_mesh(:,end), Z_mesh_out(:,end), Y_mesh_out(:,end), [0.6 0.8 1], 'FaceAlpha', 0.3);

light('Position', [0 -500 500], 'Style', 'local'); % 主光源
light('Position', [2000 500 500], 'Style', 'local'); % 补光
lighting gouraud; material shiny;
xlabel('X (Length)'); ylabel('Z (Depth)'); zlabel('Y (Height)');
title('偏心玻璃管三维重建 (内壁与外壁独立插值)');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w'); % 黑色背景
axis equal; grid on; ax.GridColor=[1 1 1]; ax.GridAlpha=0.2;

view(0, 5); % 调整视角以便观察管口的偏心情况

% 旋转动画 (取消注释可查看自动旋转)
% while true
%     camorbit(0.5, 0);
%     drawnow;
% end

% =========================================================================
% 5. 辅助函数 (核心逻辑修改：分离位置与形状)
% =========================================================================

function [R_out, R_in, offset_vec, maxD] = analyze_profile_separated(pts_out, pts_in, num_pts)
    % 清洗
    pts_out = fix_dimensions(pts_out);
    pts_in  = fix_dimensions(pts_in);
    
    % 1. 分别计算几何中心
    center_out = mean(pts_out, 1);
    center_in  = mean(pts_in, 1);
    
    % 2. 计算偏心向量 (Inner - Outer)
    offset_vec = center_in - center_out;
    
    % 3. 去除各自中心，提取纯形状 (Pure Shape)
    % 这一步保证了我们提取的 R 是相对于该轮廓自身的中心的
    out_centered = bsxfun(@minus, pts_out, center_out);
    in_centered  = bsxfun(@minus, pts_in, center_in);
    
    % 4. 计算最大外径 (用于过渡区长度)
    dists = sqrt(sum(out_centered.^2, 2));
    maxD = max(dists) * 2;
    
    % 5. 重采样
    R_out = resample_polar(out_centered, num_pts);
    R_in  = resample_polar(in_centered, num_pts);
end

function [R_perf_out, R_perf_in, offset_vec] = fit_perfect_separated(pts_out, pts_in, num_pts)
    % 拟合时，依然要保留实测数据中的偏心关系
    
    % 1. 获取中心和偏心向量 (直接复用分析逻辑)
    pts_out = fix_dimensions(pts_out);
    pts_in  = fix_dimensions(pts_in);
    center_out = mean(pts_out, 1);
    center_in  = mean(pts_in, 1);
    offset_vec = center_in - center_out;
    
    % 2. 分别拟合完美椭圆
    % 注意：拟合函数内部会自动去中心化，所以无需预处理
    [a_out, b_out, phi_out] = get_ellipse_params_no_toolbox(pts_out);
    [a_in,  b_in,  phi_in]  = get_ellipse_params_no_toolbox(pts_in);
    
    % 3. 生成完美椭圆的极坐标分布
    theta = linspace(0, 2*pi, num_pts);
    gen_ellipse = @(a, b, phi, t) sqrt( (a*b)^2 ./ ( (b*cos(t-phi)).^2 + (a*sin(t-phi)).^2 ) );
    
    R_perf_out = gen_ellipse(a_out, b_out, phi_out, theta);
    R_perf_in  = gen_ellipse(a_in,  b_in,  phi_in,  theta);
end

function [a, b, phi] = get_ellipse_params_no_toolbox(pts)
    % 基于协方差矩阵的椭圆拟合 (无工具箱版)
    center = mean(pts, 1);
    pts_c = bsxfun(@minus, pts, center);
    C = cov(pts_c); 
    [V, D] = eig(C);
    eig_vals = diag(D);
    [~, idx] = sort(eig_vals, 'descend');
    coeff = V(:, idx);
    rotated_pts = pts_c * coeff; 
    width = max(rotated_pts(:,1)) - min(rotated_pts(:,1));
    height = max(rotated_pts(:,2)) - min(rotated_pts(:,2));
    a = width / 2; b = height / 2;
    if b > a, temp = a; a = b; b = temp; end
    phi = atan2(coeff(2,1), coeff(1,1));
end

function pts_clean = fix_dimensions(pts)
    [r, c] = size(pts);
    if r > 10 && c > 10 
        [y, z] = find(pts > 0);
        pts_clean = [y, z];
    elseif r == 2 && c > 2, pts_clean = pts';
    elseif c == 3, pts_clean = pts(:, 2:3);
    elseif r == 3, pts_clean = pts(2:3, :)';
    else, pts_clean = pts;
    end
end

function R_interp = resample_polar(pts, num_angles)
    [th, r] = cart2pol(pts(:,1), pts(:,2));
    th = mod(th, 2*pi);
    [th_sorted, idx] = sort(th);
    r_sorted = r(idx);
    [th_u, ui] = unique(th_sorted);
    r_u = r_sorted(ui);
    th_pad = [th_u-2*pi; th_u; th_u+2*pi];
    r_pad = [r_u; r_u; r_u];
    grid = linspace(0, 2*pi, num_angles);
    R_interp = interp1(th_pad, r_pad, grid, 'linear', 'extrap');
end

%% 估算玻璃棒最大插入直径
% --- 1. 参数设置 ---
tube_ID = ID_px; % 玻璃管内径 (px)
csvFile = 'edge_coordinates.csv';

if ~exist(csvFile, 'file')
    error('未找到 edge_coordinates.csv，请先运行上一段提取代码。');
end

% --- 2. 读取并处理数据 ---
data = readtable(csvFile);
X = data.X;
% 填补可能存在的 NaN 空洞
Y_top = fillmissing(data.TopY, 'linear');
Y_bot = fillmissing(data.BottomY, 'linear');

% 计算实际中心线
Y_center = (Y_top + Y_bot) / 2;

% --- 3. 线性拟合求最佳插入路径 ---
% 拟合中心线
% p(1) 斜率，p(2) 截距
p = polyfit(X, Y_center, 1);
Y_ideal = polyval(p, X); % 理想的笔直玻璃棒的中心轴

% 计算中心线的偏离度 (Residuals)
deviations = Y_center - Y_ideal;

% 找到偏离的极值
max_dev = max(deviations); % 向下的最大弯曲 (图像坐标系Y向下)
min_dev = min(deviations); % 向上的最大弯曲

% 计算偏离的总幅度
total_bend = max_dev - min_dev;

% --- 4. 计算最大可插入直径 ---
max_rod_diameter = tube_ID - total_bend;

if max_rod_diameter <= 0
    fprintf('管子弯曲过大，直棒无法插入！\n');
    max_rod_diameter = 0;
end

fprintf('================ 计算结果 ================\n');
fprintf('玻璃管内径: %.2f px\n', tube_ID);
fprintf('中心线弯曲幅度(极差): %.2f px\n', total_bend);
fprintf('------------------------------------------\n');
fprintf('【直玻璃棒最大直径】: %.2f px\n', max_rod_diameter);
fprintf('==========================================\n');

% --- 5. 可视化分析 ---
figure('Name', '直玻璃棒插入模拟分析', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);

% 子图1
subplot(2, 1, 1);
hold on;
% 绘制管壁
plot(X, Y_top, 'w-', 'LineWidth', 1);
plot(X, Y_bot, 'w-', 'LineWidth', 1);
% 绘制实际中心线
plot(X, Y_center, 'b--', 'LineWidth', 1);
% 绘制理想直线轴
plot(X, Y_ideal, 'g-', 'LineWidth', 1.5);
title('玻璃管原始弯曲情况');
legend('上内壁', '下内壁', '实际中心线', '理想直线轴');
axis equal; grid on;
set(gca, 'YDir', 'reverse');

% 子图2
subplot(2, 1, 2);
hold on;

% 将所有数据减去理想直线，相当于把玻璃棒水平放置
flat_top = Y_top - Y_ideal;
flat_bot = Y_bot - Y_ideal;
flat_center = Y_center - Y_ideal;

% 绘制拉直后的管壁
fill([X; flipud(X)], [flat_top; flipud(flat_bot)], [0.3 0.3 0.9], 'EdgeColor', 'none'); % 白色背景即管内
plot(X, flat_top, 'w-', 'LineWidth', 1);
plot(X, flat_bot, 'w-', 'LineWidth', 1);
plot(X, flat_center, 'b--', 'LineWidth', 0.5);

% 在"拉直"坐标系中：
% 上管壁最"低"的点 (Y值最大) 是限制上边界的瓶颈，即玻璃棒的上表面所有位置都应低于上管壁最"低"的点
% 下管壁最"高"的点 (Y值最小) 是限制下边界的瓶颈，即玻璃棒的下表面所有位置都应高于下管壁最"高"的点

% 计算受限边界
limit_top = max(flat_top); % 上壁的最下突起 (瓶颈)
limit_bot = min(flat_bot); % 下壁的最上突起 (瓶颈)

% 绘制直棒区域
y_rod_top = limit_top; 
y_rod_bottom = limit_bot;
h_rod = fill([X(1) X(end) X(end) X(1)], ...
             [y_rod_top y_rod_top y_rod_bottom y_rod_bottom], ...
             'g', 'FaceAlpha', 0.4, 'EdgeColor', 'g');

% 标注瓶颈点
plot(X(flat_top == limit_top), limit_top, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(X(flat_bot == limit_bot), limit_bot, 'rx', 'MarkerSize', 10, 'LineWidth', 2);

title(sprintf('有效通过区域分析 (直径=%.2fpx)', max_rod_diameter));
ylabel('相对中心轴的偏差 (px)');
xlabel('管长 (px)');
yline(0, 'k--');
legend([h_rod], '玻璃棒');
grid on;
set(gca, 'YDir', 'reverse');

% 保存结果图
saveas(gcf, 'rod_insertion_analysis.png');