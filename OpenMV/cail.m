data = readmatrix('./test.xlsx');  % 直接读取数值矩阵[5,6](@ref)
% 验证数据格式
if size(data, 2) ~= 4
    error('Excel文件必须包含4列数据：原坐标系x, 原坐标系y, 目标坐标系x, 目标坐标系y');
end

% 提取源坐标系和目标坐标系点集
src_pts = data(:, 1:2);  % 前两列：原坐标系x,y
dst_pts = data(:, 3:4);  % 后两列：目标坐标系x,y

% 调用原有的仿射变换函数
T = affine_transform_2d(src_pts, dst_pts);

% 显示转换结果
fprintf('从文件 %s 成功读取 %d 个点对\n', './test.xlsx', size(data, 1));
disp('计算得到的仿射变换矩阵：');
disp(T);
function T = affine_transform_2d(src_pts, dst_pts)
% 输入: 
%   src_pts - 源坐标系点集 (n×2矩阵, 每行[x,y])
%   dst_pts - 目标坐标系点集 (n×2矩阵, 每行[X,Y])
% 输出: 
%   T - 3x3仿射变换矩阵

% 检查输入有效性
if size(src_pts,1) < 3
    error('至少需要3个对应点');
end

n = size(src_pts,1);
A = zeros(2*n, 6);
B = zeros(2*n, 1);

% 构建最小二乘方程组
for i = 1:n
    x = src_pts(i,1);
    y = src_pts(i,2);
    X = dst_pts(i,1);
    Y = dst_pts(i,2);
    
    A(2*i-1, :) = [x, y, 1, 0, 0, 0];
    A(2*i, :)   = [0, 0, 0, x, y, 1];
    
    B(2*i-1) = X;
    B(2*i)   = Y;
end

% 求解最小二乘问题
params = A \ B;

% 构建3x3变换矩阵
T = [params(1), params(2), params(3);
     params(4), params(5), params(6);
     0, 0, 1];
end