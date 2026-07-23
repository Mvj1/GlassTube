%% 玻璃管最大可插入直棒通径分析
% 正式运行前，请在 glass_tube_default_config.m 中配置独立的左右端面图像。
clear; clc; close all;

cfg = glass_tube_default_config();
result = run_glass_tube_analysis(cfg);
