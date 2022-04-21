load('UDDS_drive_cycle.mat')          %加载路谱，这里被提前计算好了轮端需求功率            
ts = 1;                               %时间间隔
N = length(t);                        %总时长，总阶段数
P_dem = P_dem*1000;                   %功率单位转换 KW to W
subplot(2,1,1);                       %画两行一列的图
plot(t,P_dem)                       
title('Power demand')
subplot(2,1,2);
plot(t,v)
title('Desired velocity')

fl_wt_en = 0.001;      %单位做功消耗的燃油质量
Pe_max = 30000;        % [W] 最大内燃机输出功率，之后需要改成与转速的二维图表
Pb_max = 15000;        % [W] 最大电池放电功率，之后需要改成与转速的二维图表
Pb_min = -15000;       % [W] 最大电池充电功率，之后需要改成与转速的二维图表
Q_batt = 18000;        % [As] 电池容量
U_oc = 320;            % [V] 电池开路电压
SOC_min = 0.3;         % Lower SOC limit
SOC_max = 0.8;         % Upper SOC limit
ns = 80; %SOC 离散化的个数
SOC_grid = linspace(SOC_min,SOC_max,ns)';   %SOC 离散化
% DP
V = zeros(ns,N);            %Value function 阶段指标函数，代表从k到N的燃油消耗值，预先分配内存
V(:,N) = 0;                 %边界条件，一般最后时刻燃油消耗量为0   

for i = N-1:-1:1            %逆序
    for j = 1:ns            %遍历SOC节点
     lb = max([(((SOC_max-SOC_grid(j))*Q_batt*U_oc)/-ts),Pb_min, P_dem(i)-Pe_max]);          %允许的充电功率，功率下限
     ub = min([(((SOC_min-SOC_grid(j))*Q_batt*U_oc)/-ts),Pb_max, P_dem(i)]);                 %允许的放电功率，功率上限，，
     P_batt_grid = linspace(lb,ub,250);      %该案例中电池功率作为控制变量，将其离散化
     P_eng_grid = P_dem(i) - P_batt_grid;         %对每个可能的电池放电功率，计算发动机需要输出的功率
     c2g_grid = (ts*fl_wt_en* P_eng_grid)./(eng_eff(P_eng_grid)); %计算成本函数，即每个时间间隔内的燃油消耗量
     SOC_next_stage = SOC_grid(j) - (ts .* P_batt_grid ./ (Q_batt*U_oc)); %计算每个电池功率下，当下阶段i当下SOCj的后一阶段i+1处的SOC值
     V_next_stage = interp1(SOC_grid,V(:,i+1),SOC_next_stage);%插值获得i+1阶段的阶段成本
     [V(j,i), k] = min(c2g_grid + V_next_stage); %求i阶段到i+1阶段燃油消耗+i+1阶段的成本再取最小值，即从N到i阶段的燃油消耗最小值，找出来是在第几个决策变量下取到的最小
     u_opt(j,i) = P_batt_grid(k);                %计算在第j个SOC节点处第i阶段的最优控制
    end
end


%计算完这一步DP，SOC0是自由的，可能选择ns个值，每个SOC0都有独有的控制路线.相当于一张SOC MAP上的最优控制都求出来了，后面就是查表取值，以空间换时间

[Pb_07, Pe_07, FC_07, SOC_07]= RUNHEV(0.7,N,SOC_grid,u_opt,P_dem);
[Pb_05, Pe_05, FC_05, SOC_05]= RUNHEV(0.5,N,SOC_grid,u_opt,P_dem);
[Pb_03, Pe_03, FC_03, SOC_03]= RUNHEV(0.3,N,SOC_grid,u_opt,P_dem);
figure;
plot(SOC_07)
hold on;
plot(SOC_05)
plot(SOC_03)
title('SOC')
legend('SOC 0.7','SOC 0.5','SOC 0.3')