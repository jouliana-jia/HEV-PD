load('UDDS_drive_cycle.mat')          %����·�ף����ﱻ��ǰ��������ֶ�������            
ts = 1;                               %ʱ����
N = length(t);                        %��ʱ�����ܽ׶���
P_dem = P_dem*1000;                   %���ʵ�λת�� KW to W
subplot(2,1,1);                       %������һ�е�ͼ
plot(t,P_dem)                       
title('Power demand')
subplot(2,1,2);
plot(t,v)
title('Desired velocity')

fl_wt_en = 0.001;      %��λ�������ĵ�ȼ������
Pe_max = 30000;        % [W] �����ȼ��������ʣ�֮����Ҫ�ĳ���ת�ٵĶ�άͼ��
Pb_max = 15000;        % [W] ����طŵ繦�ʣ�֮����Ҫ�ĳ���ת�ٵĶ�άͼ��
Pb_min = -15000;       % [W] ����س�繦�ʣ�֮����Ҫ�ĳ���ת�ٵĶ�άͼ��
Q_batt = 18000;        % [As] �������
U_oc = 320;            % [V] ��ؿ�·��ѹ
SOC_min = 0.3;         % Lower SOC limit
SOC_max = 0.8;         % Upper SOC limit
ns = 80; %SOC ��ɢ���ĸ���
SOC_grid = linspace(SOC_min,SOC_max,ns)';   %SOC ��ɢ��
% DP
V = zeros(ns,N);            %Value function �׶�ָ�꺯���������k��N��ȼ������ֵ��Ԥ�ȷ����ڴ�
V(:,N) = 0;                 %�߽�������һ�����ʱ��ȼ��������Ϊ0   

for i = N-1:-1:1            %����
    for j = 1:ns            %����SOC�ڵ�
     lb = max([(((SOC_max-SOC_grid(j))*Q_batt*U_oc)/-ts),Pb_min, P_dem(i)-Pe_max]);          %����ĳ�繦�ʣ���������
     ub = min([(((SOC_min-SOC_grid(j))*Q_batt*U_oc)/-ts),Pb_max, P_dem(i)]);                 %����ķŵ繦�ʣ��������ޣ���
     P_batt_grid = linspace(lb,ub,250);      %�ð����е�ع�����Ϊ���Ʊ�����������ɢ��
     P_eng_grid = P_dem(i) - P_batt_grid;         %��ÿ�����ܵĵ�طŵ繦�ʣ����㷢������Ҫ����Ĺ���
     c2g_grid = (ts*fl_wt_en* P_eng_grid)./(eng_eff(P_eng_grid)); %����ɱ���������ÿ��ʱ�����ڵ�ȼ��������
     SOC_next_stage = SOC_grid(j) - (ts .* P_batt_grid ./ (Q_batt*U_oc)); %����ÿ����ع����£����½׶�i����SOCj�ĺ�һ�׶�i+1����SOCֵ
     V_next_stage = interp1(SOC_grid,V(:,i+1),SOC_next_stage);%��ֵ���i+1�׶εĽ׶γɱ�
     [V(j,i), k] = min(c2g_grid + V_next_stage); %��i�׶ε�i+1�׶�ȼ������+i+1�׶εĳɱ���ȡ��Сֵ������N��i�׶ε�ȼ��������Сֵ���ҳ������ڵڼ������߱�����ȡ������С
     u_opt(j,i) = P_batt_grid(k);                %�����ڵ�j��SOC�ڵ㴦��i�׶ε����ſ���
    end
end


%��������һ��DP��SOC0�����ɵģ�����ѡ��ns��ֵ��ÿ��SOC0���ж��еĿ���·��.�൱��һ��SOC MAP�ϵ����ſ��ƶ�������ˣ�������ǲ��ȡֵ���Կռ任ʱ��

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