
function [u_act,Pe_act,FC_act,SOC_act]=RUNHEV(SOC0,N,SOCgrd,u_opt,Pd)
    
    SOC_grid = SOCgrd;
    P_dem = Pd;
    U_oc = 320;
    ts =1;
    Q_batt = 18000;
    fl_wt_en = 0.001;
     u_act=zeros;
     Pe_act=zeros;
     FC_act=zeros;
     SOC_act=SOC0;
     SOC_act(1) = SOC0;
      
    for i = 1:N-1
        u_act(i) = interp1(SOC_grid,u_opt(:,i),SOC_act(i));%��ֵ��ȡ��ǰSOC�µ����ſ���
        Pe_act(i) = P_dem(i) - u_act(i);                   %���ſ����µ���ȼ������
        FC_act(i) = (ts*fl_wt_en*Pe_act(i))/(eng_eff(Pe_act(i))); %ȼ������
        SOC_act(i+1) = SOC_act(i) - ((ts*u_act(i))/(Q_batt*U_oc));%��һ�׶�SOC  
    end

end

