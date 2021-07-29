clear all
% close all
clc


load('res_data.csv');
% load('res_data.csv');
% load('res_data.csv');


% load('res_data_iris.mat');
% load('res_data.mat');



time   = res_data(:,1);
% reference
n1 = 1;
px_ref = res_data(:,n1+1);
py_ref = res_data(:,n1+2);
pz_ref = res_data(:,n1+3);
% q4_ref = res_data(:,n1+4);
% q5_ref = res_data(:,n1+5);
% q6_ref = res_data(:,n1+6);

% measured
n2 = n1+3;
px_mea = res_data(:,n2+1);
py_mea = res_data(:,n2+2);
pz_mea = res_data(:,n2+3);
qx_mea = res_data(:,n2+4);
qy_mea = res_data(:,n2+5);
qz_mea = res_data(:,n2+6);
qw_mea = res_data(:,n2+7);

% qd mea(may be differentiated)
% n3 = 13;
% qd1_dif = res_data(:,n3+1);
% qd2_dif = res_data(:,n3+2);
% qd3_dif = res_data(:,n3+3);
% qd4_dif = res_data(:,n3+4);
% qd5_dif = res_data(:,n3+5);
% qd6_dif = res_data(:,n3+6);
% 
% % u ref
% n4 = 19;
% u1_ref = res_data(:,n4+1);
% u2_ref = res_data(:,n4+2);
% u3_ref = res_data(:,n4+3);
% u4_ref = res_data(:,n4+4);
% u5_ref = res_data(:,n4+5);
% u6_ref = res_data(:,n4+6);
% % u measured
% n5 = 25;
% u1_mea = res_data(:,n5+1);
% u2_mea = res_data(:,n5+2);
% u3_mea = res_data(:,n5+3);
% u4_mea = res_data(:,n5+4);
% u5_mea = res_data(:,n5+5);
% u6_mea = res_data(:,n5+6);

% q estimate
% n6 = 31;
% q1_est = res_data(:,n6+1);
% q2_est = res_data(:,n6+2);
% q3_est = res_data(:,n6+3);
% q4_est = res_data(:,n6+4);
% q5_est = res_data(:,n6+5);
% q6_est = res_data(:,n6+6);
% 
% % qd estimate
% n7 = 37;
% qd1_est = res_data(:,n7+1);
% qd2_est = res_data(:,n7+2);
% qd3_est = res_data(:,n7+3);
% qd4_est = res_data(:,n7+4);
% qd5_est = res_data(:,n7+5);
% qd6_est = res_data(:,n7+6);

% dist estimate
% n8 
% d1_est = res_data(:,44);
% d2_est = res_data(:,45);
% d3_est = res_data(:,46);
% d4_est = res_data(:,47);
% d5_est = res_data(:,48);
% d6_est = res_data(:,49);
% n8 = 43;
% qdd1_est = res_data(:,n8 + 1);
% qdd2_est = res_data(:,n8 + 2);
% qdd3_est = res_data(:,n8 + 3);
% qdd4_est = res_data(:,n8 + 4);
% qdd5_est = res_data(:,n8 + 5);
% qdd6_est = res_data(:,n8 + 6);
% 
% % dist estimate
% d1_mea = 0*res_data(:,44);
% d2_mea = 0*res_data(:,45);
% d3_mea = 0*res_data(:,46);
% d4_mea = 0*res_data(:,47);
% d5_mea = 0*res_data(:,48);
% d6_mea = 0*res_data(:,49);

%
% n9 = 49;
% df1_est = res_data(:,n9+1);
% df2_est = res_data(:,n9+2);
% df3_est = res_data(:,n9+3);
% df4_est = res_data(:,n9+4);
% df5_est = res_data(:,n9+5);
% df6_est = res_data(:,n9+6);
% %
% n10 = n9+6;
% df1_real = res_data(:,n10+1);
% df2_real = res_data(:,n10+2);
% df3_real = res_data(:,n10+3);
% df4_real = res_data(:,n10+4);
% df5_real = res_data(:,n10+5);
% df6_real = res_data(:,n10+6);

% Plot positions
figure; hold on;
% title('Position Estimate');
subplot(311); hold on
% plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
plot(time, px_ref, 'r--', 'linewidth', 1.2); 
plot(time, px_mea, 'b', 'linewidth', 1.2); grid;
ylabel('${p}_x$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [-1.1 -0.9]);

subplot(312); hold on
% plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
plot(time, py_ref, 'r--', 'linewidth', 1.2); 
plot(time, py_mea , 'b', 'linewidth', 1.2); grid;
legend('Ref','Real');
ylabel('${p}_y$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [-0.2 0.2]);

subplot(313); hold on
% plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
plot(time, pz_ref, 'r--', 'linewidth', 1.2);  
plot(time, pz_mea, 'b', 'linewidth', 1.2); grid;
ylabel('${p}_z$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [-0.6 -0.4]);
h1=axes('position',[0.52 0.0001 0.0001 0.0001],'fontsize',10);axis off; %set(gcf,'color','none');set(gca,'color','none');
title('time (s)','fontsize',10)

% quaternions
% figure;
% subplot(221); hold on
% % plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
% % plot(time, q4_ref, 'r--', 'linewidth', 1.2); 
% plot(time, qx_mea, 'b', 'linewidth', 1.2); grid; %set(gca, 'ylim', [-0. 0.6]);
% ylabel('${q}_x$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [0.4 0.6]);
% 
% subplot(222); hold on
% % plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
% % plot(time, q5_ref, 'r--', 'linewidth', 1.2);  
% plot(time, qy_mea, 'b', 'linewidth', 1.2); grid;
% ylabel('${q}_y$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [-0.52 -0.48]);
% 
% subplot(223); hold on
% % plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
% % plot(time, q6_ref, 'r--', 'linewidth', 1.2); 
% plot(time, qz_mea, 'b', 'linewidth', 1.2); grid;
% ylabel('${q}_z$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [0.45 0.55]);
% 
% subplot(224); hold on
% % plot(res_data(:,[1,4,7,10]),res_data(:,[2,5,8,11]));
% % plot(time, q6_ref, 'r--', 'linewidth', 1.2); 
% plot(time, qw_mea, 'b', 'linewidth', 1.2); grid;
% ylabel('${q}_w$ (rad)','interpreter','latex','fontsize',13) ;box off;  %set(gca, 'ylim', [0.45 0.55]);
% 
% % h2=legend('Ref','Real'); set(h2,'color','none','edgecolor','white');
% h1=axes('position',[0.52 0.0001 0.0001 0.0001],'fontsize',10);axis off; %set(gcf,'color','none');set(gca,'color','none');
% title('time (s)','fontsize',10)






