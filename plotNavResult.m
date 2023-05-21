% plot navigation results
close all

%% plot roll pitch yaw
figure('Name', 'roll-pitch-yaw')
subplot(3, 1, 1)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 1), ...
     t(data_range), LOG.INS.eul_b0b_fedback(:, 1), ...
     t(data_range), LOG.INS.eul_b0b_corrected(:, 1), ...
     t(data_range), ref.eulb0b(:, 1))
legend('yaw INS', 'yaw INS fedback', 'yaw INS corrected', 'yaw ref')

subplot(3, 1, 2)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 2), ...
     t(data_range), LOG.INS.eul_b0b_fedback(:, 2), ...
     t(data_range), LOG.INS.eul_b0b_corrected(:, 2), ...
     t(data_range), ref.eulb0b(:, 2))
legend('pitch INS', 'pitch INS fedback', 'pitch INS corrected', 'pitch ref')

subplot(3, 1, 3)
hold on
grid on
plot(t(data_range), LOG.INS.eul_b0b(:, 3), ...
     t(data_range), LOG.INS.eul_b0b_fedback(:, 3), ...
     t(data_range), LOG.INS.eul_b0b_corrected(:, 3), ...
     t(data_range), ref.eulb0b(:, 3))
legend('roll INS', 'roll INS fedback', 'roll INS corrected', 'roll ref')

%% plot INS states
figure('Name', 'INS')
subplot(4,1,1);
plot(t(data_range), LOG.INS.eul_b0b(:, 1), t(data_range), ref.eulb0b(:, 1))
title('yaw')
legend('yaw INS (deg)', 'yaw ref (deg)')
grid on
hold on

subplot(4,1,2);
plot(t(data_range), LOG.INS.v_eb_n(:, 1), '.',...
     t(data_range), LOG.INS.v_eb_n(:, 2), '.',...
     t(data_range), ref.VeloN(data_range),...
     t(data_range), ref.VeloE(data_range))
title('velocity')
legend('Vn INS', 'Ve INS', 'Vn ref', 'Ve ref')
grid on
hold on

subplot(4,1,3);
% plot(t(data_range), LOG.INS.llh_incre_total(:,1) + INS.lat0_frac - (GNSS.lat_GNSS(data_range) * llh_scale - INS.lat0_int))
plot(t(data_range), LOG.INS.llh_incre_total(:, 1), ...
     t(data_range), ref.lat(:) - ref.lat(1))
title('latitude increment')
legend('latitude increment INS (milli rad)', 'latitude increment ref (milli rad)')
grid on
hold on

subplot(4,1,4);
% plot(t(data_range), LOG.INS.llh_incre_total(:,2) + INS.lon0_frac - (GNSS.lon_GNSS(data_range) * llh_scale - INS.lon0_int))
plot(t(data_range), LOG.INS.llh_incre_total(:, 2), ...
     t(data_range), ref.lon(:) - ref.lon(1))
title('longitude increment')
legend('longitude increment INS (milli rad)', 'longitude increment ref (milli rad)')
grid on
hold on

%% plot Kalman Filter error states
figure('Name', 'navigation states error KF')
subplot(3, 1, 1)
hold on
grid on
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 1)))
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 2)))
plot(t(data_range), rad2deg(LOG.KF.dpsi_nb(:, 3)))
legend('d roll', 'd pitch', 'd yaw')
ylabel('deg')

subplot(3, 1, 2)
hold on
grid on
plot(t(data_range), LOG.KF.dv_eb_n(:, 1))
plot(t(data_range), LOG.KF.dv_eb_n(:, 2))
% plot(t(data_range), LOG.KF.dv_eb_n(:, 3))
legend('d vn', 'd ne')

subplot(3, 1, 3)
hold on
grid on
plot(t(data_range), LOG.KF.dllh(:, 1))
plot(t(data_range), LOG.KF.dllh(:, 2))
% plot(t(data_range), LOG.KF.dllh(:, 3))
legend('d lat (milli rad)', 'd lon (milli rad)')

figure('Name', 'IMU sensor error KF')
subplot(2, 1, 1)
hold on
grid on
plot(t(data_range), LOG.KF.ba(:, 1))
plot(t(data_range), LOG.KF.ba(:, 2))
plot(t(data_range), LOG.KF.ba(:, 3))
legend('ba x', 'ba y', 'ba z')

subplot(2, 1, 2)
hold on
grid on
plot(t(data_range), LOG.KF.bg(:, 1))
plot(t(data_range), LOG.KF.bg(:, 2))
plot(t(data_range), LOG.KF.bg(:, 3))
legend('bg x', 'bg y', 'bg z')

%% plot error covariance
figure('Name', 'navigation states error variance')
subplot(3,1,1)
plot(t(data_range), LOG.KF.P(:, 1, 1), ...
     t(data_range), LOG.KF.P(:, 2, 2), ...
     t(data_range), LOG.KF.P(:, 3, 3))
title('rotation error variance')
legend('roll error variance (rad)^2', ...
       'pitch error variance (rad)^2', ...
       'yaw error variance (rad)^2')
grid on

subplot(3,1,2)
plot(t(data_range), LOG.KF.P(:, 4, 4), ...
     t(data_range), LOG.KF.P(:, 5, 5))
title('velocity error covariance')
legend('Vn error variance (m/s)^2', ...
       'Ve error variance (m/s)^2')
grid on

subplot(3,1,3)
plot(t(data_range), LOG.KF.P(:, 6, 6), ...
     t(data_range), LOG.KF.P(:, 7, 7))
title('position error variance')
legend('latitude error variance (milli rad)^2', ...
       'longitude error variance (milli rad)^2')
grid on

figure('Name', 'IMU error variance')
subplot(2,1,1)
plot(t(data_range), LOG.KF.P(:, 8, 8), ...
     t(data_range), LOG.KF.P(:, 9, 9), ...
     t(data_range), LOG.KF.P(:, 10, 10))
title('accelerometer error variance')
legend('ba x variance (m/s^2)^2', ...
       'ba y variance (m/s^2)^2', ...
       'ba z variance (m/s^2)^2')
grid on

subplot(2,1,2)
plot(t(data_range), LOG.KF.P(:, 11, 11), ...
     t(data_range), LOG.KF.P(:, 12, 12), ...
     t(data_range), LOG.KF.P(:, 13, 13))
title('gyro error variance')
legend('bg x variance (rad/s)^2', ...
       'bg y variance (rad/s)^2', ...
       'bg z variance (rad/s)^2')
grid on

%% plot INS corrected states
figure('Name', 'INS corrected')
subplot(4,1,1);
plot(t(data_range), LOG.INS.eul_b0b_corrected(:, 1),...
     t(data_range), ref.eulb0b(:, 1))
title('yaw')
legend('yaw INS-corrected', 'yaw ref')
grid on
hold on

subplot(4,1,2);
plot(t(data_range), LOG.INS.v_eb_n_corrected(:, 1), '.',...
     t(data_range), LOG.INS.v_eb_n_corrected(:, 2), '.',...
     t(data_range), ref.VeloN(data_range),...
     t(data_range), ref.VeloE(data_range))
title('velocity')
legend('Vn INS corrected', 'Ve INS corrected', 'Vn ref', 'Ve ref')
ylim([-4 4])
grid on
hold on

subplot(4,1,3);
% plot(t(data_range),...
%      LOG.INS.llh_incre_total_corrected(:,1) + INS.lat0_frac...
%      - (GNSS.lat_GNSS(data_range) * llh_scale - INS.lat0_int))
plot(t(data_range), LOG.INS.llh_incre_total_corrected(:, 1), ...
     t(data_range), ref.lat(:) - ref.lat(1))
title('latitude increment')
legend('latitude increment INS corrected (milli rad)', ...
       'latitude increment ref (milli rad)')
grid on
hold on

subplot(4,1,4);
% plot(t(data_range),...
%      LOG.INS.llh_incre_total_corrected(:,2) + INS.lon0_frac...
%      - (GNSS.lon_GNSS(data_range) * llh_scale - INS.lon0_int))
plot(t(data_range), LOG.INS.llh_incre_total_corrected(:, 2), ...
     t(data_range), ref.lon(:) - ref.lon(1))
title('longitude incremnet')
legend('longitude increment INS corrected (milli rad)', ...
       'longitude increment ref (milli rad)')
grid on
hold on

%%
% figure('Name', 'navigation states error KF')
% subplot(3, 1, 1)
% hold on
% grid on
% plot(rad2deg(LOG.KF.dpsi_nb(:, 1)))
% plot(rad2deg(LOG.KF.dpsi_nb(:, 2)))
% plot(rad2deg(LOG.KF.dpsi_nb(:, 3)))
% legend('d roll', 'd pitch', 'd yaw')
% ylabel('deg')
% 
% subplot(3, 1, 2)
% hold on
% grid on
% plot(LOG.KF.dv_eb_n(:, 1))
% plot(LOG.KF.dv_eb_n(:, 2))
% plot(LOG.KF.dv_eb_n(:, 3))
% legend('d vn', 'd ne', 'd vd')
% 
% subplot(3, 1, 3)
% hold on
% grid on
% plot(LOG.KF.dllh(:, 1))
% plot(LOG.KF.dllh(:, 2))
% plot(LOG.KF.dllh(:, 3))
% legend('d lat', 'd lon', 'd h')
% 
% figure('Name', 'IMU sensor error KF')
% subplot(2, 1, 1)
% hold on
% grid on
% plot(LOG.KF.ba(:, 1))
% plot(LOG.KF.ba(:, 2))
% plot(LOG.KF.ba(:, 3))
% legend('ba x', 'ba y', 'ba z')
% 
% subplot(2, 1, 2)
% hold on
% grid on
% plot(LOG.KF.bg(:, 1))
% plot(LOG.KF.bg(:, 2))
% plot(LOG.KF.bg(:, 3))
% legend('bg x', 'bg y', 'bg z')

% figure('Name', 'P')
% plot(k)
