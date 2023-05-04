%% pre-allocate vectors for logging
% INS
LOG.INS.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.v_eb_n(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.Rnb(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.Rb0b(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre_total(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre(1:range_end-range_start+1, 1:3) = nan;

% KF prediction and correction
% LOG.KF.dpsi_nb(1:2*(range_end-range_start+1), 3) = nan;
% LOG.KF.dv_eb_n(1:2*(range_end-range_start+1), 3) = nan;
% LOG.KF.dllh(1:2*(range_end-range_start+1), 3) = nan;
% LOG.KF.ba(1:2*(range_end-range_start+1), 3) = nan;
% LOG.KF.bg(1:2*(range_end-range_start+1), 3) = nan;
% 
% LOG.KF.P = cell(2*(range_end-range_start+1), 1);
% for i = 1:2*(range_end-range_start+1)
%     LOG.KF.P{i}(1:15, 1:15) = nan;
% end

% KF correction
LOG.KF.dpsi_nb(1:range_end-range_start+1, 3) = nan;
LOG.KF.dv_eb_n(1:range_end-range_start+1, 3) = nan;
LOG.KF.dllh(1:range_end-range_start+1, 3) = nan;
LOG.KF.ba(1:range_end-range_start+1, 3) = nan;
LOG.KF.bg(1:range_end-range_start+1, 3) = nan;

LOG.KF.P = cell(range_end-range_start+1, 1);
for i = 1:range_end-range_start+1
    LOG.KF.P{i}(1:15, 1:15) = nan;
end

% INS corrected by KF error estimate
% LOG.INScorrected.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.INScorrected.v_eb_n(1:range_end-range_start+1, 1:3) = nan;
LOG.INScorrected.Rnb(1:range_end-range_start+1, 1:3) = nan;
LOG.INScorrected.Rb0b(1:range_end-range_start+1, 1:3) = nan;
LOG.INScorrected.llh_incre_total(1:range_end-range_start+1, 1:3) = nan;
LOG.INScorrected.llh_incre(1:range_end-range_start+1, 1:3) = nan;
