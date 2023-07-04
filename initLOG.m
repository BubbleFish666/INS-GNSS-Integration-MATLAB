%% pre-allocate vectors for logging (excluding height-related variables)
% INS
LOG.INS.v_eb_n(1:range_end-range_start+1, 1:2) = nan;
LOG.INS.eul_nb(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.eul_b0b(1:range_end-range_start+1, 1:3) = nan;
% ok... I'm too lazy to change this llh here, but it works
LOG.INS.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre_total(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre(1:range_end-range_start+1, 1:3) = nan;

% KF correction
LOG.KF.dpsi_nb(1:range_end-range_start+1, 3) = nan;
LOG.KF.dv_eb_n(1:range_end-range_start+1, 2) = nan;
LOG.KF.dllh(1:range_end-range_start+1, 2) = nan;
LOG.KF.ba(1:range_end-range_start+1, 3) = nan;
LOG.KF.bg(1:range_end-range_start+1, 3) = nan;
LOG.KF.P(1:range_end-range_start+1, 13, 13) = nan;

% INS corrected by KF error estimate
% LOG.INScorrected.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.v_eb_n_corrected(1:range_end-range_start+1, 1:2) = nan;
LOG.INS.eul_nb_corrected(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.eul_b0b_corrected(1:range_end-range_start+1, 1:3) = nan;
% ok... I'm too lazy to change this llh here, but it works
LOG.INS.llh_incre_total_corrected(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre_corrected(1:range_end-range_start+1, 1:3) = nan;
