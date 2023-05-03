% pre-allocate vectors for logging
LOG.INS.llh(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.v_eb_n(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.Rnb(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.Rb0b(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre_total(1:range_end-range_start+1, 1:3) = nan;
LOG.INS.llh_incre(1:range_end-range_start+1, 1:3) = nan;

LOG.KF.dpsi_nb(1:2*(range_end-range_start+1), 3) = nan;
LOG.KF.dv_eb_n(1:2*(range_end-range_start+1), 3) = nan;
LOG.KF.dllh(1:2*(range_end-range_start+1), 3) = nan;
LOG.KF.ba(1:2*(range_end-range_start+1), 3) = nan;
LOG.KF.bg(1:2*(range_end-range_start+1), 3) = nan;

LOG.KF.P = cell(2*(range_end-range_start+1), 1);
for i = 1:2*(range_end-range_start+1)
    LOG.KF.P{i}(1:15, 1:15) = nan;
end
