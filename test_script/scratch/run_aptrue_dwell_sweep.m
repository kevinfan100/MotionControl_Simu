% STATUS: ACTIVE (scratch) | PURPOSE: discriminator (ii) for the Meng-vs-canon hold
%   residual of the a'_true arm 4 (2026-09-02): vary the bottom DWELL (T_sim 11.5 /
%   12.5 / 14.5 => dwell 1 / 2 / 4 s after the 10 s cosine descent), same 10 seeds.
%   Wiring check: the descent part (t <= 10.5) must be bit-identical across the three.
%   Read: E at the end and the hold-window mean vs dwell. Per-step residual => grows
%   with dwell; approach-made residual => flat.
function out = run_aptrue_dwell_sweep()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); addpath(fullfile(root, 'test_script', 'integration'));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();
    TS = [11.5 12.5 14.5]; out = struct('T_sim', TS);
    E0 = [];
    for i = 1:numel(TS)
        OV = struct('trajectory_type','osc','h_init',15,'h_bottom',2.5,'amplitude',0, ...
                    'frequency',1,'n_cycles',1,'t_hold',0.5,'t_descend_override',10,'T_sim',TS(i),'h_min',2.475);
        w0bar = OV.h_init / pc.R; [~, cp] = calc_correction_functions(w0bar); at = 1/cp;
        ws0 = 1 + w0bar - 1/((8/9)*(1 - at));
        o = struct('arm','best','config_override',OV,'scenario','deep','verbose',false,'seeds',1:10,'log_P_full',false, ...
                   'ap_known',true,'ap_known_at','est','app_known',true, ...
                   'ctrl_const_override',struct('lock_b',true,'ws0_perp',ws0,'law_exact_step',true,'pred_mean2',true));
        clear run_formC_b motion_control_law_formC_b;
        evalc('R = run_formC_b(o);');
        t = R.runs{1}.tout(:); nS = 10; E = zeros(numel(t), nS);
        for q = 1:nS; r = R.runs{q}; ad = r.a_hat_out(1,3)/r.a_bar_hat_out(1,3); E(:,q) = r.a_bar_hat_out(:,3) - r.a_true_out(:,3)/ad; end
        md = t <= 10.5; mh = t > 10.6; mlast = t > t(end) - 0.5;
        if isempty(E0); E0 = E(md,:); same = NaN; else; same = max(abs(E(md,:) - E0), [], 'all'); end
        p = polyfit(t(mh), mean(E(mh,:),2), 1);
        fprintf('T_sim %.1f (dwell %.0f s): descent identical to first run? max|diff| %s | near(7.5-10.5) %+.5f | hold mean %+.5f (SEM %.5f) | last 0.5 s %+.5f (SEM %.5f) | hold slope %+.2e/s\n', ...
            TS(i), TS(i)-10.5, mat2str(same,3), mean(E(t>=7.5 & t<=10.5,:),'all'), mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), ...
            mean(E(mlast,:),'all'), std(mean(E(mlast,:),1))/sqrt(nS), p(1));
        out.(sprintf('T%02d', round(10*TS(i)))) = struct('t', t, 'E', E);
        clear R;
    end
    save(fullfile(od, 'aptrue_dwell_sweep.mat'), '-struct', 'out', '-v7.3'); fprintf('saved aptrue_dwell_sweep.mat\n');
end
