export const meta = {
  name: 'eq17-6state-deep-comprehension',
  description: 'Deep-read every eq17-6state derivation/verification/comparison document, extract actual formulas + code mappings + caveats, then surface contradictions for follow-up discussion',
  phases: [
    { title: 'ReadDerivations', detail: 'one reader per derivation/verification/comparison doc cluster' },
    { title: 'Critic', detail: 'completeness + contradiction critic across all summaries' },
  ],
}

const ROOT = '/Users/kevin/Code/MotionControl_Simu'
const D = `${ROOT}/reference/eq17_analysis/derivation`
const INV = `${ROOT}/reference/eq17_analysis/investigations`
const VER = `${ROOT}/reference/eq17_analysis/verification`
const SH = `${ROOT}/reference/shared`

const DOC_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['doc_id','purpose','key_results','derivation_chain','assumptions','code_mapping','caveats_open_issues','relevance_to_6state'],
  properties: {
    doc_id: { type: 'string' },
    purpose: { type: 'string', description: 'what this document derives or establishes (1-2 sentences)' },
    key_results: { type: 'array', description: 'the ACTUAL final formulas / numeric values, quoted', items: { type: 'object', additionalProperties: false, required: ['name','formula_or_value'], properties: { name:{type:'string'}, formula_or_value:{type:'string'}, notes:{type:'string'} } } },
    derivation_chain: { type: 'string', description: 'the logical steps from premises to result, concise but complete' },
    assumptions: { type: 'array', items: { type: 'string' } },
    code_mapping: { type: 'string', description: 'how this maps to the 6-state code: file, function, variable names, line numbers if findable' },
    caveats_open_issues: { type: 'array', items: { type: 'string' } },
    relevance_to_6state: { type: 'string', description: 'why this matters for understanding the 6-state estimator construction' },
  },
}

phase('ReadDerivations')

const READERS = [
  { label: 'phase1-Fe-H', docs: `${D}/phase1_Fe_derivation.md`,
    extra: 'This is the F_e + H matrix derivation (726 lines). Extract: the full 6-state F_e matrix, Row 3 terms (F_dx, dF_dx), the H matrix, the Eq.18 vs Eq.19 form distinction, the epsilon MA(2) structure, linearization-at-truth.' },
  { label: 'phase2-Cdpmr-Cn-IFvar', docs: `${D}/phase2_C_dpmr_C_n_derivation.md , ${D}/phase2_IF_var_dpr_derivation.md`,
    extra: 'Extract: closed-form C_dpmr, C_n (both simplified a_pd->0 AND full a_pd-dependent if present), IF_var (Option A MA(2) value), the sigma2_dxr = C_dpmr*4kBT*a + C_n*sigma2_n_s grounding, ARMA inflation.' },
  { label: 'phase3-algebraic', docs: `${D}/phase3_algebraic_verification.md`,
    extra: 'Algebraic verification phase. Extract what was algebraically verified and the cross-checks/identities confirmed.' },
  { label: 'phase4-observability', docs: `${D}/phase4_observability_rank.md`,
    extra: 'Observability rank analysis. Extract: rank results, PE conditions, whether dual measurement gives rank=full without PE, single-feedback degradation behavior.' },
  { label: 'phase5-Q', docs: `${D}/phase5_Q_matrix_derivation.md`,
    extra: 'Q matrix derivation. Extract: Q33 (all components/forms — thermal, randgain, n_x), Q55, Q66/Q77 status, the Path A/B/C distinction, K_h/K_h-prime wall sensitivity, Var(w_a) chain rule. Note any difference between this (possibly 7-state-oriented) and the committed 6-state Q.' },
  { label: 'phase6-R', docs: `${D}/phase6_R_matrix_derivation.md`,
    extra: 'R matrix derivation. Extract: R(1,1), R(2,2) intrinsic + delay term, IF_eff/IF_var, xi sensor floor per-axis, 5*Q77 vs buffered-sum delay, the 3-guard gate logic.' },
  { label: 'phase7-lyapunov', docs: `${D}/phase7_lyapunov_bench.md`,
    extra: 'Lyapunov benchmark. Extract: the closed-loop variance Lyapunov solve, C_dpmr verification via Lyapunov, tracking-std predictions, any benchmark numbers vs simulation.' },
  { label: 'Q66OL-R22-md', docs: `${D}/Q66_OL_R22_derivation.md`,
    extra: 'Q66 open-loop + R22 derivation (markdown summary; full version is the .pdf). Extract: the chain rule delta_a ~ -a*(K_h/R)*delta_h, why Q77=0 is optimal (gain variance lands on a_x slot not rate slot), R22 EWMA self-overlap finite-window correction.' },
  { label: 'R22-tex', docs: `${D}/R22_derivation.tex`,
    extra: 'Formal LaTeX R22 derivation — this is the S4-S6 source the 6-state code cites for the exact per-step IF_eff closed form. Extract: the IF_eff = 1 + 2*(...)/(...)^2 closed form, the A/B/C s-weighted autocorrelation sums, how IF_abc is defined, the delay term.' },
  { label: 'Cdpmr-tex', docs: `${D}/Cdpmr_Cn_derivation.tex`,
    extra: 'Formal LaTeX C_dpmr/C_n derivation — the FULL a_pd-dependent closed form source (gives 3.16/1.11 for the 6-state, vs simplified 3.96/1.18 for 7-state). Extract: both closed forms, the a_pd-dependent transfer function, the difference from the simplified version.' },
  { label: 'Fe-H-tex', docs: `${D}/Fe_H_derivation.tex`,
    extra: 'Formal LaTeX F_e/H derivation. Extract the formal F_e matrix and H, and any details beyond the phase1 markdown.' },
  { label: 'investigations', docs: `${INV}/Q66_physical_test_report.md , ${INV}/cdpmr_closed_form_verify_5seed.md , ${INV}/xD_suppression_finding.md , ${INV}/near_wall_gap.md , ${INV}/development_log_iir_prefill.md`,
    extra: 'Investigation reports. Extract: Q66 physical test results (ratios), C_dpmr 5-seed closed-form validation ratios, the xD suppression finding, the near-wall catastrophic divergence gap (K_h blowup, ill-conditioned P), the IIR prefill development log conclusions.' },
  { label: 'verification', docs: `${VER}/development_log_phase8.md , ${VER}/phase8_e2e_h50_results_v4.md , ${VER}/phase9_validation_report.md`,
    extra: 'End-to-end verification. Extract: phase8 e2e h50 results (tracking std, a_hat bias per axis — the 7-state bias numbers), phase9 R22 validation ratios, the development log conclusions. These are the empirical pass/fail numbers.' },
  { label: 'compare-7state-writeup', docs: `${SH}/writeup_architecture.tex`,
    extra: 'This is the 7-state (paper 2025 Eq.6) CANONICAL writeup — the COMPARISON baseline for the 6-state. Extract: the 7-state Q/R/F_e/IIR/Lyapunov treatment, the C_dpmr/C_n abstraction, and specifically the structural differences vs the 6-state eq17 path (control law, disturbance state count, F_e position). Focus on what makes 6-state different.' },
  { label: 'shared-theory', docs: `${SH}/kf_derivation.tex , ${SH}/variance_recursion.tex`,
    extra: 'Shared KF + variance-recursion theory. Extract: the generic KF/observer derivation, the time-varying Sigma_e variance recursion, rho->le, C_dpm Lyapunov. Note these are cross-controller theory.' },
  { label: 'design-spec-readme', docs: `${ROOT}/reference/eq17_analysis/design_v2.md , ${ROOT}/reference/eq17_analysis/README.md`,
    extra: 'The 6-state design spec (design_v2.md) and the eq17_analysis index README. Extract: the phase 0-7 derivation plan, the closed-form methodology principle, the document map / index of what lives where.' },
]

const summaries = await parallel(READERS.map(r => () =>
  agent(
    `Working dir ${ROOT}. Deep-read this/these document(s) to build a thorough technical understanding of the eq17-6state estimator. Read FULLY (not excerpts). Documents: ${r.docs}\n\n${r.extra}\n\nReturn structured data. In key_results, QUOTE the actual final formulas and numeric values (not vague descriptions). In code_mapping, point to the 6-state implementation (model/controller/motion_control_law_eq17_6state.m and build_eq17_6state_constants.m) where this derivation lands.`,
    { label: r.label, phase: 'ReadDerivations', schema: DOC_SCHEMA, agentType: 'Explore' }
  )
))

phase('Critic')

const CRITIC_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['contradictions','gaps','derivation_chain_map','open_questions_for_discussion'],
  properties: {
    contradictions: { type: 'array', description: 'places where two documents (or doc vs committed code) disagree on a formula/value/claim', items: { type: 'object', additionalProperties: false, required: ['topic','doc_a','doc_b','discrepancy'], properties: { topic:{type:'string'}, doc_a:{type:'string'}, doc_b:{type:'string'}, discrepancy:{type:'string'} } } },
    gaps: { type: 'array', description: 'derivation steps or topics not covered by any document', items: { type: 'string' } },
    derivation_chain_map: { type: 'string', description: 'the end-to-end logical chain: from plant model -> control law -> error dynamics -> F_e/H -> Q -> R -> Lyapunov -> validation, naming which doc owns each link' },
    open_questions_for_discussion: { type: 'array', description: 'the most important things to clarify with the user in follow-up', items: { type: 'string' } },
  },
}

const critic = await agent(
  `You are a completeness + contradiction critic. Here are structured summaries of ALL eq17-6state derivation/verification/comparison documents:\n\n${JSON.stringify(summaries.filter(Boolean), null, 1)}\n\nAlso known committed-code facts (from prior review): 6-state state = [dx1,dx2,dx3,dx_D^d,a_x,da_x]; F_e(3,3)=lambda_c; deterministic-map predict; Q33=3 components (thermal history + randgain + n_x); Q55=2*var(a_xram); Q(6,6)=0; R22 = K_var*IF_eff_exact*(a+xi)^2 + per-step buffered delay sum; C_dpmr=3.16/C_n=1.11 (full a_pd form); S=E{q r^T}=0 deferred.\n\nProduce: (1) contradictions between documents or between a document and the committed code (e.g. phase5/phase6 may describe a 7-state-oriented Q77/5*Q77 that the 6-state replaced — flag these); (2) gaps not covered; (3) a derivation_chain_map naming which doc owns each link; (4) the open questions most worth clarifying with the user before deep discussion. Be specific and quote formulas.`,
  { label: 'completeness-critic', phase: 'Critic', schema: CRITIC_SCHEMA }
)

return { summaries: summaries.filter(Boolean), critic }
