# reference/

Reference material for the MotionControl_Simu project: cross-controller
derivations, per-controller analysis trees, source papers, theses, system
diagrams, and the branch narrative index.

## Layout

```
reference/
├── README.md                       (this file)
│
├── shared/                         cross-controller derivations
│   ├── writeup_architecture.tex / .pdf    main Q/R + EKF + IIR writeup
│   ├── kf_derivation.tex
│   └── variance_recursion.tex / .pdf
│
├── eq6_analysis/                   eq6 (Paper 2025 Eq.6) controller tree
│   └── (see eq6_analysis/README.md)
│
├── eq17_analysis/                  eq17 (Paper 2023 Eq.17) controller tree
│   └── (see eq17_analysis/README.md)
│
├── branch-stories/                 historical narrative index
│   ├── sigma.md                    feat/sigma-ratio-filter exploration story
│   └── eq17.md                     test/eq17-5state-ekf exploration story
│
├── controller_paper_source/        source for the Paper:
│   ├── Estimation_and_Control.tex / .pdf
│   └── variable_naming_table.tex / .pdf
│
├── thesis/                         3 reference theses (PDF)
│
└── system_figs/                    system-level diagrams (cross-cutting)
    ├── C_del_x.png
    ├── total_block_diagram.png
    ├── thermal_force.png
    └── Drag_MATLAB.pdf
```

## Where to start

- **Most current Q/R + EKF derivation** → `shared/writeup_architecture.pdf`
- **eq6 controller (Paper 2025 Eq.6)** → `eq6_analysis/README.md`
- **eq17 controller (Paper 2023 Eq.17)** → `eq17_analysis/README.md`
- **Why this project has two controllers?** → `branch-stories/sigma.md` +
  `branch-stories/eq17.md`
- **Source paper for the controller** → `controller_paper_source/Estimation_and_Control.pdf`
- **System-level block diagram** → `system_figs/total_block_diagram.png`

## Archive Convention

Each `*_analysis/` tree carries its own `archive/` subdir for historical
material (originals of synthesized docs, deprecated experiments, snapshot
from previous cleanups). Synthesis lives in the active subdir; verbatim
originals live under `archive/`.

For the project-wide history-freeze layer, see:
- Git tags `archive/sigma-pre-cleanup`, `archive/eq17-pre-cleanup` on origin
- Offline bundles in `D:\archives\MotionControl_Simu\`
- Narrative index in `branch-stories/`
