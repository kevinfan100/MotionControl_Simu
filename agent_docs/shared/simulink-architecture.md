# Simulink 模型架構

## 系統方塊圖
```
params_const (Bus:ParamsBus, Ts=inf) → [Goto params] → From params_Traj/Ctrl/Thermal/Drag/Noise
Integrator output                    → [Goto p_m]    → From p_m_Ctrl/Thermal/Drag

Clock ────────────┐                                    ┌→ p_d[k+1] → [Unit Delay IC=p0] → pd[k] ─┬→ p_d_out
From params_Traj ─┤→ Trajectory_Generator (DISCRETE 1/1600) ─┤                                        │
                                                              └→ del_pd ─┐                             │
                                                                         │                             │
del_pd ──────────────┐                                                   │                             │
pd[k] ───────────────┤                                                                                 │
Delay output ────────┤→ Controller (DISCRETE 1/1600) → f_d ───┬→ f_d_out
From params_Ctrl ────┘                                        │
                                                               │
From p_m_Ctrl ────────┐                                        │
From params_Noise ────┤→ Measurement_Noise (DISCRETE 1/1600) ──┬→ p_m_out
                                                               └→ [Delay(2, Ts) IC=p0] → Controller input 3
                                                                │
From p_m_Thermal ────┐                                          │
From params_Thermal ─┤→ Thermal_Force (DISCRETE 1/1600) → F_th ─┬→ F_th_out
                                                                 │
                          f_d + F_th → [Sum_Forces: ++] → [ZOH 1/1600]
                                                              │
F_total (from ZOH) ──┐                                       │
From p_m_Drag ────────┤→ Drag_coeff_matrix (CONTINUOUS) → p_dot
From params_Drag ─────┘                                    │
                                                           ▼
                                    p_dot → [Integrator IC=p0] → p_m ──→ [Goto p_m]
```

## Block 執行模式
| Block | ChartUpdate | SampleTime | 說明 |
|-------|------------|------------|------|
| Trajectory_Generator | DISCRETE | 1/1600 | 離散軌跡產生 |
| Controller | DISCRETE | 1/1600 | 離散控制律 |
| Thermal_Force | DISCRETE | 1/1600 | 離散隨機力產生 |
| Measurement_Noise | DISCRETE | 1/1600 | 量測噪聲注入（可開關） |
| Delay(2, Ts) | DISCRETE | 1/1600 | 感測器延遲模擬（2 步），IC=p0 |
| Drag coefficient matrix | CONTINUOUS | 0 | 連續動力學（每 solver step 更新） |
| Integrator | Continuous (ODE) | - | 連續積分，IC=p0 (workspace) |

## Solver 設定
- Solver: Fixed-step ode4 (Runge-Kutta 4th order), step size = 1e-6
- 離散取樣率: 1600 Hz (Ts = 1/1600 sec)

## 資料記錄（ToWorkspace）
| Block 名稱 | VariableName | SampleTime | 說明 |
|-----------|-------------|------------|------|
| p_d_out | p_d_out | 1/1600 | 期望軌跡（離散） |
| f_d_out | f_d_out | 1/1600 | 控制力（離散） |
| F_th_out | F_th_out | 1/1600 | 熱力（離散） |
| p_m_out | p_m_out | 1/1600 | 量測位置（離散，經 Measurement_Noise 後、Delay 前） |
