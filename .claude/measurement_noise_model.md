# 量測噪聲模型研究整理

## 目標

在模擬中建構真實的量測噪聲模型，使控制器能在接近實際情況的環境下進行測試。

具體需求：
- 量測噪聲 σ_measurement 如何隨磁珠 z 軸高度（離焦距離）變化
- 模型要反映 Menq 實驗室使用的 off-focus imaging 追蹤系統特性

---

## 核心概念

### Off-focus Imaging 原理

1. 當磁珠在焦點上：影像清晰、邊緣銳利
2. 當磁珠離開焦點：影像模糊、出現同心圓繞射環
3. 不同 z 高度 → 不同的繞射環大小和強度分布 → 不同的量測解析度

### 重要釐清

- **離焦距離 z_defocus**：磁珠與焦平面的距離，影響量測解析度
- **牆面距離 h**：磁珠中心到牆面的距離，影響 Wall Effect（流體力學）
- **這兩者是獨立的概念**

---

## 主要論文來源

### 1. Zhang & Menq (2008) - Applied Optics 47, 2361-2370
- 標題：Three-dimensional particle tracking with subnanometer resolution using off-focus images
- 內容：原始 RV（Radius Vector）演算法，首次提出解析度的理論分析
- 連結：https://opg.optica.org/ao/abstract.cfm?uri=ao-47-13-2361

### 2. Zhang & Menq (2009) - JOSA A 26, 1484-1493
- 標題：Best linear unbiased axial localization in three-dimensional fluorescent bead tracking with subnanometer resolution using off-focus images
- 內容：改進版 NBLUE 演算法，更完整的解析度公式
- 連結：https://opg.optica.org/josaa/abstract.cfm?uri=josaa-26-6-1484

### 3. Cheng, Jhiang & Menq (2013) - Applied Optics 52, 7530-7539
- 標題：Real-time visual sensing system achieving high-speed 3D particle tracking with nanometer resolution
- 內容：FPGA 實現，高速追蹤系統
- 連結：https://opg.optica.org/ao/abstract.cfm?uri=ao-52-31-7530

### 4. Meng & Menq (2023) - IEEE/ASME Trans. Mechatronics 28(1), 280-291
- 標題：Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions
- 內容：完整控制系統，包含視覺追蹤的實驗數據
- 位置：reference/Ultra-Precise_High-Speed_Untethered_Manipulation_of_Magnetic_Scanning_Microprobe_in_Aqueous_Solutions.pdf

---

## 數學公式

### 主公式（2009 年論文 Equation 19）

```
σ_z(z) = g̃ · S̃(z) · σ_√I
```

### 參數說明

| 參數 | 符號 | 說明 | 如何取得 |
|------|------|------|----------|
| z 軸解析度 | σ_z(z) | 位置估計的標準差 (nm) | 公式計算 |
| 光漂白增益 | g̃ | 隨光漂白增加，初始 ≈ 1 | 模擬中設為 1 |
| 敏感度係數 | S̃(z) | 位置相關，關鍵參數 | 從校準數據或論文反推 |
| 像素噪聲 | σ_√I | √(ζ/4) ≈ 0.206 | 由相機增益計算 |

### 敏感度係數（2009 年論文 Equation 20）

```
S̃(z) = ψ̄(z) · ‖ρ̃'(z)‖^(-1)
```

| 符號 | 意義 |
|------|------|
| ψ̄(z) | VE 半徑向量的範數（代表影像「強度」）|
| ρ̃'(z) | NVE 模型對 z 的導數（代表影像「變化率」）|

**物理意義：**
- ‖ρ̃'(z)‖ 大 → 影像隨 z 變化快 → S̃ 小 → 解析度好
- ‖ρ̃'(z)‖ 小 → 影像隨 z 變化慢 → S̃ 大 → 解析度差

---

## 實驗數據

### 2009 年論文數據（0.75 μm 螢光粒子，120× 放大）

| z_defocus (μm) | σ_z (nm) | 說明 |
|----------------|----------|------|
| 0 | 1.0 | 焦平面，影像清晰但繞射環變化慢 |
| 1 | 0.5 | 最佳區域 |
| 2 | 0.45 | 最佳區域 |
| 3 | 0.5 | 最佳區域 |
| 4 | 0.55 | 開始劣化 |
| 5 | 0.6 | 較差 |
| 6 | 0.65 | 邊界，較差 |

### 2023 年論文數據（R = 2.25 μm，60× 放大，1.6 kHz）

| 方向 | 解析度 (nm) | 備註 |
|------|-------------|------|
| x | 0.7 | 與 z 位置無關 |
| y | 0.7 | 與 z 位置無關 |
| z | 2.3 | 在工作區域中央測得 |

---

## 建模所需參數

### 第一層：必須知道的基本參數

| 參數 | 符號 | 說明 |
|------|------|------|
| 粒子半徑 | R | 磁珠大小 [μm] |
| 放大倍率 | M | 光學系統總放大率 |
| 量測範圍 | z_range | z 軸可量測的總範圍 [μm] |
| 焦平面位置 | z_focal | 相對於參考點的位置 [μm] |
| 取樣率 | f_s | 視覺系統取樣頻率 [Hz] |

### 第二層：可從論文取得的參數

| 參數 | 符號 | 典型值 | 來源 |
|------|------|--------|------|
| 相機增益 | ζ | 0.17 | 實驗測量 |
| 像素噪聲 | σ_√I | ≈ 0.206 | √(ζ/4) |
| 光漂白增益 | g̃ | 1.0 | 模擬假設 |

### 第三層：需要從數據反推的參數

S̃(z) 敏感度係數 - 需要根據論文實驗數據擬合

---

## 焦平面在工作區中央的情況

```
                    z 軸（光軸方向）
                        ↑
    ─────────────────────────────────── 量測範圍上邊界 (解析度較差)
           ↑
           │  z_defocus 大
           │
    ═══════════════════════════════════ 最佳解析度區域
           │
           │  z_defocus 適中
           │
    ───────────────────────────────────  焦平面 z_focal (解析度較差)
           │
           │  z_defocus 適中
           │
    ═══════════════════════════════════ 最佳解析度區域
           │
           │  z_defocus 大
           ↓
    ─────────────────────────────────── 量測範圍下邊界 (解析度較差)
```

離焦距離計算：
```
z_defocus = |z_bead - z_focal_plane|
```

---

## 簡化模型實現（草案）

```matlab
function [n_x, n_y, n_z] = calc_measurement_noise(z_bead, params)
    % 系統參數
    z_focal = params.z_focal;       % 焦平面位置 [μm]
    z_range = params.z_range;       % 量測範圍 [μm]
    sigma_xy = params.sigma_xy;     % x/y 解析度 [μm]
    sigma_z_center = params.sigma_z_center;  % z 解析度（中央）[μm]

    % 計算離焦距離
    z_defocus = abs(z_bead - z_focal);

    % σ_z 隨 z 變化的模型
    z_norm = z_defocus / (z_range / 2);  % 正規化到 [0, 1]
    z_norm = min(z_norm, 1);  % 限制範圍

    % 根據 2009 年論文 Figure 6 的形狀擬合
    % 在 z_norm ≈ 0.17 時最佳，邊界時較差
    if z_norm < 0.17
        % 從焦平面到最佳區域
        scale = 1.0 - 0.5 * (z_norm / 0.17);
    else
        % 從最佳區域到邊界
        scale = 0.5 + 0.5 * ((z_norm - 0.17) / 0.83)^2;
    end

    sigma_z = sigma_z_center * scale / 0.5;  % 縮放到實際數值

    % 產生量測噪聲
    n_x = sigma_xy * randn;
    n_y = sigma_xy * randn;
    n_z = sigma_z * randn;
end
```

---

## 待確認事項

1. **系統參數**
   - 粒子半徑 R = ? μm（目前 CLAUDE.md 寫 2.8 μm）
   - 放大倍率 M = ?
   - 量測範圍 z_range = ? μm
   - 焦平面位置 z_focal = ? μm

2. **數據來源選擇**
   - 方案 A：使用 2009 年論文數據（有完整曲線，但粒子較小）
   - 方案 B：使用 2023 年論文數據（粒子較大，但只有單點）
   - 方案 C：混合使用

3. **縮放關係**
   - σ_z 如何隨粒子大小 R 變化？
   - 可能需要查閱 2008 年論文的理論推導

---

## 參考文獻

1. Z. Zhang and C.-H. Menq, "Three-dimensional particle tracking with subnanometer resolution using off-focus images," Appl. Opt. 47, 2361-2370 (2008)

2. Z. Zhang and C.-H. Menq, "Best linear unbiased axial localization in three-dimensional fluorescent bead tracking with subnanometer resolution using off-focus images," J. Opt. Soc. Am. A 26, 1484-1493 (2009)

3. P. Cheng, S. M. Jhiang, and C. H. Menq, "Real-time visual sensing system achieving high-speed 3D particle tracking with nanometer resolution," Appl. Opt. 52, 7530-7539 (2013)

4. T.-M. Meng and C.-H. Menq, "Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions," IEEE/ASME Trans. Mechatronics 28(1), 280-291 (2023)
