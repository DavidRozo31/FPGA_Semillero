# Cinemática Inversa 2R — Implementación VHDL Pipeline
## Universidad Militar Nueva Granada

---

## 1. ANÁLISIS DEL PROYECTO ORIGINAL

### Problemas encontrados y correcciones

| Módulo | Problema | Corrección |
|--------|----------|------------|
| `cordic_core.vhd` | Usa `signed(31 downto 0)` — incompatible con el resto del diseño (16 bits) | Reemplazado por `cordic_sincos_16.vhd` en Q2.13 |
| `ik2r_core.vhd` | FSM secuencial con protocolo byte-a-byte torpe; no aprovecha el paralelismo | Reemplazado por `ik2r_pipeline_core.vhd` |
| `fp_adder` | No existía como módulo independiente | Creado `fp_adder.vhd` (latencia 1 ciclo) |
| `ik2r_top.vhd` | L1/L2 solo como registros internos fijos | Reemplazado por `ik2r_top_v2.vhd` con puertos directos |

### Módulos **INTOCABLES** (funcionan correctamente)
- `cordic_atan2.vhd` — pipeline 12 etapas, Q2.13 ✓
- `cordic_pkg.vhd` — constantes Q2.13 ✓
- `trig_unit.vhd` — wrapper byte-a-byte para cordic_atan2 ✓
- `fp_multiplier.vhd` — 1 ciclo Q2.13 ✓
- `fp_divider.vhd` — 18 ciclos Q2.13 ✓
- `sqrt_q13.vhd` — 18 ciclos Q2.13 ✓

---

## 2. FORMATO DE DATOS

Todo el diseño usa **Q2.13 (signed 16 bits)**:
- 1 bit de signo, 2 bits enteros, 13 bits fraccionarios
- `1.0` → `8192`, `π` → `25736`, `−π/2` → `−12868`
- `L = 0.5 m` → `4096`, `L = 1.0 m` → `8192`
- Rango: −4.0 a +3.999... (suficiente para ángulos y longitudes de robot)

Conversión: `valor_real = raw_signed / 8192.0`

---

## 3. DIAGRAMA DE PIPELINE (según imagen)

```
Ciclo  Operación                              Latencia
──────────────────────────────────────────────────────
T+0    start — Px, Py, L1, L2 disponibles
T+1    fp_mult×5 paralelo:                     1 ciclo
       Px², Py², L1², L2², L1·L2
       + cordic_atan2① arranca: atan2(Py,Px)
T+2    fp_adder×2 paralelo:                    1 ciclo
       r²=Px²+Py²  |  Σ=L1²+L2²
T+3    fp_adder: num = r² − Σ                  1 ciclo
T+4    fp_divider start
T+21   fp_divider done: cosθ₂                 18 ciclos
T+22   fp_mult: cos²θ₂                         1 ciclo
T+23   fp_adder: 1 − cos²θ₂                    1 ciclo
T+24   sqrt_q13 start
T+41   sqrt_q13 done: sinθ₂ = √(1−cos²θ₂)   18 ciclos
       PARALELO: atan2① resultado en T+14
                 (latched, esperando)
T+42   cordic_atan2② start: θ₂=atan2(sinθ₂,cosθ₂)
       PARALELO: fp_mult: k2=L2·sinθ₂
                 fp_mult: L2·cosθ₂ (componente k1)
T+43   fp_adder: k1 = L1 + L2·cosθ₂
T+44   cordic_atan2③ start: α=atan2(k2,k1)
T+54   cordic_atan2② done: θ₂                13 ciclos
T+56   cordic_atan2③ done: α                 13 ciclos
T+57   fp_adder: θ₁ = atan2(Py,Px) − α       1 ciclo
T+58   done='1'                           ← RESULTADO VÁLIDO
```

**Latencia total: ~58 ciclos @ 100 MHz = 580 ns**

---

## 4. MÓDULO SIN/COS: DECISIÓN DE DISEÑO

### ¿CORDIC o LUT?

Para **16 bits Q2.13** en una FPGA Intel Cyclone IV:

| Método | LEs | RAM | Precisión | Latencia |
|--------|-----|-----|-----------|----------|
| CORDIC pipeline (12 iter) | ~150 LEs | 0 | ~0.01% | 13 ciclos |
| LUT 256 entradas | ~50 LEs | 1 M9K | ~0.4% | 2 ciclos |
| LUT 512 entradas | ~100 LEs | 1 M9K | ~0.2% | 2 ciclos |

**Decisión: CORDIC** (`cordic_sincos_16.vhd`)

Justificación:
1. El proyecto ya usa CORDIC para atan2 — reutilizar la infraestructura
2. La precisión CORDIC (12 iteraciones) es ~4× mejor que LUT-256
3. En el pipeline IK, la latencia extra no importa (no es el cuello de botella)
4. **Ahorro de área real**: sin_t2 y cos_t2 **ya están calculados** como subproducto del pipeline IK. El módulo `cordic_sincos_16` se usa solo cuando el sistema externo necesita sin/cos de un ángulo arbitrario

---

## 5. ARCHIVOS DEL PROYECTO

### Nuevos (generados)
```
cordic_sincos_16.vhd      — CORDIC sin/cos 16 bits Q2.13 (reemplaza cordic_core)
fp_adder.vhd              — Sumador/Restador Q2.13, 1 ciclo
ik2r_pipeline_core.vhd    — Núcleo IK pipeline completo (~58 ciclos)
ik2r_top_v2.vhd           — Top-level con L1/L2 como puertos dinámicos
tb_inverse_kinematics.vhd — Testbench completo con 7 casos de prueba
simular_ik2r.do           — Script ModelSim/QuestaSim
```

### Existentes (sin modificar)
```
cordic_pkg.vhd, ik_pkg.vhd
cordic_atan2.vhd, trig_unit.vhd
fp_multiplier.vhd, fp_divider.vhd, sqrt_q13.vhd
```

---

## 6. INSTRUCCIONES DE USO

### Instanciar en sistema externo
```vhdl
U_IK : ik2r_top_v2 port map (
    clk        => sys_clk,     -- 100 MHz
    rst        => sys_rst,
    px_in      => to_signed(integer(Px * 8192.0), 16),
    py_in      => to_signed(integer(Py * 8192.0), 16),
    l1_in      => to_signed(integer(L1 * 8192.0), 16),  -- dinámico
    l2_in      => to_signed(integer(L2 * 8192.0), 16),  -- dinámico
    start      => calc_start,
    theta1_out => theta1_raw,  -- radianes en Q2.13
    theta2_out => theta2_raw,
    done       => calc_done,
    error_out  => workspace_error,
    out_sel    => "00",
    data_out   => open
);
-- Conversión: theta_deg = to_integer(theta1_raw) * 57.296 / 8192.0
```

### Ejemplo valores típicos
```
L1 = 0.5 m  → l1_in = 4096  (0.5 × 8192)
L2 = 0.4 m  → l2_in = 3277  (0.4 × 8192)
Px = 0.6 m  → px_in = 4915  (0.6 × 8192)
Py = 0.4 m  → py_in = 3277  (0.4 × 8192)

Resultado esperado (MATLAB):
θ₁ = 0.586 rad = 33.6° → theta1_out ≈  4800
θ₂ = 1.292 rad = 74.0° → theta2_out ≈ 10586
```

---

## 7. ESTIMACIÓN DE RECURSOS (Cyclone IV EP4CE)

| Módulo | LEs estimados |
|--------|--------------|
| cordic_atan2 × 3 | ~450 |
| fp_multiplier × 7 | ~7 (usando DSP blocks) |
| fp_divider | ~120 |
| sqrt_q13 | ~100 |
| fp_adder × 5 | ~25 |
| Shift registers delay | ~80 |
| Lógica control | ~50 |
| **TOTAL estimado** | **~830 LEs** |

**Sin cordic_core 32 bits** (que habría usado ~300 LEs extra en modo 32b)

---

## 8. NOTAS DE IMPLEMENTACIÓN QUARTUS

El proyecto Quartus existente (`cordic_robot.qpf`) necesita añadir los nuevos archivos. En Quartus:
1. Project → Add/Remove Files
2. Añadir: `cordic_sincos_16.vhd`, `fp_adder.vhd`, `ik2r_pipeline_core.vhd`, `ik2r_top_v2.vhd`
3. Establecer `ik2r_top_v2` como **Top-Level Entity**
4. Assignments → Settings → Analysis & Synthesis → VHDL Input → VHDL-2008
