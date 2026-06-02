<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>VHDL — Guía de Referencia</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:ital,wght@0,400;0,600;1,400&family=Fraunces:opsz,wght@9..144,300;9..144,600&family=DM+Sans:wght@300;400;500&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #0e0f14;
    --surface: #161820;
    --surface2: #1e2028;
    --border: rgba(255,255,255,0.07);
    --border2: rgba(255,255,255,0.13);
    --text: #e8e9f0;
    --muted: #8a8d9e;
    --accent: #7c6aff;
    --accent2: #4cc9a4;
    --accent3: #f0a05a;
    --accent4: #e05c7a;
    --code-bg: #12131a;
    --keyword: #c792ea;
    --string: #c3e88d;
    --comment: #546e7a;
    --type: #82aaff;
    --number: #f78c6c;
    --operator: #89ddff;
    --func: #82aaff;
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'DM Sans', sans-serif;
    font-size: 15px;
    line-height: 1.75;
    min-height: 100vh;
  }

  /* Layout */
  .shell { display: flex; min-height: 100vh; }

  /* Sidebar */
  .sidebar {
    width: 260px;
    min-width: 260px;
    background: var(--surface);
    border-right: 1px solid var(--border);
    padding: 2rem 0;
    position: sticky;
    top: 0;
    height: 100vh;
    overflow-y: auto;
    display: flex;
    flex-direction: column;
  }
  .sidebar-logo {
    padding: 0 1.5rem 2rem;
    border-bottom: 1px solid var(--border);
    margin-bottom: 1.5rem;
  }
  .sidebar-logo .chip {
    display: inline-block;
    background: linear-gradient(135deg, var(--accent) 0%, var(--accent2) 100%);
    color: white;
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 0.1em;
    padding: 3px 10px;
    border-radius: 4px;
    margin-bottom: 0.6rem;
  }
  .sidebar-logo h1 {
    font-family: 'Fraunces', serif;
    font-size: 22px;
    font-weight: 600;
    color: var(--text);
    line-height: 1.2;
  }
  .sidebar-logo p {
    font-size: 12px;
    color: var(--muted);
    margin-top: 4px;
  }

  .nav-section {
    padding: 0 0.75rem;
    margin-bottom: 0.25rem;
  }
  .nav-label {
    font-size: 10px;
    font-weight: 500;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--muted);
    padding: 0.4rem 0.75rem;
    margin-bottom: 0.15rem;
  }
  .nav-item {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 7px 12px;
    border-radius: 7px;
    cursor: pointer;
    font-size: 13.5px;
    color: var(--muted);
    text-decoration: none;
    transition: all 0.15s;
    border: none;
    background: none;
    width: 100%;
    text-align: left;
  }
  .nav-item:hover { background: rgba(255,255,255,0.05); color: var(--text); }
  .nav-item.active { background: rgba(124,106,255,0.15); color: var(--accent); }
  .nav-item .dot {
    width: 5px; height: 5px;
    border-radius: 50%;
    background: currentColor;
    opacity: 0.5;
    flex-shrink: 0;
  }
  .nav-item.active .dot { opacity: 1; }

  /* Main content */
  .main {
    flex: 1;
    max-width: 860px;
    padding: 3rem 3.5rem;
    overflow-x: hidden;
  }

  /* Section */
  .section { display: none; }
  .section.active { display: block; }

  .section-header {
    margin-bottom: 2.5rem;
    padding-bottom: 1.5rem;
    border-bottom: 1px solid var(--border);
  }
  .section-tag {
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    color: var(--accent2);
    letter-spacing: 0.08em;
    text-transform: uppercase;
    margin-bottom: 0.5rem;
  }
  .section-header h2 {
    font-family: 'Fraunces', serif;
    font-size: 32px;
    font-weight: 600;
    color: var(--text);
    line-height: 1.15;
  }
  .section-header p {
    color: var(--muted);
    margin-top: 0.5rem;
    font-size: 14.5px;
    max-width: 600px;
    line-height: 1.65;
  }

  /* Prose */
  p { color: var(--muted); font-size: 14.5px; line-height: 1.75; margin-bottom: 1rem; }
  p strong { color: var(--text); font-weight: 500; }

  h3 {
    font-family: 'Fraunces', serif;
    font-size: 19px;
    font-weight: 600;
    color: var(--text);
    margin: 2rem 0 0.8rem;
  }
  h4 {
    font-size: 13px;
    font-weight: 500;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--accent3);
    margin: 1.5rem 0 0.5rem;
    font-family: 'JetBrains Mono', monospace;
  }

  /* Code blocks */
  .code-block {
    background: var(--code-bg);
    border: 1px solid var(--border);
    border-radius: 10px;
    margin: 1rem 0 1.5rem;
    overflow: hidden;
  }
  .code-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 14px;
    border-bottom: 1px solid var(--border);
    background: rgba(255,255,255,0.025);
  }
  .code-header .lang {
    font-family: 'JetBrains Mono', monospace;
    font-size: 11px;
    color: var(--muted);
    letter-spacing: 0.04em;
  }
  .copy-btn {
    background: none;
    border: 1px solid var(--border2);
    color: var(--muted);
    font-size: 11px;
    padding: 3px 10px;
    border-radius: 5px;
    cursor: pointer;
    font-family: 'DM Sans', sans-serif;
    transition: all 0.15s;
  }
  .copy-btn:hover { border-color: var(--accent); color: var(--accent); }
  .copy-btn.copied { border-color: var(--accent2); color: var(--accent2); }

  pre {
    padding: 1.2rem 1.4rem;
    overflow-x: auto;
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    line-height: 1.7;
    color: #cdd3de;
  }

  /* Syntax highlight classes */
  .kw { color: var(--keyword); }
  .tp { color: var(--type); }
  .st { color: var(--string); }
  .cm { color: var(--comment); font-style: italic; }
  .nm { color: var(--number); }
  .op { color: var(--operator); }
  .fn { color: #82aaff; }
  .id { color: #eeffff; }
  .pn { color: var(--accent3); }

  /* Callout */
  .callout {
    border-radius: 8px;
    padding: 1rem 1.2rem;
    margin: 1rem 0;
    font-size: 13.5px;
    line-height: 1.6;
    display: flex;
    gap: 10px;
    align-items: flex-start;
  }
  .callout.info { background: rgba(124,106,255,0.08); border: 1px solid rgba(124,106,255,0.2); color: #b8b0ff; }
  .callout.tip { background: rgba(76,201,164,0.08); border: 1px solid rgba(76,201,164,0.2); color: #7ae0c2; }
  .callout.warn { background: rgba(240,160,90,0.08); border: 1px solid rgba(240,160,90,0.2); color: #f0c090; }
  .callout.danger { background: rgba(224,92,122,0.08); border: 1px solid rgba(224,92,122,0.2); color: #f0909e; }
  .callout-icon { font-size: 16px; flex-shrink: 0; margin-top: 1px; }
  .callout strong { color: currentColor; font-weight: 600; }

  /* Data type table */
  .type-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
    gap: 12px;
    margin: 1rem 0 1.5rem;
  }
  .type-card {
    background: var(--surface2);
    border: 1px solid var(--border);
    border-radius: 9px;
    padding: 14px 16px;
    transition: border-color 0.2s;
  }
  .type-card:hover { border-color: var(--border2); }
  .type-card .name {
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    font-weight: 600;
    color: var(--accent);
    margin-bottom: 5px;
  }
  .type-card .desc {
    font-size: 12.5px;
    color: var(--muted);
    line-height: 1.5;
    margin-bottom: 8px;
  }
  .type-card .lib {
    display: inline-block;
    font-size: 10px;
    font-family: 'JetBrains Mono', monospace;
    background: rgba(124,106,255,0.12);
    color: var(--accent);
    padding: 2px 7px;
    border-radius: 4px;
  }
  .type-card .lib.ieee { background: rgba(76,201,164,0.12); color: var(--accent2); }

  /* Structure diagram */
  .struct-diagram {
    background: var(--code-bg);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 1.5rem;
    margin: 1rem 0 1.5rem;
    font-family: 'JetBrains Mono', monospace;
    font-size: 13px;
    line-height: 1.9;
  }
  .struct-diagram .layer {
    padding: 10px 14px;
    border-radius: 6px;
    margin: 4px 0;
  }
  .struct-diagram .layer.l1 { background: rgba(124,106,255,0.1); border-left: 3px solid var(--accent); }
  .struct-diagram .layer.l2 { background: rgba(76,201,164,0.08); border-left: 3px solid var(--accent2); margin-left: 18px; }
  .struct-diagram .layer.l3 { background: rgba(240,160,90,0.06); border-left: 3px solid var(--accent3); margin-left: 36px; }
  .struct-diagram .layer.l4 { background: rgba(224,92,122,0.06); border-left: 3px solid var(--accent4); margin-left: 54px; }
  .struct-diagram .comment { color: var(--comment); font-style: italic; font-size: 11.5px; margin-left: 1em; }

  /* Conversion table */
  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 13px;
    margin: 1rem 0 1.5rem;
  }
  th {
    text-align: left;
    padding: 8px 12px;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--muted);
    border-bottom: 1px solid var(--border2);
    font-weight: 500;
  }
  td {
    padding: 8px 12px;
    border-bottom: 1px solid var(--border);
    color: var(--muted);
    font-family: 'JetBrains Mono', monospace;
    font-size: 12.5px;
  }
  td:first-child { color: var(--type); }
  tr:last-child td { border-bottom: none; }
  tr:hover td { background: rgba(255,255,255,0.02); }

  /* Port mode badges */
  .badge {
    display: inline-block;
    font-size: 11px;
    font-family: 'JetBrains Mono', monospace;
    padding: 2px 8px;
    border-radius: 4px;
    font-weight: 600;
  }
  .badge.in { background: rgba(76,201,164,0.12); color: var(--accent2); }
  .badge.out { background: rgba(240,160,90,0.12); color: var(--accent3); }
  .badge.inout { background: rgba(224,92,122,0.12); color: var(--accent4); }
  .badge.buffer { background: rgba(124,106,255,0.12); color: var(--accent); }

  /* Tabs */
  .tabs { display: flex; gap: 4px; margin-bottom: -1px; position: relative; z-index: 1; flex-wrap: wrap; }
  .tab-btn {
    background: none;
    border: 1px solid var(--border);
    border-bottom: none;
    color: var(--muted);
    font-size: 12.5px;
    padding: 7px 16px;
    border-radius: 7px 7px 0 0;
    cursor: pointer;
    font-family: 'JetBrains Mono', monospace;
    transition: all 0.15s;
  }
  .tab-btn:hover { color: var(--text); background: rgba(255,255,255,0.03); }
  .tab-btn.active {
    background: var(--code-bg);
    color: var(--accent);
    border-color: var(--border2);
    border-bottom-color: var(--code-bg);
  }
  .tab-pane { display: none; }
  .tab-pane.active { display: block; }

  /* Steps */
  .steps { margin: 1rem 0 1.5rem; }
  .step {
    display: flex;
    gap: 14px;
    margin-bottom: 1.2rem;
    align-items: flex-start;
  }
  .step-num {
    width: 26px; height: 26px;
    border-radius: 50%;
    background: rgba(124,106,255,0.12);
    color: var(--accent);
    font-family: 'JetBrains Mono', monospace;
    font-size: 12px;
    font-weight: 600;
    display: flex; align-items: center; justify-content: center;
    flex-shrink: 0;
    margin-top: 2px;
  }
  .step-body { flex: 1; }
  .step-title { font-weight: 500; color: var(--text); font-size: 14px; margin-bottom: 3px; }
  .step-desc { font-size: 13px; color: var(--muted); line-height: 1.6; }

  /* scrollbar */
  ::-webkit-scrollbar { width: 5px; height: 5px; }
  ::-webkit-scrollbar-track { background: transparent; }
  ::-webkit-scrollbar-thumb { background: var(--border2); border-radius: 4px; }

  /* Progress bar at top */
  .topbar {
    position: fixed; top: 0; left: 260px; right: 0; height: 2px;
    background: linear-gradient(90deg, var(--accent) 0%, var(--accent2) 100%);
    z-index: 100;
    transform-origin: left;
    transform: scaleX(0);
    transition: transform 0.3s;
  }

  @media (max-width: 900px) {
    .sidebar { display: none; }
    .topbar { left: 0; }
    .main { padding: 2rem 1.5rem; }
  }
</style>
</head>
<body>

<div class="topbar" id="progress"></div>

<div class="shell">
  <!-- Sidebar -->
  <nav class="sidebar">
    <div class="sidebar-logo">
      <div class="chip">HDL</div>
      <h1>VHDL</h1>
      <p>Guía de referencia interactiva</p>
    </div>

    <div class="nav-section">
      <div class="nav-label">Fundamentos</div>
      <button class="nav-item active" onclick="show('intro')">
        <span class="dot"></span>Introducción
      </button>
      <button class="nav-item" onclick="show('tipos')">
        <span class="dot"></span>Tipos de datos
      </button>
      <button class="nav-item" onclick="show('conversiones')">
        <span class="dot"></span>Conversiones
      </button>
    </div>

    <div class="nav-section">
      <div class="nav-label">Estructura</div>
      <button class="nav-item" onclick="show('entidad')">
        <span class="dot"></span>Entidad
      </button>
      <button class="nav-item" onclick="show('arquitectura')">
        <span class="dot"></span>Arquitectura
      </button>
      <button class="nav-item" onclick="show('senales')">
        <span class="dot"></span>Señales y variables
      </button>
    </div>

    <div class="nav-section">
      <div class="nav-label">Sentencias</div>
      <button class="nav-item" onclick="show('concurrentes')">
        <span class="dot"></span>Concurrentes
      </button>
      <button class="nav-item" onclick="show('secuenciales')">
        <span class="dot"></span>Secuenciales
      </button>
      <button class="nav-item" onclick="show('componentes')">
        <span class="dot"></span>Componentes
      </button>
    </div>

    <div class="nav-section">
      <div class="nav-label">Ejemplos</div>
      <button class="nav-item" onclick="show('ejemplos')">
        <span class="dot"></span>Ejemplos completos
      </button>
    </div>
  </nav>

  <!-- Main -->
  <main class="main" id="main">

    <!-- ─── INTRO ─── -->
    <div class="section active" id="sec-intro">
      <div class="section-header">
        <div class="section-tag">01 — Overview</div>
        <h2>¿Qué es VHDL?</h2>
        <p>Lenguaje de descripción de hardware que modela la estructura y comportamiento de circuitos digitales ejecutados en paralelo, impulsados por eventos o interrupciones.</p>
      </div>

      <div class="callout info">
        <span class="callout-icon">⚡</span>
        <div><strong>VHDL</strong> = VHSIC (<em>Very High Speed Integrated Circuit</em>) + HDL (<em>Hardware Description Language</em>). No distingue entre mayúsculas y minúsculas.</div>
      </div>

      <h3>Estructura general de un programa</h3>
      <p>Todo código VHDL se organiza en dos bloques fundamentales: la <strong>Entidad</strong> (declara puertos) y la <strong>Arquitectura</strong> (describe el comportamiento).</p>

      <div class="struct-diagram">
        <div class="layer l1"><span class="kw">LIBRARY</span> library_name; <span class="cm">-- Importar librerías</span></div>
        <div class="layer l1"><span class="kw">USE</span> library_name.package_name.all;</div>
        <br>
        <div class="layer l2"><span class="kw">ENTITY</span> <span class="id">entity_name</span> <span class="kw">IS</span></div>
        <div class="layer l3"><span class="kw">PORT</span>(</div>
        <div class="layer l4">port_name : <span class="pn">modo</span> <span class="tp">tipo</span>;</div>
        <div class="layer l3">);</div>
        <div class="layer l2"><span class="kw">END</span> <span class="id">entity_name</span>;</div>
        <br>
        <div class="layer l2"><span class="kw">ARCHITECTURE</span> <span class="id">arch_name</span> <span class="kw">OF</span> <span class="id">entity_name</span> <span class="kw">IS</span></div>
        <div class="layer l3"><span class="cm">-- Señales y constantes globales</span></div>
        <div class="layer l3"><span class="kw">BEGIN</span></div>
        <div class="layer l4"><span class="cm">-- Sentencias concurrentes</span></div>
        <div class="layer l4"><span class="kw">PROCESS</span>(lista sensitiva)</div>
        <div class="layer l4" style="padding-left:34px"><span class="cm">-- Sentencias secuenciales</span></div>
        <div class="layer l4"><span class="kw">END PROCESS</span>;</div>
        <div class="layer l3"><span class="kw">END</span> <span class="id">arch_name</span>;</div>
      </div>

      <div class="callout tip">
        <span class="callout-icon">💡</span>
        <div>Las <strong>señales</strong> son globales a la arquitectura. Las <strong>variables</strong> son locales a cada proceso. Las <strong>constantes</strong> y subprogramas pueden ser globales o locales según dónde se declaren.</div>
      </div>

      <h3>Modos de puerto</h3>
      <div class="steps">
        <div class="step">
          <div class="step-num">→</div>
          <div class="step-body">
            <div class="step-title"><span class="badge in">IN</span></div>
            <div class="step-desc">Solo lectura. La entidad recibe el valor del exterior. No puede ser leído dentro de la arquitectura.</div>
          </div>
        </div>
        <div class="step">
          <div class="step-num">←</div>
          <div class="step-body">
            <div class="step-title"><span class="badge out">OUT</span></div>
            <div class="step-desc">Solo escritura. La entidad envía el valor hacia el exterior. No puede ser leído dentro de la arquitectura.</div>
          </div>
        </div>
        <div class="step">
          <div class="step-num">↔</div>
          <div class="step-body">
            <div class="step-title"><span class="badge inout">INOUT</span></div>
            <div class="step-desc">Lectura y escritura. Puerto bidireccional (buses, memoria).</div>
          </div>
        </div>
        <div class="step">
          <div class="step-num">↑</div>
          <div class="step-body">
            <div class="step-title"><span class="badge buffer">BUFFER</span></div>
            <div class="step-desc">Salida que puede leerse internamente. Similar a OUT pero permite retroalimentación.</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ─── TIPOS ─── -->
    <div class="section" id="sec-tipos">
      <div class="section-header">
        <div class="section-tag">02 — Tipos de datos</div>
        <h2>Tipos de datos</h2>
        <p>VHDL ofrece tipos para bits, enteros, punto fijo y flotante, organizados en paquetes según la librería de origen.</p>
      </div>

      <h3>Paquete <code style="color:var(--accent2);font-family:JetBrains Mono,monospace;font-size:0.9em">standard</code> — Librería std</h3>
      <div class="callout tip"><span class="callout-icon">✓</span><div>No requiere importar ninguna librería.</div></div>

      <div class="type-grid">
        <div class="type-card">
          <div class="name">boolean</div>
          <div class="desc">Representa <code>false</code> o <code>true</code>.</div>
          <span class="lib">std</span>
        </div>
        <div class="type-card">
          <div class="name">bit</div>
          <div class="desc">Estado lógico <code>'0'</code> o <code>'1'</code>. Soporta operaciones lógicas.</div>
          <span class="lib">std</span>
        </div>
        <div class="type-card">
          <div class="name">bit_vector</div>
          <div class="desc">Vector de bits. Sin representación numérica.</div>
          <span class="lib">std</span>
        </div>
        <div class="type-card">
          <div class="name">integer</div>
          <div class="desc">Entero con signo 32-bit. Subtipos: <code>natural</code> (≥0), <code>positive</code> (>0).</div>
          <span class="lib">std</span>
        </div>
        <div class="type-card">
          <div class="name">time</div>
          <div class="desc">Para modelar retardos: <code>10 ns</code>, <code>5 us</code>.</div>
          <span class="lib">std</span>
        </div>
      </div>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — std types</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="id">flag</span>     : <span class="tp">boolean</span> := <span class="kw">false</span>;
<span class="id">a</span>        : <span class="tp">bit</span> := <span class="st">'1'</span>;
<span class="id">data</span>     : <span class="tp">bit_vector</span>(<span class="nm">3</span> <span class="kw">downto</span> <span class="nm">0</span>) := <span class="st">"1010"</span>;
<span class="id">n</span>        : <span class="tp">integer</span> := <span class="nm">532</span>;
<span class="id">contador</span> : <span class="tp">integer</span> <span class="kw">range</span> <span class="nm">0</span> <span class="kw">to</span> <span class="nm">100</span>;
<span class="id">nat</span>      : <span class="tp">natural</span> := <span class="nm">0</span>;
<span class="id">pos</span>      : <span class="tp">positive</span> := <span class="nm">1</span>;
<span class="id">t1</span>       : <span class="tp">time</span> := <span class="nm">10</span> <span class="kw">ns</span>;</pre>
      </div>

      <h3>Paquete <code style="color:var(--accent2);font-family:JetBrains Mono,monospace;font-size:0.9em">std_logic_1164</code> — Librería IEEE</h3>
      <div class="callout info"><span class="callout-icon">📦</span><div>Requiere: <code style="color:var(--accent)">library ieee; use ieee.std_logic_1164.all;</code></div></div>

      <p>Define <strong>9 estados lógicos</strong> para modelar el comportamiento físico real:</p>
      <table>
        <thead><tr><th>Estado</th><th>Descripción</th></tr></thead>
        <tbody>
          <tr><td>'U'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Uninitialized — valor por defecto</td></tr>
          <tr><td>'X'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Unknown — conflicto de drivers</td></tr>
          <tr><td>'0'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Lógica baja (fuerte)</td></tr>
          <tr><td>'1'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Lógica alta (fuerte)</td></tr>
          <tr><td>'Z'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Alta impedancia — bus tristate</td></tr>
          <tr><td>'W'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Weak unknown</td></tr>
          <tr><td>'L'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Weak 0 (pull-down)</td></tr>
          <tr><td>'H'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Weak 1 (pull-up)</td></tr>
          <tr><td>'-'</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Don't care — optimización</td></tr>
        </tbody>
      </table>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — std_logic</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">library</span> <span class="id">ieee</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">std_logic_1164</span>.<span class="kw">all</span>;

<span class="id">a</span> : <span class="tp">std_logic</span> := <span class="st">'1'</span>;
<span class="id">bus8</span> : <span class="tp">std_logic_vector</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>) := (<span class="kw">others</span> => <span class="st">'0'</span>);</pre>
      </div>

      <h3>Paquete <code style="color:var(--accent2);font-family:JetBrains Mono,monospace;font-size:0.9em">numeric_std</code> — Librería IEEE</h3>
      <div class="callout info"><span class="callout-icon">📦</span><div>Requiere: <code style="color:var(--accent)">library ieee; use ieee.numeric_std.all;</code></div></div>

      <div class="type-grid">
        <div class="type-card">
          <div class="name">unsigned</div>
          <div class="desc">Entero sin signo. n bits → [0 … 2ⁿ−1]. Permite operaciones aritméticas.</div>
          <span class="lib ieee">ieee</span>
        </div>
        <div class="type-card">
          <div class="name">signed</div>
          <div class="desc">Entero con signo (complemento a 2). MSB indica signo. n bits → [−2ⁿ⁻¹ … 2ⁿ⁻¹−1].</div>
          <span class="lib ieee">ieee</span>
        </div>
      </div>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — numeric_std</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">library</span> <span class="id">ieee</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">numeric_std</span>.<span class="kw">all</span>;

<span class="id">a</span> : <span class="tp">unsigned</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>) := <span class="st">"11111111"</span>; <span class="cm">-- 255</span>
<span class="id">b</span> : <span class="tp">signed</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>)   := <span class="st">"11111111"</span>; <span class="cm">-- -1</span></pre>
      </div>

      <h3>Punto fijo y flotante</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — fixed & float</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">library</span> <span class="id">ieee</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">fixed_pkg</span>.<span class="kw">all</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">float_pkg</span>.<span class="kw">all</span>;

<span class="cm">-- Punto fijo: 4 bits enteros + 4 fraccionales</span>
<span class="id">a</span> : <span class="tp">sfixed</span>(<span class="nm">3</span> <span class="kw">downto</span> -<span class="nm">4</span>) := <span class="fn">to_sfixed</span>(<span class="nm">1.375</span>, a'high, a'low);

<span class="cm">-- Sin signo: 3 bits enteros + 5 fraccionales</span>
<span class="id">b</span> : <span class="tp">ufixed</span>(<span class="nm">2</span> <span class="kw">downto</span> -<span class="nm">5</span>) := <span class="fn">to_ufixed</span>(<span class="nm">2.5</span>, b'high, b'low);

<span class="cm">-- Punto flotante IEEE 754</span>
<span class="id">c</span> : <span class="tp">float32</span> := <span class="fn">to_float</span>(<span class="nm">3.14</span>, <span class="tp">float32</span>);</pre>
      </div>
    </div>

    <!-- ─── CONVERSIONES ─── -->
    <div class="section" id="sec-conversiones">
      <div class="section-header">
        <div class="section-tag">03 — Conversiones</div>
        <h2>Conversiones de tipos</h2>
        <p>Las conversiones son necesarias para conectar señales de distintos tipos. VHDL es fuertemente tipado.</p>
      </div>

      <div class="callout warn">
        <span class="callout-icon">⚠</span>
        <div><strong>Regla práctica:</strong> Usa <code>std_logic_vector</code> para buses, registros y puertos. Usa <code>signed</code>/<code>unsigned</code> para aritmética.</div>
      </div>

      <h3>Conversiones entre vectores de bits</h3>
      <table>
        <thead><tr><th>Desde</th><th>Hacia</th><th>Sintaxis</th></tr></thead>
        <tbody>
          <tr><td>std_logic_vector</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">unsigned</td><td>unsigned(slv)</td></tr>
          <tr><td>std_logic_vector</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">signed</td><td>signed(slv)</td></tr>
          <tr><td>unsigned</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">std_logic_vector</td><td>std_logic_vector(u)</td></tr>
          <tr><td>signed</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">std_logic_vector</td><td>std_logic_vector(s)</td></tr>
          <tr><td>unsigned</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">integer</td><td>to_integer(u)</td></tr>
          <tr><td>signed</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">integer</td><td>to_integer(s)</td></tr>
          <tr><td>integer</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">unsigned</td><td>to_unsigned(n, bits)</td></tr>
          <tr><td>integer</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">signed</td><td>to_signed(n, bits)</td></tr>
        </tbody>
      </table>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — conversiones completas</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">library</span> <span class="id">ieee</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">std_logic_1164</span>.<span class="kw">all</span>;
<span class="kw">use</span> <span class="id">ieee</span>.<span class="id">numeric_std</span>.<span class="kw">all</span>;

<span class="kw">signal</span> <span class="id">slv</span> : <span class="tp">std_logic_vector</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">signal</span> <span class="id">u</span>   : <span class="tp">unsigned</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">signal</span> <span class="id">s</span>   : <span class="tp">signed</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">signal</span> <span class="id">n</span>   : <span class="tp">integer</span>;

<span class="cm">-- slv → unsigned / signed</span>
<span class="id">u</span> <= <span class="fn">unsigned</span>(<span class="id">slv</span>);
<span class="id">s</span> <= <span class="fn">signed</span>(<span class="id">slv</span>);

<span class="cm">-- unsigned / signed → slv</span>
<span class="id">slv</span> <= <span class="fn">std_logic_vector</span>(<span class="id">u</span>);
<span class="id">slv</span> <= <span class="fn">std_logic_vector</span>(<span class="id">s</span>);

<span class="cm">-- unsigned / signed → integer</span>
<span class="id">n</span> <= <span class="fn">to_integer</span>(<span class="id">u</span>);
<span class="id">n</span> <= <span class="fn">to_integer</span>(<span class="id">s</span>);

<span class="cm">-- integer → unsigned / signed  (especificar bits)</span>
<span class="id">u</span> <= <span class="fn">to_unsigned</span>(<span class="id">n</span>, <span class="nm">8</span>);
<span class="id">s</span> <= <span class="fn">to_signed</span>(<span class="id">n</span>, <span class="nm">8</span>);</pre>
      </div>
    </div>

    <!-- ─── ENTIDAD ─── -->
    <div class="section" id="sec-entidad">
      <div class="section-header">
        <div class="section-tag">04 — Entidad</div>
        <h2>Entidad</h2>
        <p>Describe la interfaz del circuito: sus puertos de entrada y salida. Es la "caja negra" del módulo.</p>
      </div>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — sintaxis</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">ENTITY</span> <span class="id">entity_name</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(
        <span class="id">port1_name</span> : <span class="pn">port_mode</span> <span class="tp">port_type</span>;
        <span class="id">port2_name</span> : <span class="pn">port_mode</span> <span class="tp">port_type</span>;
        <span class="id">portN_name</span> : <span class="pn">port_mode</span> <span class="tp">port_type</span>
    );
<span class="kw">END</span> <span class="id">entity_name</span>;</pre>
      </div>

      <h3>Ejemplo — Multiplexador 4:1</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — mux 4:1</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">ENTITY</span> <span class="id">Mux4to1</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(
        <span class="id">A</span>, <span class="id">B</span>, <span class="id">C</span>, <span class="id">D</span> : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">sel</span>         : <span class="kw">IN</span>  <span class="tp">std_logic_vector</span>(<span class="nm">1</span> <span class="kw">downto</span> <span class="nm">0</span>);
        <span class="id">Q</span>           : <span class="kw">OUT</span> <span class="tp">std_logic</span>
    );
<span class="kw">END</span> <span class="id">Mux4to1</span>;</pre>
      </div>

      <h3>Ejemplo — Registro de 8 bits</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — registro 8 bits</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">ENTITY</span> <span class="id">Reg8bit</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(
        <span class="id">clk</span>   : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">rst</span>   : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">en</span>    : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">D</span>     : <span class="kw">IN</span>  <span class="tp">std_logic_vector</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>);
        <span class="id">Q</span>     : <span class="kw">OUT</span> <span class="tp">std_logic_vector</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>)
    );
<span class="kw">END</span> <span class="id">Reg8bit</span>;</pre>
      </div>
    </div>

    <!-- ─── ARQUITECTURA ─── -->
    <div class="section" id="sec-arquitectura">
      <div class="section-header">
        <div class="section-tag">05 — Arquitectura</div>
        <h2>Arquitectura</h2>
        <p>Describe el comportamiento interno de la entidad. Puede existir múltiples arquitecturas para una misma entidad.</p>
      </div>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — sintaxis</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">ARCHITECTURE</span> <span class="id">arch_name</span> <span class="kw">OF</span> <span class="id">entity_name</span> <span class="kw">IS</span>
    <span class="cm">-- Declaraciones globales (signals, constants, components)</span>
<span class="kw">BEGIN</span>
    <span class="cm">-- Sentencias concurrentes</span>
<span class="kw">END</span> <span class="id">arch_name</span>;</pre>
      </div>

      <h3>Constantes</h3>
      <div class="callout tip"><span class="callout-icon">📌</span><div>Declaradas antes del <code>BEGIN</code>. No pueden modificarse durante la ejecución.</div></div>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — constants</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">CONSTANT</span> <span class="id">N_BITS</span>  : <span class="tp">integer</span> := <span class="nm">8</span>;
<span class="kw">CONSTANT</span> <span class="id">CLK_DIV</span> : <span class="tp">integer</span> := <span class="nm">50_000_000</span>; <span class="cm">-- 50 MHz</span>
<span class="kw">CONSTANT</span> <span class="id">RESET</span>   : <span class="tp">std_logic</span> := <span class="st">'0'</span>;</pre>
      </div>

      <h3>Atributos útiles</h3>
      <table>
        <thead><tr><th>Atributo</th><th>Aplica a</th><th>Descripción</th></tr></thead>
        <tbody>
          <tr><td>'event</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Señales</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">True si la señal tuvo evento en este delta</td></tr>
          <tr><td>'range</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Vectores</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Retorna el rango del vector</td></tr>
          <tr><td>'high</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Vectores</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Índice más alto</td></tr>
          <tr><td>'low</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Vectores</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Índice más bajo</td></tr>
          <tr><td>'length</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Vectores</td><td style="color:var(--muted);font-family:DM Sans,sans-serif">Cantidad de elementos</td></tr>
        </tbody>
      </table>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — atributos</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="cm">-- Flanco de subida (dos formas equivalentes)</span>
<span class="kw">IF</span> <span class="id">clk</span> = <span class="st">'1'</span> <span class="kw">AND</span> <span class="id">clk</span><span class="op">'</span><span class="fn">event</span> <span class="kw">THEN</span> ...
<span class="kw">IF</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>)       <span class="kw">THEN</span> ...  <span class="cm">-- recomendado</span>

<span class="cm">-- Iterar sobre todo el rango de un vector</span>
<span class="kw">FOR</span> <span class="id">i</span> <span class="kw">IN</span> <span class="id">wire1</span><span class="op">'</span><span class="fn">range</span> <span class="kw">LOOP</span> ...

<span class="cm">-- Tamaño dinámico</span>
<span class="id">N</span> := <span class="id">bus_data</span><span class="op">'</span><span class="fn">length</span>; <span class="cm">-- 8 para bus(7 downto 0)</span></pre>
      </div>
    </div>

    <!-- ─── SEÑALES ─── -->
    <div class="section" id="sec-senales">
      <div class="section-header">
        <div class="section-tag">06 — Señales y variables</div>
        <h2>Señales, variables y asignación</h2>
        <p>Comprender la diferencia entre señales y variables es clave para evitar bugs de simulación.</p>
      </div>

      <div class="callout danger">
        <span class="callout-icon">⚡</span>
        <div><strong>Diferencia crítica:</strong> Las señales (<code>&lt;=</code>) se actualizan al final del delta cycle. Las variables (<code>:=</code>) se actualizan inmediatamente.</div>
      </div>

      <div class="tabs">
        <button class="tab-btn active" onclick="switchTab(this,'signals')">SIGNAL</button>
        <button class="tab-btn" onclick="switchTab(this,'variables')">VARIABLE</button>
        <button class="tab-btn" onclick="switchTab(this,'assign')">Asignación</button>
      </div>
      <div class="code-block" style="border-radius: 0 8px 8px 8px;">
        <div class="tab-pane active" id="tab-signals">
          <div class="code-header"><span class="lang">vhdl — signals</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="cm">-- Declaradas en la arquitectura, antes del BEGIN</span>
<span class="kw">SIGNAL</span> <span class="id">sig1</span>  : <span class="tp">std_logic</span> := <span class="st">'0'</span>;
<span class="kw">SIGNAL</span> <span class="id">bus8</span>  : <span class="tp">std_logic_vector</span>(<span class="nm">7</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">SIGNAL</span> <span class="id">cnt</span>   : <span class="tp">unsigned</span>(<span class="nm">3</span> <span class="kw">downto</span> <span class="nm">0</span>) := (<span class="kw">others</span> => <span class="st">'0'</span>);

<span class="cm">-- Asignación de señal (ocurre en siguiente delta)</span>
<span class="id">sig1</span> <= <span class="st">'1'</span>;
<span class="id">bus8</span> <= <span class="st">"10101010"</span>;
<span class="id">cnt</span>  <= <span class="id">cnt</span> + <span class="nm">1</span>;</pre>
        </div>
        <div class="tab-pane" id="tab-variables">
          <div class="code-header"><span class="lang">vhdl — variables</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="cm">-- Declaradas dentro de PROCESS, antes del BEGIN del proceso</span>
<span class="kw">PROCESS</span>(<span class="id">clk</span>)
    <span class="kw">VARIABLE</span> <span class="id">temp</span> : <span class="tp">integer</span> := <span class="nm">0</span>;
    <span class="kw">VARIABLE</span> <span class="id">acc</span>  : <span class="tp">unsigned</span>(<span class="nm">15</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">BEGIN</span>
    <span class="cm">-- Asignación de variable (inmediata)</span>
    <span class="id">temp</span> := <span class="id">temp</span> + <span class="nm">1</span>;
    <span class="id">acc</span>  := (<span class="kw">others</span> => <span class="st">'0'</span>);
<span class="kw">END PROCESS</span>;</pre>
        </div>
        <div class="tab-pane" id="tab-assign">
          <div class="code-header"><span class="lang">vhdl — operador de asignación</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="cm">-- Inicialización (constantes, variables, señales)</span>
<span class="kw">SIGNAL</span> <span class="id">x</span> : <span class="tp">integer</span> := <span class="nm">0</span>;       <span class="cm">-- := inicializa</span>

<span class="cm">-- Asignación de señal (concurrente o dentro de proceso)</span>
<span class="id">x</span> <= <span class="nm">5</span>;                           <span class="cm">-- <= para señales</span>

<span class="cm">-- Asignación de variable (solo dentro de proceso)</span>
<span class="kw">VARIABLE</span> <span class="id">v</span> : <span class="tp">integer</span>;
<span class="id">v</span> := <span class="nm">10</span>;                          <span class="cm">-- := para variables</span>

<span class="cm">-- Concatenación de vectores</span>
<span class="id">resultado</span> <= <span class="id">MSB</span> & <span class="id">LSB</span>;         <span class="cm">-- operador &</span></pre>
        </div>
      </div>
    </div>

    <!-- ─── CONCURRENTES ─── -->
    <div class="section" id="sec-concurrentes">
      <div class="section-header">
        <div class="section-tag">07 — Sentencias concurrentes</div>
        <h2>Sentencias concurrentes</h2>
        <p>Se ejecutan en paralelo, de forma asíncrona. Modelan lógica combinacional (compuertas, multiplexadores).</p>
      </div>

      <div class="callout warn"><span class="callout-icon">⚠</span><div>Para evitar <strong>latches</strong> en <code>with-select</code> y <code>when-else</code>, siempre incluye la cláusula <code>WHEN OTHERS</code> o <code>ELSE</code> final.</div></div>

      <h3>with-select</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — with-select (MUX 4:1)</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">WITH</span> <span class="id">sel</span> <span class="kw">SELECT</span>
    <span class="id">Q</span> <= <span class="id">A</span> <span class="kw">WHEN</span> <span class="st">"00"</span>,
         <span class="id">B</span> <span class="kw">WHEN</span> <span class="st">"01"</span>,
         <span class="id">C</span> <span class="kw">WHEN</span> <span class="st">"10"</span>,
         <span class="id">D</span> <span class="kw">WHEN OTHERS</span>; <span class="cm">-- evita latch</span></pre>
      </div>

      <h3>when-else</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — when-else (MUX con condición)</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="id">Q</span> <= <span class="id">A</span> <span class="kw">WHEN</span> <span class="id">sel</span> = <span class="st">"00"</span> <span class="kw">ELSE</span>
     <span class="id">B</span> <span class="kw">WHEN</span> <span class="id">sel</span> = <span class="st">"01"</span> <span class="kw">ELSE</span>
     <span class="id">C</span> <span class="kw">WHEN</span> <span class="id">sel</span> = <span class="st">"10"</span> <span class="kw">ELSE</span>
     <span class="id">D</span>; <span class="cm">-- última condición implícita</span></pre>
      </div>

      <h3>Proceso</h3>
      <p>Los procesos se ejecutan en paralelo entre sí, pero secuencialmente dentro. Se activan cuando cambia alguna señal de la lista sensitiva.</p>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — process</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">PROCESS</span>(<span class="id">clk</span>, <span class="id">rst</span>) <span class="cm">-- lista sensitiva</span>
    <span class="cm">-- Variables locales aquí</span>
<span class="kw">BEGIN</span>
    <span class="kw">IF</span> <span class="id">rst</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
        <span class="id">Q</span> <= (<span class="kw">others</span> => <span class="st">'0'</span>);
    <span class="kw">ELSIF</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>) <span class="kw">THEN</span>
        <span class="id">Q</span> <= <span class="id">D</span>;
    <span class="kw">END IF</span>;
<span class="kw">END PROCESS</span>;</pre>
      </div>

      <h3>Sentencias de tiempo — AFTER / WAIT</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — timing</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="cm">-- AFTER (concurrente o secuencial)</span>
<span class="id">clk</span> <= <span class="st">'0'</span>, <span class="st">'1'</span> <span class="kw">AFTER</span> <span class="nm">10</span> <span class="kw">ns</span>, <span class="st">'0'</span> <span class="kw">AFTER</span> <span class="nm">20</span> <span class="kw">ns</span>;

<span class="cm">-- WAIT (solo secuencial)</span>
<span class="kw">WAIT FOR</span> <span class="nm">100</span> <span class="kw">ns</span>;
<span class="kw">WAIT UNTIL</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>);
<span class="kw">WAIT ON</span> <span class="id">a</span>, <span class="id">b</span>; <span class="cm">-- espera cambio en a o b</span></pre>
      </div>
    </div>

    <!-- ─── SECUENCIALES ─── -->
    <div class="section" id="sec-secuenciales">
      <div class="section-header">
        <div class="section-tag">08 — Sentencias secuenciales</div>
        <h2>Sentencias secuenciales</h2>
        <p>Se ejecutan dentro de procesos, funciones o procedimientos. Modelan lógica secuencial (flip-flops, contadores).</p>
      </div>

      <h3>IF — ELSIF — ELSE</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — if statement</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">IF</span> <span class="id">rst</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
    <span class="id">count</span> <= (<span class="kw">others</span> => <span class="st">'0'</span>);
<span class="kw">ELSIF</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>) <span class="kw">THEN</span>
    <span class="kw">IF</span> <span class="id">en</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
        <span class="id">count</span> <= <span class="id">count</span> + <span class="nm">1</span>;
    <span class="kw">END IF</span>;
<span class="kw">END IF</span>; <span class="cm">-- ELSE evita latch en combinacional</span></pre>
      </div>

      <h3>CASE — WHEN</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — case statement (decodificador)</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">CASE</span> <span class="id">sel</span> <span class="kw">IS</span>
    <span class="kw">WHEN</span> <span class="st">"00"</span> => <span class="id">Y</span> <= <span class="id">A</span>;
    <span class="kw">WHEN</span> <span class="st">"01"</span> => <span class="id">Y</span> <= <span class="id">B</span>;
    <span class="kw">WHEN</span> <span class="st">"10"</span> => <span class="id">Y</span> <= <span class="id">C</span>;
    <span class="kw">WHEN</span> <span class="kw">OTHERS</span> => <span class="id">Y</span> <= <span class="st">'0'</span>; <span class="cm">-- obligatorio</span>
<span class="kw">END CASE</span>;</pre>
      </div>

      <h3>FOR LOOP</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — for loop</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="cm">-- El índice i no se declara</span>
<span class="kw">FOR</span> <span class="id">i</span> <span class="kw">IN</span> <span class="nm">0</span> <span class="kw">TO</span> <span class="nm">7</span> <span class="kw">LOOP</span>
    <span class="id">result</span>(<span class="id">i</span>) <= <span class="id">data</span>(<span class="id">i</span>) <span class="kw">XOR</span> <span class="id">mask</span>(<span class="id">i</span>);
<span class="kw">END LOOP</span>;

<span class="cm">-- Usando el rango del vector</span>
<span class="kw">FOR</span> <span class="id">i</span> <span class="kw">IN</span> <span class="id">data</span><span class="op">'</span><span class="fn">range</span> <span class="kw">LOOP</span> ... <span class="kw">END LOOP</span>;</pre>
      </div>

      <h3>Funciones y procedimientos</h3>
      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — function & procedure</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="cm">-- Función: retorna un valor</span>
<span class="kw">FUNCTION</span> <span class="fn">max2</span>(<span class="id">a</span>, <span class="id">b</span> : <span class="tp">integer</span>) <span class="kw">RETURN</span> <span class="tp">integer</span> <span class="kw">IS</span>
<span class="kw">BEGIN</span>
    <span class="kw">IF</span> <span class="id">a</span> > <span class="id">b</span> <span class="kw">THEN RETURN</span> <span class="id">a</span>; <span class="kw">ELSE RETURN</span> <span class="id">b</span>; <span class="kw">END IF</span>;
<span class="kw">END FUNCTION</span>;

<span class="cm">-- Procedimiento: no retorna valor, modifica parámetros</span>
<span class="kw">PROCEDURE</span> <span class="fn">reset_bus</span>(<span class="id">bus</span> : <span class="kw">OUT</span> <span class="tp">std_logic_vector</span>) <span class="kw">IS</span>
<span class="kw">BEGIN</span>
    <span class="id">bus</span> := (<span class="kw">others</span> => <span class="st">'0'</span>);
<span class="kw">END PROCEDURE</span>;</pre>
      </div>
    </div>

    <!-- ─── COMPONENTES ─── -->
    <div class="section" id="sec-componentes">
      <div class="section-header">
        <div class="section-tag">09 — Componentes</div>
        <h2>Componentes e instanciación</h2>
        <p>Permiten reutilizar entidades como bloques en diseños jerárquicos (structural modeling).</p>
      </div>

      <div class="steps">
        <div class="step"><div class="step-num">1</div><div class="step-body"><div class="step-title">Declarar el componente</div><div class="step-desc">Antes del <code>BEGIN</code> de la arquitectura, declarar la interfaz del componente.</div></div></div>
        <div class="step"><div class="step-num">2</div><div class="step-body"><div class="step-title">Instanciar y mapear puertos</div><div class="step-desc">Después del <code>BEGIN</code>, instanciar el componente y conectar señales con <code>PORT MAP</code>.</div></div></div>
      </div>

      <div class="code-block">
        <div class="code-header"><span class="lang">vhdl — component instantiation</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
        <pre><span class="kw">ARCHITECTURE</span> <span class="id">structural</span> <span class="kw">OF</span> <span class="id">TopLevel</span> <span class="kw">IS</span>

    <span class="cm">-- 1. Declaración del componente</span>
    <span class="kw">COMPONENT</span> <span class="id">Gate_AND</span>
        <span class="kw">PORT</span>(
            <span class="id">a</span>, <span class="id">b</span> : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
            <span class="id">c</span>    : <span class="kw">OUT</span> <span class="tp">std_logic</span>
        );
    <span class="kw">END COMPONENT</span>;

    <span class="kw">SIGNAL</span> <span class="id">wire1</span>, <span class="id">wire2</span>, <span class="id">wire3</span> : <span class="tp">std_logic</span>;

<span class="kw">BEGIN</span>
    <span class="cm">-- 2. Instanciación (puede haber múltiples copias)</span>
    <span class="id">U1</span> : <span class="id">Gate_AND</span>
        <span class="kw">PORT MAP</span>(
            <span class="id">a</span> => <span class="id">wire1</span>,
            <span class="id">b</span> => <span class="id">wire2</span>,
            <span class="id">c</span> => <span class="id">wire3</span>
        );
<span class="kw">END</span> <span class="id">structural</span>;</pre>
      </div>

      <div class="callout tip"><span class="callout-icon">💡</span><div>También puedes instanciar directamente con <code>ENTITY work.Gate_AND(arch_Gate_AND)</code> sin declarar el componente.</div></div>
    </div>

    <!-- ─── EJEMPLOS ─── -->
    <div class="section" id="sec-ejemplos">
      <div class="section-header">
        <div class="section-tag">10 — Ejemplos completos</div>
        <h2>Ejemplos completos</h2>
        <p>Circuitos digitales completos listos para simular o sintetizar.</p>
      </div>

      <div class="tabs">
        <button class="tab-btn active" onclick="switchTabEx(this,'ex1')">AND Gate</button>
        <button class="tab-btn" onclick="switchTabEx(this,'ex2')">D Flip-Flop</button>
        <button class="tab-btn" onclick="switchTabEx(this,'ex3')">Contador</button>
        <button class="tab-btn" onclick="switchTabEx(this,'ex4')">ALU 4-bit</button>
      </div>
      <div class="code-block" style="border-radius: 0 8px 8px 8px;">

        <div class="tab-pane active" id="tab-ex1">
          <div class="code-header"><span class="lang">vhdl — compuerta AND</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="kw">LIBRARY</span> <span class="id">IEEE</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">STD_LOGIC_1164</span>.<span class="kw">ALL</span>;

<span class="kw">ENTITY</span> <span class="id">Gate_AND</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(<span class="id">a</span>, <span class="id">b</span> : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
         <span class="id">c</span>    : <span class="kw">OUT</span> <span class="tp">std_logic</span>);
<span class="kw">END</span> <span class="id">Gate_AND</span>;

<span class="kw">ARCHITECTURE</span> <span class="id">arch</span> <span class="kw">OF</span> <span class="id">Gate_AND</span> <span class="kw">IS</span>
<span class="kw">BEGIN</span>
    <span class="id">c</span> <= <span class="id">a</span> <span class="kw">AND</span> <span class="id">b</span>;
<span class="kw">END</span> <span class="id">arch</span>;</pre>
        </div>

        <div class="tab-pane" id="tab-ex2">
          <div class="code-header"><span class="lang">vhdl — D flip-flop con reset síncrono</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="kw">LIBRARY</span> <span class="id">IEEE</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">STD_LOGIC_1164</span>.<span class="kw">ALL</span>;

<span class="kw">ENTITY</span> <span class="id">DFF</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(
        <span class="id">clk</span>, <span class="id">rst</span>, <span class="id">D</span> : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">Q</span>, <span class="id">Qn</span>       : <span class="kw">OUT</span> <span class="tp">std_logic</span>
    );
<span class="kw">END</span> <span class="id">DFF</span>;

<span class="kw">ARCHITECTURE</span> <span class="id">behavioral</span> <span class="kw">OF</span> <span class="id">DFF</span> <span class="kw">IS</span>
<span class="kw">BEGIN</span>
    <span class="kw">PROCESS</span>(<span class="id">clk</span>)
    <span class="kw">BEGIN</span>
        <span class="kw">IF</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>) <span class="kw">THEN</span>
            <span class="kw">IF</span> <span class="id">rst</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
                <span class="id">Q</span>  <= <span class="st">'0'</span>;
                <span class="id">Qn</span> <= <span class="st">'1'</span>;
            <span class="kw">ELSE</span>
                <span class="id">Q</span>  <= <span class="id">D</span>;
                <span class="id">Qn</span> <= <span class="kw">NOT</span> <span class="id">D</span>;
            <span class="kw">END IF</span>;
        <span class="kw">END IF</span>;
    <span class="kw">END PROCESS</span>;
<span class="kw">END</span> <span class="id">behavioral</span>;</pre>
        </div>

        <div class="tab-pane" id="tab-ex3">
          <div class="code-header"><span class="lang">vhdl — contador ascendente/descendente</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="kw">LIBRARY</span> <span class="id">IEEE</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">STD_LOGIC_1164</span>.<span class="kw">ALL</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">NUMERIC_STD</span>.<span class="kw">ALL</span>;

<span class="kw">ENTITY</span> <span class="id">Counter</span> <span class="kw">IS</span>
    <span class="kw">GENERIC</span>(<span class="id">N</span> : <span class="tp">integer</span> := <span class="nm">8</span>);
    <span class="kw">PORT</span>(
        <span class="id">clk</span>  : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">rst</span>  : <span class="kw">IN</span>  <span class="tp">std_logic</span>;
        <span class="id">up</span>   : <span class="kw">IN</span>  <span class="tp">std_logic</span>; <span class="cm">-- '1'=up, '0'=down</span>
        <span class="id">cnt</span>  : <span class="kw">OUT</span> <span class="tp">std_logic_vector</span>(<span class="id">N</span>-<span class="nm">1</span> <span class="kw">downto</span> <span class="nm">0</span>)
    );
<span class="kw">END</span> <span class="id">Counter</span>;

<span class="kw">ARCHITECTURE</span> <span class="id">behavioral</span> <span class="kw">OF</span> <span class="id">Counter</span> <span class="kw">IS</span>
    <span class="kw">SIGNAL</span> <span class="id">count</span> : <span class="tp">unsigned</span>(<span class="id">N</span>-<span class="nm">1</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">BEGIN</span>
    <span class="kw">PROCESS</span>(<span class="id">clk</span>, <span class="id">rst</span>)
    <span class="kw">BEGIN</span>
        <span class="kw">IF</span> <span class="id">rst</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
            <span class="id">count</span> <= (<span class="kw">others</span> => <span class="st">'0'</span>);
        <span class="kw">ELSIF</span> <span class="fn">rising_edge</span>(<span class="id">clk</span>) <span class="kw">THEN</span>
            <span class="kw">IF</span> <span class="id">up</span> = <span class="st">'1'</span> <span class="kw">THEN</span>
                <span class="id">count</span> <= <span class="id">count</span> + <span class="nm">1</span>;
            <span class="kw">ELSE</span>
                <span class="id">count</span> <= <span class="id">count</span> - <span class="nm">1</span>;
            <span class="kw">END IF</span>;
        <span class="kw">END IF</span>;
    <span class="kw">END PROCESS</span>;
    <span class="id">cnt</span> <= <span class="fn">std_logic_vector</span>(<span class="id">count</span>);
<span class="kw">END</span> <span class="id">behavioral</span>;</pre>
        </div>

        <div class="tab-pane" id="tab-ex4">
          <div class="code-header"><span class="lang">vhdl — ALU 4-bit</span><button class="copy-btn" onclick="copyCode(this)">Copiar</button></div>
          <pre><span class="kw">LIBRARY</span> <span class="id">IEEE</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">STD_LOGIC_1164</span>.<span class="kw">ALL</span>;
<span class="kw">USE</span> <span class="id">IEEE</span>.<span class="id">NUMERIC_STD</span>.<span class="kw">ALL</span>;

<span class="kw">ENTITY</span> <span class="id">ALU4</span> <span class="kw">IS</span>
    <span class="kw">PORT</span>(
        <span class="id">A</span>, <span class="id">B</span> : <span class="kw">IN</span>  <span class="tp">std_logic_vector</span>(<span class="nm">3</span> <span class="kw">downto</span> <span class="nm">0</span>);
        <span class="id">op</span>   : <span class="kw">IN</span>  <span class="tp">std_logic_vector</span>(<span class="nm">2</span> <span class="kw">downto</span> <span class="nm">0</span>);
        <span class="id">Y</span>    : <span class="kw">OUT</span> <span class="tp">std_logic_vector</span>(<span class="nm">3</span> <span class="kw">downto</span> <span class="nm">0</span>);
        <span class="id">zero</span> : <span class="kw">OUT</span> <span class="tp">std_logic</span>
    );
<span class="kw">END</span> <span class="id">ALU4</span>;

<span class="kw">ARCHITECTURE</span> <span class="id">behavioral</span> <span class="kw">OF</span> <span class="id">ALU4</span> <span class="kw">IS</span>
    <span class="kw">SIGNAL</span> <span class="id">result</span> : <span class="tp">unsigned</span>(<span class="nm">3</span> <span class="kw">downto</span> <span class="nm">0</span>);
<span class="kw">BEGIN</span>
    <span class="kw">PROCESS</span>(<span class="id">A</span>, <span class="id">B</span>, <span class="id">op</span>)
    <span class="kw">BEGIN</span>
        <span class="kw">CASE</span> <span class="id">op</span> <span class="kw">IS</span>
            <span class="kw">WHEN</span> <span class="st">"000"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="id">A</span>) + <span class="fn">unsigned</span>(<span class="id">B</span>);
            <span class="kw">WHEN</span> <span class="st">"001"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="id">A</span>) - <span class="fn">unsigned</span>(<span class="id">B</span>);
            <span class="kw">WHEN</span> <span class="st">"010"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="id">A</span> <span class="kw">AND</span> <span class="id">B</span>);
            <span class="kw">WHEN</span> <span class="st">"011"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="id">A</span> <span class="kw">OR</span>  <span class="id">B</span>);
            <span class="kw">WHEN</span> <span class="st">"100"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="id">A</span> <span class="kw">XOR</span> <span class="id">B</span>);
            <span class="kw">WHEN</span> <span class="st">"101"</span> => <span class="id">result</span> <= <span class="fn">unsigned</span>(<span class="kw">NOT</span> <span class="id">A</span>);
            <span class="kw">WHEN</span> <span class="kw">OTHERS</span> => <span class="id">result</span> <= (<span class="kw">others</span> => <span class="st">'0'</span>);
        <span class="kw">END CASE</span>;
    <span class="kw">END PROCESS</span>;
    <span class="id">Y</span>    <= <span class="fn">std_logic_vector</span>(<span class="id">result</span>);
    <span class="id">zero</span> <= <span class="st">'1'</span> <span class="kw">WHEN</span> <span class="id">result</span> = (<span class="kw">others</span> => <span class="st">'0'</span>) <span class="kw">ELSE</span> <span class="st">'0'</span>;
<span class="kw">END</span> <span class="id">behavioral</span>;</pre>
        </div>
      </div>
    </div>

  </main>
</div>

<script>
  const sections = ['intro','tipos','conversiones','entidad','arquitectura','senales','concurrentes','secuenciales','componentes','ejemplos'];

  function show(id) {
    sections.forEach(s => {
      document.getElementById('sec-'+s).classList.toggle('active', s===id);
    });
    document.querySelectorAll('.nav-item').forEach(btn => {
      btn.classList.toggle('active', btn.getAttribute('onclick').includes("'"+id+"'"));
    });
    document.getElementById('main').scrollTop = 0;
    window.scrollTo(0, 0);
    const idx = sections.indexOf(id);
    const prog = document.getElementById('progress');
    prog.style.transform = `scaleX(${(idx+1)/sections.length})`;
  }

  function copyCode(btn) {
    const pre = btn.closest('.code-block').querySelector('pre');
    const text = pre.innerText;
    navigator.clipboard.writeText(text).then(() => {
      btn.textContent = '¡Copiado!';
      btn.classList.add('copied');
      setTimeout(() => { btn.textContent = 'Copiar'; btn.classList.remove('copied'); }, 1800);
    });
  }

  function switchTab(btn, id) {
    const block = btn.closest('.code-block, .section > *');
    const parent = btn.closest('.section');
    parent.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b===btn));
    parent.querySelectorAll('.tab-pane').forEach(p => p.classList.toggle('active', p.id==='tab-'+id));
  }

  function switchTabEx(btn, id) {
    const parent = btn.closest('.section');
    parent.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b===btn));
    parent.querySelectorAll('.tab-pane').forEach(p => p.classList.toggle('active', p.id==='tab-'+id));
  }
</script>
</body>
</html>
