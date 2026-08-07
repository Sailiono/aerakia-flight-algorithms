"use client";

import { useEffect, useMemo, useRef, useState, type PointerEvent, type WheelEvent } from "react";

type Axis = "roll" | "pitch" | "yaw";
type PlotMode = "geodesic" | "error" | "angle";
type Row = { t: number; roll: number[]; pitch: number[]; yaw: number[]; geo: number[] };
type Metric = { before: number; after: number; reductionPct: number };
type Curve = {
  id: string; label: string; dataset: string; referenceKind: string; evidenceGrade: string;
  duration: number; samplePeriodMs: number; samples: number; axisRmse: Record<Axis, Metric>;
  attitudeGeodesicRmse: Metric; integrity: Record<string, number | null>; rows: Row[];
};
type Dataset = {
  id: string; label: string; evidenceGrade: string; referenceKind: string; role: string;
  sourcePath: string; dataset: string | null; sequence: string | null; samples: number | null;
  durationS: number | null; coverage: { imuSamples: number | null; headingUpdates: number | null; gnssUpdates: number | null; outageS: number | null };
  highlights: { eskfAttitudeRmseDeg: number | null; eskfYawRmseDeg: number | null; tiltRmseDeg: number | null; positionRmseM: number | null; outagePeakErrorM: number | null; healthyRatio: number | null; magnetometerOnYawRmseDeg: number | null; magnetometerOffYawRmseDeg: number | null; navigationNeesMean: number | null };
  limitations: string[];
};
type Data = {
  schemaVersion: number; generatedAt: string; algorithmCommit: string; repositoryDirty: boolean;
  evidenceGrades: Record<string, string>; overview: { datasetCount: number; curveCount: number; totalSamples: number; independentTruthCount: number; physicalHeadingCount: number; syntheticTrialCount: number | null };
  datasets: Dataset[]; curves: Curve[];
};

const colors = ["#1c2b33", "#d46b2c", "#007d78"];
const names = ["独立参考", "原始 ESKF", "参考偏置诊断"];
const gradeColors: Record<string, string> = { A: "grade-a", B: "grade-b", C: "grade-c", D: "grade-d", E: "grade-e", "A-candidate": "grade-a" };

function wrapAngleDeg(value: number) { return ((value + 180) % 360 + 360) % 360 - 180; }

function actualPoints(rows: Row[], axis: Axis, start: number, end: number, mode: PlotMode) {
  const selected = rows.filter((row) => row.t >= start && row.t <= end);
  const stride = Math.max(1, Math.floor(selected.length / 1600));
  return selected.filter((_, i) => i % stride === 0 || i === selected.length - 1).map((row) => [row.t, mode === "geodesic" ? row.geo : mode === "error" ? row[axis].map((value, index) => index === 0 ? 0 : wrapAngleDeg(value - row[axis][0])) : row[axis]] as [number, number[]]);
}

function path(values: [number, number[]][], index: number, min: number, max: number, start: number, end: number, breakOnJump: boolean) {
  let previous: number | null = null;
  return values.map(([t, data], i) => {
    const x = ((t - start) / (end - start)) * 1000;
    const y = 380 - ((data[index] - min) / Math.max(1e-9, max - min)) * 380;
    const jump = breakOnJump && previous !== null && Math.abs(data[index] - previous) > 180;
    previous = data[index];
    return `${i && !jump ? "L" : "M"}${x.toFixed(2)},${y.toFixed(2)}`;
  }).join(" ");
}

function nearestRow(rows: Row[], t: number) {
  let low = 0; let high = rows.length - 1;
  while (low < high) { const mid = Math.floor((low + high) / 2); if (rows[mid].t < t) low = mid + 1; else high = mid; }
  const candidate = rows[low]; const previous = rows[Math.max(0, low - 1)];
  return Math.abs(candidate.t - t) < Math.abs(previous.t - t) ? candidate : previous;
}

function formatNumber(value: number | null, digits = 2) { return value === null || !Number.isFinite(value) ? "未记录" : value.toLocaleString("en-US", { maximumFractionDigits: digits }); }
function formatSamples(value: number | null) { return value === null ? "未记录" : value.toLocaleString("en-US"); }

export default function Home() {
  const [data, setData] = useState<Data | null>(null);
  const [id, setId] = useState("mh01");
  const [axis, setAxis] = useState<Axis>("yaw");
  const [plotMode, setPlotMode] = useState<PlotMode>("geodesic");
  const [view, setView] = useState<"curves" | "coverage">("curves");
  const [zoom, setZoom] = useState(1);
  const [center, setCenter] = useState(.5);
  const [hover, setHover] = useState<Row | null>(null);
  const [selectedDataset, setSelectedDataset] = useState("insane-indoor");
  const drag = useRef<{ x: number; center: number } | null>(null);
  useEffect(() => { fetch("/data/validation-data.json").then((response) => response.json()).then(setData); }, []);
  const curve = data?.curves.find((item) => item.id === id) ?? data?.curves[0];
  const dataset = data?.datasets.find((item) => item.id === selectedDataset) ?? data?.datasets[0];
  const range = useMemo(() => {
    if (!curve) return null;
    const span = curve.duration / zoom;
    const start = Math.max(0, Math.min(curve.duration - span, center * curve.duration - span / 2));
    const end = start + span;
    const points = actualPoints(curve.rows, axis, start, end, plotMode);
    const flat = points.flatMap((row) => row[1]); const low = Math.min(...flat); const high = Math.max(...flat); const pad = Math.max(2, (high - low) * .12);
    return { start, end, points, min: low - pad, max: high + pad };
  }, [curve, axis, plotMode, zoom, center]);
  function onWheel(event: WheelEvent<SVGSVGElement>) { event.preventDefault(); setZoom((value) => Math.max(1, Math.min(24, Number((value + (event.deltaY < 0 ? .7 : -.7)).toFixed(1))))); }
  function onMove(event: PointerEvent<SVGSVGElement>) {
    if (!curve || !range) return;
    const rect = event.currentTarget.getBoundingClientRect();
    if (drag.current) { setCenter(Math.max(0, Math.min(1, drag.current.center - (event.clientX - drag.current.x) / rect.width / zoom))); return; }
    const t = range.start + ((event.clientX - rect.left) / rect.width) * (range.end - range.start);
    setHover(nearestRow(curve.rows, t));
  }
  if (!curve || !range || !data || !dataset) return <main className="loading">正在载入验证数据目录...</main>;
  const axisMetric = curve.axisRmse[axis];
  const hoveredValues = hover?.[axis];
  const hoveredBeforeError = hoveredValues ? wrapAngleDeg(hoveredValues[1] - hoveredValues[0]) : null;
  const hoveredAfterError = hoveredValues ? wrapAngleDeg(hoveredValues[2] - hoveredValues[0]) : null;
  return <main className="shell">
    <header className="topbar"><b>AERAKIA / EVIDENCE</b><span>可复现验证目录 · commit {data.algorithmCommit.slice(0, 8)}{data.repositoryDirty ? " · dirty" : ""}</span></header>
    <section className="intro"><div><p className="eyebrow">VALIDATION WORKSTATION</p><h1>把每个结论，落到数据上。</h1></div><p>同一套回放结果同时服务于统计、曲线诊断和硬件接入前的回归。证据等级、真值来源和限制单独显示，不把合成压力测试冒充飞行精度。</p></section>
    <section className="summary-grid">
      <div className="summary-item"><span>验证数据条目</span><b>{data.overview.datasetCount}</b><small>{data.overview.independentTruthCount} 条独立物理真值</small></div>
      <div className="summary-item"><span>收录样本量</span><b>{formatSamples(data.overview.totalSamples)}</b><small>含 PX4 工程参考和故障压力样本</small></div>
      <div className="summary-item"><span>物理航向证据</span><b>{data.overview.physicalHeadingCount}</b><small>INSANE：本地磁航向/双 RTK 输入</small></div>
      <div className="summary-item"><span>随机验证</span><b>{formatSamples(data.overview.syntheticTrialCount)}</b><small>Monte Carlo trials，非物理真值</small></div>
    </section>
    <nav className="view-tabs" aria-label="验证视图"><button className={view === "curves" ? "active" : ""} onClick={() => setView("curves")}>曲线诊断</button><button className={view === "coverage" ? "active" : ""} onClick={() => setView("coverage")}>覆盖矩阵</button></nav>
    {view === "curves" ? <section className="workspace">
      <aside className="panel">
        <p className="label">独立真值曲线</p>{data.curves.map((item) => <button className={`case ${item.id === curve.id ? "active" : ""}`} key={item.id} onClick={() => { setId(item.id); setZoom(1); setCenter(.5); }}>{item.label}<small>{item.samples.toLocaleString()} samples · {item.duration.toFixed(3)} s · Grade {item.evidenceGrade}</small></button>)}
        <p className="label below">坐标轴</p><div className="seg">{(["roll", "pitch", "yaw"] as Axis[]).map((value) => <button className={axis === value ? "selected" : ""} onClick={() => setAxis(value)} key={value}>{value}</button>)}</div>
        <p className="label below">绘制方式</p><div className="seg plot-seg"><button className={plotMode === "geodesic" ? "selected" : ""} onClick={() => setPlotMode("geodesic")}>几何误差</button><button className={plotMode === "error" ? "selected" : ""} onClick={() => setPlotMode("error")}>最短角差</button><button className={plotMode === "angle" ? "selected" : ""} onClick={() => setPlotMode("angle")}>Euler 角</button></div>
        <div className="metrics"><span>几何 RMSE <b>{curve.attitudeGeodesicRmse.before}° → <i>{curve.attitudeGeodesicRmse.after}°</i></b></span><span>{axis.toUpperCase()} 最短角差 RMSE <b>{axisMetric.before}° → {axisMetric.after}°</b></span><span className="gain">相对变化 <b>{axisMetric.reductionPct}%</b></span><span>导航 NEES <b>{formatNumber(curve.integrity.navigationNeesMean)}</b></span></div>
        <p className="fine">参考偏置轨迹用于隔离传播/更新数学，不代表在线 bias 可观测性。曲线输入：独立真值 EuRoC，原始与校正轨迹使用同一时间戳。</p>
      </aside>
      <section className="viewer"><div className="viewer-head"><div><p className="label">原始采样点查看器</p><h2>{curve.dataset} · {plotMode === "geodesic" ? "姿态几何误差" : `${axis} · ${plotMode === "error" ? "最短角差" : "原始 Euler 角"}`}</h2></div><div className="legend">{names.map((name, i) => <span key={name}><i style={{ background: colors[i] }} />{plotMode !== "angle" && i === 0 ? "0° 基线" : name}</span>)}</div></div>
        <div className="chart"><svg viewBox="0 0 1000 380" preserveAspectRatio="none" onWheel={onWheel} onPointerMove={onMove} onPointerDown={(event) => { drag.current = { x: event.clientX, center }; event.currentTarget.setPointerCapture(event.pointerId); }} onPointerUp={() => { drag.current = null; }} onPointerLeave={() => { drag.current = null; setHover(null); }}>{[0, .25, .5, .75, 1].map((value) => <line key={`x-${value}`} x1={value * 1000} x2={value * 1000} y1="0" y2="380" className="grid" />)}{[0, .25, .5, .75, 1].map((value) => <line key={`y-${value}`} y1={value * 380} y2={value * 380} x1="0" x2="1000" className="grid" />)}{colors.map((color, index) => <path key={color} d={path(range.points, index, range.min, range.max, range.start, range.end, plotMode !== "geodesic")} fill="none" stroke={color} strokeWidth={index === 0 ? 1.4 : 2} vectorEffect="non-scaling-stroke" />)}{hover && hover.t >= range.start && hover.t <= range.end && <line x1={((hover.t - range.start) / (range.end - range.start)) * 1000} x2={((hover.t - range.start) / (range.end - range.start)) * 1000} y1="0" y2="380" className="crosshair" />}</svg><div className="axis-label top">{range.max.toFixed(1)}°</div><div className="axis-label bottom">{range.min.toFixed(1)}°</div><div className="tooltip">{hover && hoveredValues ? <><b>{hover.t.toFixed(3)} s</b><span>参考 {hoveredValues[0].toFixed(2)}°</span><span>原始 {hoveredValues[1].toFixed(2)}° / 差 {hoveredBeforeError?.toFixed(2)}°</span><span>校正 {hoveredValues[2].toFixed(2)}° / 差 {hoveredAfterError?.toFixed(2)}°</span></> : <>移动鼠标读取真实采样点</>}</div></div>
        <div className="timeline"><div><span>{range.start.toFixed(3)} s</span><b>窗口 {((range.end - range.start)).toFixed(3)} s · {zoom.toFixed(1)}×</b><span>{range.end.toFixed(3)} s</span></div><input aria-label="时间窗口中心" type="range" min="0" max="1000" value={center * 1000} onChange={(event) => setCenter(Number(event.target.value) / 1000)} /><div className="controls"><button onClick={() => setZoom(1)}>全程</button><button onClick={() => setZoom((value) => Math.min(24, value * 2))}>放大</button><button onClick={() => setZoom((value) => Math.max(1, value / 2))}>缩小</button><span>滚轮缩放 · 拖动平移 · 滑块定位</span></div></div>
      </section>
    </section> : <section className="coverage"><div className="coverage-head"><div><p className="label">数据与证据目录</p><h2>先看覆盖，再看单条曲线。</h2></div><p>样本量只说明覆盖规模；Grade A/B/C/D/E 说明真值与来源边界，不能合并成一个“准确率总分”。</p></div><div className="coverage-grid">{data.datasets.map((item) => <button className={`dataset-row ${item.id === selectedDataset ? "selected" : ""}`} key={item.id} onClick={() => setSelectedDataset(item.id)}><span className={`grade ${gradeColors[item.evidenceGrade] ?? "grade-e"}`}>{item.evidenceGrade}</span><span className="dataset-main"><b>{item.label}</b><small>{item.role}</small></span><span className="dataset-stat"><b>{formatSamples(item.samples)}</b><small>samples</small></span><span className="dataset-stat"><b>{formatNumber(item.highlights.eskfYawRmseDeg)}{item.highlights.eskfYawRmseDeg === null ? "" : "°"}</b><small>ESKF yaw RMSE</small></span><span className="dataset-stat"><b>{formatNumber(item.highlights.positionRmseM)}{item.highlights.positionRmseM === null ? "" : " m"}</b><small>position RMSE</small></span></button>)}</div><aside className="dataset-detail"><div><p className="label">选中条目</p><h3>{dataset.label}</h3><p>{dataset.role} · {dataset.referenceKind}</p><p className="source">来源：{dataset.sourcePath}<br />SHA-256：{dataset.sourceSha256.slice(0, 16)}…</p></div><div className="detail-metrics"><span>IMU samples <b>{formatSamples(dataset.coverage.imuSamples)}</b></span><span>GNSS/heading updates <b>{formatSamples(dataset.coverage.gnssUpdates ?? dataset.coverage.headingUpdates)}</b></span><span>最长 outage <b>{formatNumber(dataset.coverage.outageS)}{dataset.coverage.outageS === null ? "" : " s"}</b></span><span>healthy ratio <b>{dataset.highlights.healthyRatio === null ? "未记录" : `${(dataset.highlights.healthyRatio * 100).toFixed(1)}%`}</b></span></div><div className="limitations"><b>限制</b>{dataset.limitations.slice(0, 3).map((item) => <span key={item}>{item}</span>)}</div></aside></section>}
    <section className="footnote"><b>数据口径</b><span>曲线采用四元数测地误差和包角后的 Euler 诊断；参考偏置轨迹只隔离传播/更新数学。PX4、机载姿态或合成 GNSS 只在目录中按其证据等级标记，不能替代独立真值。原始数据不进前端，页面只加载经过 hash 和时间戳校验的摘要与曲线。</span></section>
  </main>;
}
