"use client";

import { useEffect, useMemo, useRef, useState, type PointerEvent, type WheelEvent } from "react";

type Row = { t: number; roll: number[]; pitch: number[]; yaw: number[] };
type AxisMetric = { before: number; after: number; reductionPct: number };
type Curve = {
  id: string; label: string; dataset: string; duration: number; samplePeriodMs: number; samples: number;
  axisRmse: Record<"roll" | "pitch" | "yaw", AxisMetric>;
  attitudeGeodesicRmse: AxisMetric;
  rows: Row[];
};
type Data = { generatedAt: string; curves: Curve[] };
const colors = ["#1c2b33", "#d46b2c", "#007d78"];
const names = ["参考真值", "优化前 ESKF", "偏置校正后 ESKF"];

function wrapAngleDeg(value: number) {
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function actualPoints(rows: Row[], axis: "roll" | "pitch" | "yaw", start: number, end: number) {
  const selected = rows.filter((row) => row.t >= start && row.t <= end);
  if (!selected.length) return [];
  let previousRaw = selected[0][axis].slice();
  let previousAligned = previousRaw.map((value, index) => index === 0
    ? value
    : value + Math.round((previousRaw[0] - value) / 360) * 360);
  const unwrapped = selected.map((row, rowIndex) => {
    if (rowIndex > 0) {
      const currentRaw = row[axis];
      previousAligned = currentRaw.map((value, index) => previousAligned[index] + wrapAngleDeg(value - previousRaw[index]));
      previousRaw = currentRaw.slice();
    }
    return [row.t, previousAligned.slice()];
  });
  const stride = Math.max(1, Math.floor(unwrapped.length / 1600));
  return unwrapped.filter((_, i) => i % stride === 0 || i === unwrapped.length - 1);
}
function path(values: number[][], index: number, min: number, max: number, start: number, end: number) {
  return values.map(([t, data], i) => {
    const x = ((t - start) / (end - start)) * 1000;
    const y = 380 - ((data[index] - min) / (max - min)) * 380;
    return `${i ? "L" : "M"}${x.toFixed(2)},${y.toFixed(2)}`;
  }).join(" ");
}

export default function Home() {
  const [data, setData] = useState<Data | null>(null);
  const [id, setId] = useState("mh01");
  const [axis, setAxis] = useState<"roll" | "pitch" | "yaw">("yaw");
  const [zoom, setZoom] = useState(1);
  const [center, setCenter] = useState(.5);
  const [hover, setHover] = useState<Row | null>(null);
  const drag = useRef<{ x: number; center: number } | null>(null);
  useEffect(() => { fetch("/data/curve-data.json").then((r) => r.json()).then(setData); }, []);
  const curve = data?.curves.find((item) => item.id === id) ?? data?.curves[0];
  const range = useMemo(() => {
    if (!curve) return null;
    const span = curve.duration / zoom; const start = Math.max(0, Math.min(curve.duration - span, center * curve.duration - span / 2));
    const end = start + span; const points = actualPoints(curve.rows, axis, start, end);
    const flat = points.flatMap((row) => row[1]); const low = Math.min(...flat); const high = Math.max(...flat); const pad = Math.max(2, (high - low) * .12);
    return { start, end, points, min: low - pad, max: high + pad };
  }, [curve, axis, zoom, center]);
  function onWheel(event: WheelEvent<SVGSVGElement>) { event.preventDefault(); setZoom((v) => Math.max(1, Math.min(24, Number((v + (event.deltaY < 0 ? .7 : -.7)).toFixed(1))))); }
  function onMove(event: PointerEvent<SVGSVGElement>) {
    if (!curve || !range) return;
    const rect = event.currentTarget.getBoundingClientRect();
    if (drag.current) { setCenter(Math.max(0, Math.min(1, drag.current.center - (event.clientX - drag.current.x) / rect.width / zoom))); return; }
    const t = range.start + ((event.clientX - rect.left) / rect.width) * (range.end - range.start);
    setHover(curve.rows[Math.max(0, Math.min(curve.rows.length - 1, Math.round(t / .005)))]);
  }
  if (!curve || !range) return <main className="loading">正在载入真实验证采样数据...</main>;
  const axisMetric = curve.axisRmse[axis];
  const hoveredValues = hover?.[axis];
  const hoveredBeforeError = hoveredValues ? wrapAngleDeg(hoveredValues[1] - hoveredValues[0]) : null;
  const hoveredAfterError = hoveredValues ? wrapAngleDeg(hoveredValues[2] - hoveredValues[0]) : null;
  return <main className="shell">
    <header className="topbar"><b>AERAKIA / EVIDENCE</b><span>真实验证时序 · 5 ms sampling</span></header>
    <section className="intro"><div><p className="eyebrow">ATTITUDE OPTIMIZATION EXPLORER</p><h1>真实曲线，按采样点看优化。</h1></div><p>这不是报告截图。查看器加载验证目录的 `results.csv` 原始时序结果，显示同一段轨迹中真值、优化前 ESKF 和偏置校正后 ESKF 的实际采样点。</p></section>
    <section className="workspace">
      <aside className="panel">
        <p className="label">配对工况</p>{data.curves.map((item) => <button className={`case ${item.id === curve.id ? "active" : ""}`} key={item.id} onClick={() => { setId(item.id); setZoom(1); setCenter(.5); }}>{item.label}<small>{item.samples.toLocaleString()} samples · {item.duration.toFixed(3)} s</small></button>)}
        <p className="label below">坐标轴</p><div className="seg">{(["roll","pitch","yaw"] as const).map((value) => <button className={axis === value ? "selected" : ""} onClick={() => setAxis(value)} key={value}>{value}</button>)}</div>
        <div className="metrics"><span>优化前姿态几何 RMSE <b>{curve.attitudeGeodesicRmse.before}°</b></span><span>优化后姿态几何 RMSE <b className="teal">{curve.attitudeGeodesicRmse.after}°</b></span><span>当前轴最短角差 RMSE <b>{axisMetric.before}° → {axisMetric.after}°</b></span><span className="gain">{axis.toUpperCase()} 相对降幅 <b>{axisMetric.reductionPct}%</b></span></div>
        <p className="fine">比较文件：<br />raw_imu_trusted_attitude_init/results.csv<br />reference_bias_corrected_trusted_attitude_init/results.csv</p>
      </aside>
      <section className="viewer">
        <div className="viewer-head"><div><p className="label">真实曲线查看器</p><h2>{curve.dataset} · {axis}</h2></div><div className="legend">{names.map((name, i) => <span key={name}><i style={{ background: colors[i] }} />{name}</span>)}</div></div>
        <div className="chart">
          <svg viewBox="0 0 1000 380" preserveAspectRatio="none" onWheel={onWheel} onPointerMove={onMove} onPointerDown={(event) => { drag.current = { x: event.clientX, center }; event.currentTarget.setPointerCapture(event.pointerId); }} onPointerUp={() => { drag.current = null; }} onPointerLeave={() => { drag.current = null; setHover(null); }}>
            {[0, .25, .5, .75, 1].map((v) => <line key={`x-${v}`} x1={v * 1000} x2={v * 1000} y1="0" y2="380" className="grid" />)}
            {[0, .25, .5, .75, 1].map((v) => <line key={`y-${v}`} y1={v * 380} y2={v * 380} x1="0" x2="1000" className="grid" />)}
            {colors.map((color, index) => <path key={color} d={path(range.points, index, range.min, range.max, range.start, range.end)} fill="none" stroke={color} strokeWidth={index === 0 ? 1.4 : 2} vectorEffect="non-scaling-stroke" />)}
            {hover && hover.t >= range.start && hover.t <= range.end && <line x1={((hover.t - range.start) / (range.end - range.start)) * 1000} x2={((hover.t - range.start) / (range.end - range.start)) * 1000} y1="0" y2="380" className="crosshair" />}
          </svg>
          <div className="axis-label top">{range.max.toFixed(1)}°</div><div className="axis-label bottom">{range.min.toFixed(1)}°</div>
          <div className="tooltip">{hover && hoveredValues ? <><b>{hover.t.toFixed(3)} s</b><span>真值 {hoveredValues[0].toFixed(2)}°</span><span>前 {hoveredValues[1].toFixed(2)}° / 最短角差 {hoveredBeforeError?.toFixed(2)}°</span><span>后 {hoveredValues[2].toFixed(2)}° / 最短角差 {hoveredAfterError?.toFixed(2)}°</span></> : <>移动鼠标读取真实采样点</>}</div>
        </div>
        <div className="timeline"><div><span>{range.start.toFixed(3)} s</span><b>窗口 {((range.end-range.start)).toFixed(3)} s · {zoom.toFixed(1)}×</b><span>{range.end.toFixed(3)} s</span></div><input type="range" min="0" max="1000" value={center*1000} onChange={(e) => setCenter(Number(e.target.value)/1000)} /><div className="controls"><button onClick={() => setZoom(1)}>全程</button><button onClick={() => setZoom((z) => Math.min(24,z*2))}>放大</button><button onClick={() => setZoom((z) => Math.max(1,z/2))}>缩小</button><span>滚轮缩放 · 鼠标拖动平移 · 滑块定位时间</span></div></div>
      </section>
    </section>
    <section className="footnote"><b>数据口径</b><span>图中每一个点来自真实 `results.csv`。角度误差按环形变量的最短角差计算，因此 179° 与 -179° 的误差是 2°；曲线也会自动展开跨 ±180° 的显示分支。为保证渲染响应速度，全程视图仅绘制窗口内的原始点抽样；放大后自动使用更高密度的真实点，不进行曲线拟合或伪造插值。</span></section>
  </main>;
}
