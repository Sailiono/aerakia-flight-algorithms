"use client";

import { useMemo, useState, type WheelEvent } from "react";

type Condition = {
  id: string;
  dataset: string;
  label: string;
  duration: number;
  before: number;
  after: number;
  reduction: number;
  beforeImage: string;
  afterImage: string;
  note: string;
  detail: string;
};

const conditions: Condition[] = [
  {
    id: "mh01",
    dataset: "EuRoC MAV",
    label: "MH_01 easy · raw IMU seeded",
    duration: 182,
    before: 8.804,
    after: 3.254,
    reduction: 63.04,
    beforeImage: "/attitude/mh01-before.png",
    afterImage: "/attitude/mh01-after.png",
    note: "同数据集、独立真值、同一 synthetic GNSS 设置与初始化族。",
    detail: "偏置校正后，yaw 轨迹更贴近参考，误差面板的持续偏离明显收窄。",
  },
  {
    id: "v103",
    dataset: "EuRoC MAV",
    label: "V1_03 difficult · raw IMU seeded",
    duration: 105,
    before: 4.04,
    after: 2.316,
    reduction: 42.66,
    beforeImage: "/attitude/v103-before.png",
    afterImage: "/attitude/v103-after.png",
    note: "困难航段，保持数据集、真值、辅助观测设置与初始化族一致。",
    detail: "困难轨迹的 roll、pitch 和 yaw 误差都在优化后更集中于零附近。",
  },
  {
    id: "vicon",
    dataset: "EuRoC MAV",
    label: "V1_03 difficult · Vicon body-pose seeded",
    duration: 105,
    before: 4.102,
    after: 2.474,
    reduction: 39.69,
    beforeImage: "/attitude/vicon-before.png",
    afterImage: "/attitude/vicon-after.png",
    note: "Vicon body-pose 链路，保持同一输入族与真值链路。",
    detail: "更换姿态链路后仍保留显著收益，说明改进不依赖单一接入路径。",
  },
];

type ViewMode = "before" | "after" | "overlay";

function formatTime(value: number) {
  return `${value.toFixed(1)} s`;
}

export default function Home() {
  const [conditionId, setConditionId] = useState("mh01");
  const [mode, setMode] = useState<ViewMode>("overlay");
  const [zoom, setZoom] = useState(1.15);
  const [time, setTime] = useState(50);
  const [showGrid, setShowGrid] = useState(true);
  const condition = useMemo(
    () => conditions.find((item) => item.id === conditionId) ?? conditions[0],
    [conditionId],
  );
  const focusSeconds = (time / 100) * condition.duration;
  const zoomPercent = Math.round(zoom * 100);
  const origin = `${time}% 51%`;

  function onWheel(event: WheelEvent<HTMLDivElement>) {
    event.preventDefault();
    setZoom((current) =>
      Math.max(1, Math.min(2.9, Number((current - event.deltaY * 0.0012).toFixed(2)))),
    );
  }

  return (
    <main className="shell">
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark" />
          <span>AERAKIA / EVIDENCE</span>
        </div>
        <div className="topbar-meta">
          <span>Optimization campaign</span>
          <span>Jul 17-21, 2026</span>
        </div>
      </header>

      <section className="intro">
        <div>
          <p className="eyebrow">ATTITUDE OPTIMIZATION EXPLORER</p>
          <h1>用真实轨迹，把优化效果看细。</h1>
        </div>
        <p className="intro-copy">
          三组严格同工况配对验证。切换版本、拖动时间轴并放大图像，查看 roll、pitch、yaw
          及对应误差轨迹的局部拟合效果。
        </p>
      </section>

      <section className="workspace" aria-label="姿态对比交互查看器">
        <aside className="control-panel">
          <div className="panel-section">
            <p className="section-label">工况</p>
            <div className="condition-list">
              {conditions.map((item) => (
                <button
                  className={`condition-button ${item.id === condition.id ? "active" : ""}`}
                  key={item.id}
                  onClick={() => {
                    setConditionId(item.id);
                    setTime(50);
                    setZoom(1.15);
                  }}
                  type="button"
                >
                  <span>{item.dataset}</span>
                  <strong>{item.label}</strong>
                  <em>{item.reduction.toFixed(0)}% RMSE reduction</em>
                </button>
              ))}
            </div>
          </div>

          <div className="panel-section metric-stack">
            <p className="section-label">当前配对结果</p>
            <div className="metric-row">
              <span>优化前</span>
              <strong>{condition.before.toFixed(2)}°</strong>
            </div>
            <div className="metric-row">
              <span>优化后</span>
              <strong className="positive">{condition.after.toFixed(2)}°</strong>
            </div>
            <div className="metric-row emphasis">
              <span>姿态 RMSE 降幅</span>
              <strong>{condition.reduction.toFixed(2)}%</strong>
            </div>
          </div>

          <div className="panel-section">
            <p className="section-label">查看方式</p>
            <div className="segmented" role="group" aria-label="版本显示模式">
              {[
                ["before", "优化前"],
                ["after", "优化后"],
                ["overlay", "叠加"],
              ].map(([value, label]) => (
                <button
                  className={mode === value ? "selected" : ""}
                  key={value}
                  onClick={() => setMode(value as ViewMode)}
                  type="button"
                >
                  {label}
                </button>
              ))}
            </div>
            <label className="switch-row">
              <input
                checked={showGrid}
                onChange={(event) => setShowGrid(event.target.checked)}
                type="checkbox"
              />
              <span>显示定位网格</span>
            </label>
          </div>

          <div className="panel-section source-note">
            <p className="section-label">比较边界</p>
            <p>{condition.note}</p>
          </div>
        </aside>

        <section className="viewer-panel">
          <div className="viewer-header">
            <div>
              <p className="section-label">轨迹查看器</p>
              <h2>{condition.label}</h2>
            </div>
            <div className="legend">
              <span><i className="before-dot" />优化前</span>
              <span><i className="after-dot" />偏置校正后</span>
            </div>
          </div>

          <div
            className={`plot-frame ${showGrid ? "with-grid" : ""}`}
            onWheel={onWheel}
            tabIndex={0}
            aria-label="可缩放姿态对比图"
          >
            <div className="plot-canvas">
              {mode !== "after" && (
                <img
                  alt={`${condition.label} 优化前姿态与误差轨迹`}
                  className={`trajectory-image before-image ${mode === "overlay" ? "overlay-image" : ""}`}
                  src={condition.beforeImage}
                  style={{ transform: `scale(${zoom})`, transformOrigin: origin }}
                />
              )}
              {mode !== "before" && (
                <img
                  alt={`${condition.label} 优化后姿态与误差轨迹`}
                  className={`trajectory-image after-image ${mode === "overlay" ? "overlay-image" : ""}`}
                  src={condition.afterImage}
                  style={{ transform: `scale(${zoom})`, transformOrigin: origin }}
                />
              )}
              <div
                className="focus-line"
                style={{ left: `${time}%` }}
                aria-hidden="true"
              />
            </div>
            <div className="zoom-badge">{zoomPercent}%</div>
            <button className="reset-button" onClick={() => setZoom(1)} type="button">
              复位视图
            </button>
          </div>

          <div className="timeline-area">
            <div className="timeline-labels">
              <span>0 s</span>
              <strong>焦点 {formatTime(focusSeconds)}</strong>
              <span>{condition.duration} s</span>
            </div>
            <input
              aria-label="拖动时间焦点"
              className="timeline-slider"
              max="100"
              min="0"
              onChange={(event) => setTime(Number(event.target.value))}
              type="range"
              value={time}
            />
            <div className="timeline-ticks" aria-hidden="true">
              <i /><i /><i /><i /><i />
            </div>
          </div>

          <div className="viewer-foot">
            <p><strong>当前观察：</strong>{condition.detail}</p>
            <p>鼠标滚轮缩放 · 拖动时间轴定位 · 叠加模式用于快速比较全程趋势</p>
          </div>
        </section>
      </section>

      <section className="evidence-strip">
        <div>
          <p className="eyebrow">COVERAGE</p>
          <h2>不只是一条更低的曲线。</h2>
        </div>
        <div className="coverage-grid">
          <article>
            <span>Blackbird</span>
            <strong>1.57°</strong>
            <p>aggressive UAV motion<br />independent motion-capture truth</p>
          </article>
          <article>
            <span>INSANE</span>
            <strong>1.53°</strong>
            <p>outdoor dual-RTK heading<br />cold-start yaw RMSE</p>
          </article>
          <article>
            <span>UrbanNav</span>
            <strong>131 s</strong>
            <p>Hong Kong GNSS outage<br />independent navigation reference</p>
          </article>
        </div>
      </section>

      <section className="method-note">
        <p>数据说明</p>
        <span>
          姿态 RMSE 为 ESKF 四元数测地线误差，数值越低越好。曲线来自现有测试报告的原始姿态对比图；
          对比仅在同一数据集、真值链路、辅助观测设置及初始化族内成立。
        </span>
      </section>
    </main>
  );
}
