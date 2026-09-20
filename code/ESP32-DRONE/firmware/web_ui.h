/**
 * @file web_ui.h
 * @brief Zero-Install Mobile Touch Flight Controller UI for ESP32-S3 Drone.
 * Stored in flash (PROGMEM) and served over HTTP to smartphones.
 */

#pragma once

#include <pgmspace.h>

static const char WEB_UI_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <title>AeroCommand // Drone Flight Controller</title>
  <style>
    :root {
      --bg-deep: #060911;
      --bg-surface: #0c1220;
      --bg-panel: rgba(12, 18, 32, 0.88);
      --border-line: rgba(255, 255, 255, 0.08);
      --border-glow: rgba(56, 189, 248, 0.35);
      --text-main: #f8fafc;
      --text-muted: #94a3b8;
      --accent-cyan: #38bdf8;
      --accent-emerald: #10b981;
      --accent-amber: #f59e0b;
      --accent-rose: #f43f5e;
      --font-ui: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      --font-mono: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
    }

    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
      -webkit-touch-callout: none;
      -webkit-user-select: none;
      user-select: none;
      touch-action: none;
    }

    body {
      font-family: var(--font-ui);
      background-color: var(--bg-deep);
      color: var(--text-main);
      height: 100vh;
      height: 100dvh;
      width: 100vw;
      overflow: hidden;
      display: flex;
      flex-direction: column;
      padding: env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left);
      background-image: 
        radial-gradient(circle at 50% 0%, rgba(56, 189, 248, 0.06) 0%, transparent 55%),
        linear-gradient(rgba(255, 255, 255, 0.015) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255, 255, 255, 0.015) 1px, transparent 1px);
      background-size: 100% 100%, 28px 28px, 28px 28px;
    }

    /* Top Telemetry Cockpit Bar */
    .top-bar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 14px;
      background: var(--bg-panel);
      backdrop-filter: blur(16px);
      border-bottom: 1px solid var(--border-line);
      height: 48px;
      flex-shrink: 0;
      z-index: 50;
    }

    .brand-group {
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--accent-rose);
      transition: background 0.2s ease, box-shadow 0.2s ease;
    }

    .status-dot.online {
      background: var(--accent-emerald);
      box-shadow: 0 0 8px var(--accent-emerald);
    }

    .status-dot.armed {
      background: var(--accent-amber);
      box-shadow: 0 0 10px var(--accent-amber);
      animation: pulse 1s infinite;
    }

    @keyframes pulse {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.5; transform: scale(0.85); }
    }

    .drone-title {
      font-size: 0.82rem;
      font-weight: 700;
      letter-spacing: 0.5px;
      text-transform: uppercase;
    }

    .hud-pills {
      display: flex;
      align-items: center;
      gap: 10px;
      font-family: var(--font-mono);
      font-size: 0.70rem;
    }

    .hud-item {
      display: flex;
      align-items: center;
      gap: 3px;
      color: var(--text-muted);
    }

    .hud-item strong {
      color: var(--text-main);
      font-weight: 600;
    }

    .hud-item.batt strong {
      color: var(--accent-cyan);
    }

    .hud-item.ping strong {
      color: var(--accent-emerald);
    }

    .top-actions {
      display: flex;
      align-items: center;
      gap: 6px;
    }

    .btn-top {
      background: #1e293b;
      border: 1px solid var(--border-line);
      color: var(--text-main);
      font-family: var(--font-ui);
      font-size: 0.68rem;
      font-weight: 700;
      padding: 5px 10px;
      border-radius: 6px;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 4px;
      letter-spacing: 0.4px;
    }

    .btn-top:active {
      background: #334155;
    }

    .btn-top.btn-tuning {
      border-color: rgba(56, 189, 248, 0.4);
      color: var(--accent-cyan);
    }

    .btn-kill {
      background: rgba(244, 63, 94, 0.2);
      border: 1px solid var(--accent-rose);
      color: var(--accent-rose);
      font-family: var(--font-ui);
      font-size: 0.70rem;
      font-weight: 800;
      padding: 5px 10px;
      border-radius: 6px;
      letter-spacing: 0.8px;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(244, 63, 94, 0.25);
    }

    .btn-kill:active {
      background: var(--accent-rose);
      color: #fff;
    }

    /* Main Dual Gimbal Deck */
    .flight-deck {
      flex: 1;
      display: grid;
      grid-template-columns: 1fr auto 1fr;
      padding: 10px;
      gap: 10px;
      align-items: center;
      justify-items: center;
      position: relative;
    }

    @media (orientation: portrait) {
      .flight-deck {
        grid-template-columns: 1fr 1fr;
        grid-template-rows: 1fr auto;
      }
      .center-dock {
        grid-column: span 2;
        order: 2;
        width: 100%;
      }
    }

    /* Gimbal Column & Housing */
    .gimbal-column {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 6px;
      width: 100%;
      position: relative;
    }

    .gimbal-wrapper {
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .gimbal-outer {
      position: relative;
      width: 195px;
      height: 195px;
      border-radius: 50%;
      background: radial-gradient(circle at center, #0f1826 0%, #070b12 100%);
      border: 1px solid var(--border-line);
      box-shadow: inset 0 0 24px rgba(0,0,0,0.8), 0 6px 20px rgba(0,0,0,0.5);
      display: flex;
      align-items: center;
      justify-content: center;
    }

    @media (max-height: 480px) {
      .gimbal-outer {
        width: 155px;
        height: 155px;
      }
    }

    /* Concentric rings & crosshairs inside well */
    .gimbal-grid {
      position: absolute;
      inset: 0;
      border-radius: 50%;
      pointer-events: none;
    }

    .gimbal-grid::before {
      content: "";
      position: absolute;
      top: 50%;
      left: 10%;
      right: 10%;
      height: 1px;
      background: rgba(255, 255, 255, 0.08);
      transform: translateY(-50%);
    }

    .gimbal-grid::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 10%;
      bottom: 10%;
      width: 1px;
      background: rgba(255, 255, 255, 0.08);
      transform: translateX(-50%);
    }

    .gimbal-ring {
      position: absolute;
      border-radius: 50%;
      border: 1px dashed rgba(255, 255, 255, 0.07);
      pointer-events: none;
    }

    .gimbal-ring.r50 { width: 50%; height: 50%; }
    .gimbal-ring.r75 { width: 75%; height: 75%; }

    /* Thumb Puck Handle */
    .gimbal-puck {
      position: absolute;
      width: 56px;
      height: 56px;
      border-radius: 50%;
      background: radial-gradient(circle at 35% 35%, #24344b 0%, #0d1522 100%);
      border: 1.5px solid var(--accent-cyan);
      box-shadow: 0 0 16px rgba(56, 189, 248, 0.4), inset 0 0 8px rgba(0,0,0,0.8);
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: box-shadow 0.15s ease;
      will-change: transform;
    }

    .gimbal-puck::after {
      content: "";
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: var(--accent-cyan);
      box-shadow: 0 0 8px var(--accent-cyan);
    }

    .gimbal-puck.active {
      border-color: #fff;
      box-shadow: 0 0 22px var(--accent-cyan), inset 0 0 12px rgba(56, 189, 248, 0.5);
    }

    /* Gimbal Digital Readout Pill */
    .gimbal-readout {
      display: flex;
      gap: 10px;
      padding: 4px 10px;
      border-radius: 20px;
      background: rgba(15, 23, 38, 0.7);
      border: 1px solid var(--border-line);
      font-family: var(--font-mono);
      font-size: 0.68rem;
      color: var(--text-muted);
    }

    .gimbal-readout span strong {
      color: var(--accent-cyan);
    }

    /* Inline Quick Trim Controls Around Gimbals */
    .trim-btn {
      position: absolute;
      background: rgba(15, 23, 38, 0.85);
      border: 1px solid var(--border-line);
      color: var(--text-muted);
      font-family: var(--font-mono);
      font-size: 0.62rem;
      font-weight: 700;
      padding: 3px 6px;
      border-radius: 4px;
      cursor: pointer;
      z-index: 10;
      transition: all 0.1s ease;
    }

    .trim-btn:active {
      background: var(--accent-cyan);
      color: #000;
      border-color: var(--accent-cyan);
    }

    .trim-up    { top: -8px; left: 50%; transform: translateX(-50%); }
    .trim-down  { bottom: -8px; left: 50%; transform: translateX(-50%); }
    .trim-left  { left: -14px; top: 50%; transform: translateY(-50%); }
    .trim-right { right: -14px; top: 50%; transform: translateY(-50%); }

    /* Center Control Dock */
    .center-dock {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 10px;
      padding: 10px 12px;
      background: var(--bg-panel);
      border: 1px solid var(--border-line);
      border-radius: 14px;
      min-width: 175px;
    }

    /* Mini Orientation Horizon (PFD) */
    .pfd-container {
      width: 82px;
      height: 82px;
      border-radius: 50%;
      border: 2px solid rgba(56, 189, 248, 0.35);
      background: #000;
      position: relative;
      overflow: hidden;
      box-shadow: 0 0 12px rgba(0,0,0,0.8), inset 0 0 8px rgba(0,0,0,0.6);
      cursor: pointer;
    }

    .pfd-sky-ground {
      position: absolute;
      width: 160px;
      height: 160px;
      top: -39px;
      left: -39px;
      background: linear-gradient(180deg, #1e3a8a 0%, #0284c7 49.5%, #f8fafc 50%, #b45309 50.5%, #78350f 100%);
      transform-origin: center center;
      transition: transform 0.05s linear;
      will-change: transform;
    }

    .pfd-crosshair {
      position: absolute;
      inset: 0;
      pointer-events: none;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .pfd-crosshair::before {
      content: "";
      position: absolute;
      width: 32px;
      height: 2px;
      background: var(--accent-amber);
      box-shadow: 0 0 4px #000;
    }

    .pfd-crosshair::after {
      content: "";
      position: absolute;
      width: 4px;
      height: 4px;
      border-radius: 50%;
      background: #fff;
    }

    .pfd-angles {
      font-family: var(--font-mono);
      font-size: 0.65rem;
      color: var(--text-muted);
      text-align: center;
      display: flex;
      gap: 8px;
    }

    .pfd-angles strong {
      color: var(--accent-cyan);
    }

    /* Slide to Arm Track */
    .arm-slider-track {
      position: relative;
      width: 160px;
      height: 40px;
      background: #050810;
      border: 1px solid var(--border-line);
      border-radius: 20px;
      overflow: hidden;
      display: flex;
      align-items: center;
      cursor: pointer;
    }

    .arm-slider-track.armed {
      background: rgba(245, 158, 11, 0.15);
      border-color: var(--accent-amber);
    }

    .arm-slider-label {
      position: absolute;
      width: 100%;
      text-align: center;
      font-size: 0.65rem;
      font-weight: 700;
      letter-spacing: 0.8px;
      color: var(--text-muted);
      text-transform: uppercase;
      pointer-events: none;
      padding-left: 18px;
    }

    .arm-slider-track.armed .arm-slider-label {
      padding-left: 0;
      color: var(--accent-amber);
    }

    .arm-slider-handle {
      position: absolute;
      left: 2px;
      width: 36px;
      height: 36px;
      border-radius: 50%;
      background: linear-gradient(135deg, #1e293b, #0f172a);
      border: 1px solid var(--accent-cyan);
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 0 10px rgba(56, 189, 248, 0.35);
      transition: transform 0.1s linear;
      will-change: transform;
    }

    .arm-slider-handle svg {
      width: 13px;
      height: 13px;
      fill: var(--accent-cyan);
    }

    .center-actions {
      display: flex;
      gap: 6px;
      width: 100%;
    }

    .btn-dock {
      flex: 1;
      background: #1e293b;
      border: 1px solid var(--border-line);
      color: var(--text-main);
      font-family: var(--font-ui);
      font-size: 0.65rem;
      font-weight: 600;
      padding: 6px 8px;
      border-radius: 6px;
      cursor: pointer;
      text-align: center;
      transition: background 0.15s ease;
    }

    .btn-dock:active {
      background: #334155;
    }

    /* Swarm Target Selector */
    .swarm-target-box {
      width: 100%;
      display: flex;
      flex-direction: column;
      gap: 3px;
      margin: 2px 0 3px;
    }

    .target-badge-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 0 2px;
    }

    .target-title {
      font-size: 0.52rem;
      font-weight: 700;
      letter-spacing: 0.06em;
      color: var(--text-dim);
    }

    .target-badge {
      font-size: 0.52rem;
      font-weight: 700;
      padding: 1px 6px;
      border-radius: 4px;
      letter-spacing: 0.04em;
    }

    .target-badge.swarm {
      background: rgba(0, 242, 254, 0.15);
      color: var(--accent-cyan);
      border: 1px solid rgba(0, 242, 254, 0.3);
    }

    .target-badge.leader {
      background: rgba(59, 130, 246, 0.18);
      color: #60a5fa;
      border: 1px solid rgba(59, 130, 246, 0.4);
    }

    .target-badge.follower {
      background: rgba(168, 85, 247, 0.2);
      color: #c084fc;
      border: 1px solid rgba(168, 85, 247, 0.4);
    }

    .target-pills {
      display: flex;
      width: 100%;
      background: #090e18;
      border: 1px solid var(--border-line);
      border-radius: 6px;
      padding: 2px;
      gap: 2px;
      overflow-x: auto;
    }

    .target-pill {
      flex: 1;
      background: transparent;
      border: 1px solid transparent;
      color: var(--text-dim);
      font-family: var(--font-ui);
      font-size: 0.58rem;
      font-weight: 600;
      padding: 4px 2px;
      border-radius: 4px;
      cursor: pointer;
      transition: all 0.15s ease;
      white-space: nowrap;
      text-align: center;
    }

    .target-pill.active {
      background: rgba(0, 242, 254, 0.2);
      color: var(--accent-cyan);
      border-color: rgba(0, 242, 254, 0.5);
      box-shadow: 0 0 6px rgba(0, 242, 254, 0.25);
      font-weight: 700;
    }

    .target-pill[data-target="1"].active {
      background: rgba(59, 130, 246, 0.22);
      color: #93c5fd;
      border-color: rgba(59, 130, 246, 0.55);
      box-shadow: 0 0 6px rgba(59, 130, 246, 0.3);
    }

    .target-pill[data-target="2"].active,
    .target-pill:not([data-target="255"]):not([data-target="1"]).active {
      background: rgba(168, 85, 247, 0.25);
      color: #d8b4fe;
      border-color: rgba(168, 85, 247, 0.6);
      box-shadow: 0 0 6px rgba(168, 85, 247, 0.35);
    }

    /* Tuning / Adjustments Modal Drawer */
    .modal-drawer {
      position: fixed;
      inset: 0;
      background: rgba(4, 7, 13, 0.75);
      backdrop-filter: blur(12px);
      display: none;
      align-items: flex-end;
      justify-content: center;
      z-index: 120;
      touch-action: pan-y !important;
    }

    .modal-drawer.show {
      display: flex;
    }

    .modal-content {
      background: #0b111e;
      border: 1px solid var(--border-line);
      border-radius: 18px 18px 0 0;
      width: 100%;
      max-width: 520px;
      max-height: 82vh;
      overflow-y: scroll;
      -webkit-overflow-scrolling: touch !important;
      overscroll-behavior-y: contain;
      padding: 16px 20px calc(36px + env(safe-area-inset-bottom));
      box-shadow: 0 -8px 30px rgba(0, 0, 0, 0.8);
      display: flex;
      flex-direction: column;
      gap: 14px;
      animation: slideUp 0.25s ease-out;
      touch-action: pan-y !important;
    }

    /* Enable vertical touch drag and scrolling through all modal children */
    .modal-content,
    .modal-content div,
    .modal-content span,
    .modal-content p,
    .modal-content header,
    .modal-content label {
      touch-action: pan-y !important;
    }

    /* Custom Sleek Scrollbar */
    .modal-content::-webkit-scrollbar {
      width: 6px;
    }
    .modal-content::-webkit-scrollbar-track {
      background: rgba(15, 23, 42, 0.6);
      border-radius: 3px;
    }
    .modal-content::-webkit-scrollbar-thumb {
      background: rgba(56, 189, 248, 0.5);
      border-radius: 3px;
    }

    @keyframes slideUp {
      from { transform: translateY(100%); }
      to { transform: translateY(0); }
    }

    .modal-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 10px;
      border-bottom: 1px solid var(--border-line);
    }

    .modal-title {
      font-size: 0.95rem;
      font-weight: 700;
      color: var(--text-main);
      display: flex;
      align-items: center;
      gap: 6px;
    }

    .modal-close {
      background: none;
      border: none;
      color: var(--text-muted);
      font-size: 1.3rem;
      cursor: pointer;
      padding: 4px 8px;
    }

    .tuning-card {
      background: rgba(18, 26, 43, 0.7);
      border: 1px solid var(--border-line);
      border-radius: 10px;
      padding: 12px;
      display: flex;
      flex-direction: column;
      gap: 10px;
      flex-shrink: 0;
      touch-action: pan-y !important;
    }

    .card-title {
      font-size: 0.72rem;
      font-weight: 700;
      color: var(--accent-cyan);
      text-transform: uppercase;
      letter-spacing: 0.6px;
    }

    .tune-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
    }

    .tune-label {
      font-size: 0.76rem;
      color: var(--text-main);
      font-weight: 500;
    }

    .tune-sub {
      font-size: 0.65rem;
      color: var(--text-muted);
      display: block;
    }

    .tune-val {
      font-family: var(--font-mono);
      font-size: 0.74rem;
      color: var(--accent-cyan);
      font-weight: 700;
      min-width: 45px;
      text-align: right;
    }

    /* Custom Switch Toggle */
    .switch {
      position: relative;
      display: inline-block;
      width: 44px;
      height: 24px;
    }

    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }

    .slider-toggle {
      position: absolute;
      cursor: pointer;
      inset: 0;
      background-color: #1e293b;
      transition: .2s;
      border-radius: 24px;
      border: 1px solid var(--border-line);
    }

    .slider-toggle:before {
      position: absolute;
      content: "";
      height: 16px;
      width: 16px;
      left: 3px;
      bottom: 3px;
      background-color: var(--text-muted);
      transition: .2s;
      border-radius: 50%;
    }

    input:checked + .slider-toggle {
      background-color: rgba(56, 189, 248, 0.3);
      border-color: var(--accent-cyan);
    }

    input:checked + .slider-toggle:before {
      transform: translateX(20px);
      background-color: var(--accent-cyan);
    }

    /* Trim Adjuster Stepper */
    .stepper {
      display: flex;
      align-items: center;
      gap: 8px;
      touch-action: pan-y !important;
    }

    .step-btn {
      width: 32px;
      height: 32px;
      border-radius: 6px;
      background: #1e293b;
      border: 1px solid var(--border-line);
      color: var(--text-main);
      font-weight: 700;
      font-size: 1rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      touch-action: manipulation !important;
    }

    .step-btn:active {
      background: var(--accent-cyan);
      color: #000;
    }

    .range-slider {
      flex: 1;
      accent-color: var(--accent-cyan);
      height: 28px;
      cursor: pointer;
      touch-action: pan-x !important;
    }

    /* Modal / Failsafe Overlay */
    .overlay-alert {
      position: fixed;
      inset: 0;
      background: rgba(6, 9, 17, 0.92);
      backdrop-filter: blur(10px);
      display: none;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      z-index: 100;
      gap: 12px;
      text-align: center;
      padding: 24px;
    }

    .overlay-alert.show {
      display: flex;
    }

    .alert-icon {
      width: 48px;
      height: 48px;
      border-radius: 50%;
      background: rgba(244, 63, 94, 0.2);
      border: 1px solid var(--accent-rose);
      display: flex;
      align-items: center;
      justify-content: center;
      box-shadow: 0 0 20px rgba(244, 63, 94, 0.4);
    }

    .alert-icon svg {
      width: 24px;
      height: 24px;
      fill: var(--accent-rose);
    }
  </style>
</head>
<body>

  <!-- Top Cockpit Bar -->
  <header class="top-bar">
    <div class="brand-group">
      <div id="conn-dot" class="status-dot"></div>
      <div>
        <div class="drone-title">AeroCommand</div>
      </div>
    </div>

    <!-- Live Telemetry Badges -->
    <div class="hud-pills">
      <div class="hud-item batt">
        BATT: <strong id="val-batt">--.-V</strong>
      </div>
      <div class="hud-item">
        ST: <strong id="val-state" style="color: var(--accent-emerald);">IDLE</strong>
      </div>
      <div class="hud-item" id="hud-swarm-box" style="display:none; color: var(--accent-cyan);">
        MESH: <strong id="val-swarm-peers">0</strong>
      </div>
      <div class="hud-item ping">
        <strong id="val-ping">--ms</strong>
      </div>
    </div>

    <!-- Top Action Buttons -->
    <div class="top-actions">
      <button id="btn-open-tuning" class="btn-top btn-tuning" title="Orientation & Trim Tuning">
        ⚙️ ADJ
      </button>
      <button id="btn-kill" class="btn-kill" title="Instant Motor Cut">
        KILL
      </button>
    </div>
  </header>

  <!-- Main Flight Control Deck -->
  <main class="flight-deck">

    <!-- Left Gimbal: Throttle (Y) & Yaw (X) + Yaw Trims -->
    <div class="gimbal-column">
      <div class="gimbal-wrapper">
        <button class="trim-btn trim-left" id="btn-trim-yaw-dec">◄</button>
        <button class="trim-btn trim-right" id="btn-trim-yaw-inc">►</button>

        <div class="gimbal-outer" id="gimbal-left-zone">
          <div class="gimbal-grid"></div>
          <div class="gimbal-ring r50"></div>
          <div class="gimbal-ring r75"></div>
          <div class="gimbal-puck" id="puck-left"></div>
        </div>
      </div>

      <div class="gimbal-readout">
        <span>THR: <strong id="readout-thr">0%</strong></span>
        <span>YAW: <strong id="readout-yaw">0°/s</strong></span>
        <span>TRIM: <strong id="readout-yaw-trim">0</strong></span>
      </div>
    </div>

    <!-- Center Dock: PFD Horizon, Slide to Arm & Subsystem Controls -->
    <div class="center-dock">
      <!-- 2D Attitude Horizon (PFD) -->
      <div class="pfd-container" id="pfd-horizon" title="Tap to open Orientation & Trim Tuning">
        <div class="pfd-sky-ground" id="pfd-sky"></div>
        <div class="pfd-crosshair"></div>
      </div>

      <div class="pfd-angles">
        <span>P: <strong id="att-pitch">+0.0°</strong></span>
        <span>R: <strong id="att-roll">+0.0°</strong></span>
      </div>

      <!-- Swarm Target Selector -->
      <div class="swarm-target-box">
        <div class="target-badge-row">
          <span class="target-title">CONTROL TARGET:</span>
          <span class="target-badge swarm" id="target-status-badge">ALL (SWARM)</span>
        </div>
        <div class="target-pills" id="target-pill-group">
          <button class="target-pill active" data-target="255">🌐 ALL</button>
          <button class="target-pill" data-target="1">⭐ LEAD (#1)</button>
          <button class="target-pill" data-target="2">🤖 FOLL (#2)</button>
        </div>
      </div>

      <!-- Slide to Arm -->
      <div class="arm-slider-track" id="arm-track">
        <span class="arm-slider-label" id="arm-label">Slide to Arm &gt;&gt;</span>
        <div class="arm-slider-handle" id="arm-handle">
          <svg viewBox="0 0 24 24"><path d="M12 2L4 5v6.09c0 5.05 3.41 9.76 8 10.91 4.59-1.15 8-5.86 8-10.91V5l-8-3zm0 18c-3.75-1-6-5.04-6-9V6.3l6-2.25 6 2.25V11c0 3.96-2.25 8-6 9z"/></svg>
        </div>
      </div>

      <div class="center-actions">
        <button id="btn-calib" class="btn-dock">Calibrate</button>
        <button id="btn-thr-mode" class="btn-dock">Hold Thr</button>
      </div>
    </div>

    <!-- Right Gimbal: Pitch (Y) & Roll (X) + Digital Trims -->
    <div class="gimbal-column">
      <div class="gimbal-wrapper">
        <button class="trim-btn trim-up" id="btn-trim-pitch-inc">▲</button>
        <button class="trim-btn trim-down" id="btn-trim-pitch-dec">▼</button>
        <button class="trim-btn trim-left" id="btn-trim-roll-dec">◄</button>
        <button class="trim-btn trim-right" id="btn-trim-roll-inc">►</button>

        <div class="gimbal-outer" id="gimbal-right-zone">
          <div class="gimbal-grid"></div>
          <div class="gimbal-ring r50"></div>
          <div class="gimbal-ring r75"></div>
          <div class="gimbal-puck" id="puck-right"></div>
        </div>
      </div>

      <div class="gimbal-readout">
        <span>P: <strong id="readout-pitch">0.0°</strong></span>
        <span>R: <strong id="readout-roll">0.0°</strong></span>
        <span>TRIM: <strong id="readout-pr-trim">P0 R0</strong></span>
      </div>
    </div>

  </main>

  <!-- Tuning & Orientation Drawer Modal -->
  <div class="modal-drawer" id="tuning-modal">
    <div class="modal-content">
      <div class="modal-header">
        <div class="modal-title">
          <span>⚙️ Flight Adjustments & Orientation</span>
        </div>
        <button class="modal-close" id="modal-close-btn">&times;</button>
      </div>

      <!-- 1. Attitude & Orientation Settings -->
      <div class="tuning-card">
        <div class="card-title">Orientation & Inversions</div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Invert Pitch (Elevator)</div>
            <span class="tune-sub">Pushing stick forward pitches nose down</span>
          </div>
          <label class="switch">
            <input type="checkbox" id="chk-invert-pitch">
            <span class="slider-toggle"></span>
          </label>
        </div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Invert Roll (Aileron)</div>
            <span class="tune-sub">Reverses stick left/right direction</span>
          </div>
          <label class="switch">
            <input type="checkbox" id="chk-invert-roll">
            <span class="slider-toggle"></span>
          </label>
        </div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Invert Yaw (Rudder)</div>
            <span class="tune-sub">Reverses spin rotation direction</span>
          </div>
          <label class="switch">
            <input type="checkbox" id="chk-invert-yaw">
            <span class="slider-toggle"></span>
          </label>
        </div>
      </div>

      <!-- 2. Sub-Trim Matrix -->
      <div class="tuning-card">
        <div class="card-title">Sub-Trim Adjustments (Fine Drift Tune)</div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Pitch Trim (Nose Up/Down)</div>
          </div>
          <div class="stepper">
            <button class="step-btn" id="modal-p-dec">-</button>
            <input type="range" class="range-slider" id="slider-trim-p" min="-100" max="100" value="0">
            <button class="step-btn" id="modal-p-inc">+</button>
            <span class="tune-val" id="val-trim-p">0</span>
          </div>
        </div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Roll Trim (Wing Left/Right)</div>
          </div>
          <div class="stepper">
            <button class="step-btn" id="modal-r-dec">-</button>
            <input type="range" class="range-slider" id="slider-trim-r" min="-100" max="100" value="0">
            <button class="step-btn" id="modal-r-inc">+</button>
            <span class="tune-val" id="val-trim-r">0</span>
          </div>
        </div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Yaw Trim (Rudder Drift)</div>
          </div>
          <div class="stepper">
            <button class="step-btn" id="modal-y-dec">-</button>
            <input type="range" class="range-slider" id="slider-trim-y" min="-100" max="100" value="0">
            <button class="step-btn" id="modal-y-inc">+</button>
            <span class="tune-val" id="val-trim-y">0</span>
          </div>
        </div>

        <button class="btn-dock" id="btn-reset-trims" style="margin-top: 4px;">
          Reset All Trims to 0
        </button>
      </div>

      <!-- 3. Rates & Throttle Safety Ceiling -->
      <div class="tuning-card">
        <div class="card-title">Throttle & Rates Limits</div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Max Throttle Ceiling</div>
            <span class="tune-sub">Limits max power (prevents battery brownout)</span>
          </div>
          <div class="stepper" style="width: 140px;">
            <input type="range" class="range-slider" id="slider-max-thr" min="40" max="100" value="75">
            <span class="tune-val" id="val-max-thr">75%</span>
          </div>
        </div>

        <div class="tune-row">
          <div>
            <div class="tune-label">Max Tilt Angle (Pitch/Roll)</div>
            <span class="tune-sub">Max stick tilt demand in Angle Mode</span>
          </div>
          <div class="stepper" style="width: 140px;">
            <input type="range" class="range-slider" id="slider-max-angle" min="15" max="45" value="30">
            <span class="tune-val" id="val-max-angle">30°</span>
          </div>
        </div>
      </div>

    </div>
  </div>

  <!-- Failsafe Disconnect Overlay -->
  <div class="overlay-alert" id="disconnect-overlay">
    <div class="alert-icon">
      <svg viewBox="0 0 24 24"><path d="M1 21h22L12 2 1 21zm12-3h-2v-2h2v2zm0-4h-2v-4h2v4z"/></svg>
    </div>
    <div style="font-size: 1.05rem; font-weight: 700; color: #fff;">LINK DISCONNECTED</div>
    <div style="font-size: 0.78rem; color: var(--text-muted); max-width: 280px;">
      Signal to drone lost. Flight controller safety watchdog automatically disarmed all motors.
    </div>
    <button id="btn-reconnect" class="btn-dock" style="margin-top: 8px; padding: 10px 20px; font-size: 0.78rem; background: var(--accent-cyan); color: #000; font-weight: 700;">
      RETRY LINK
    </button>
  </div>

  <script>
    // =========================================================================
    // Flight Control & Settings State
    // =========================================================================
    const ctrl = {
      rawThrottle: 0.0, // 0 to 1000
      rawYaw: 0.0,      // -500 to +500
      rawPitch: 0.0,    // -500 to +500
      rawRoll: 0.0,     // -500 to +500
      arm: 0,           // 0 = Disarm, 1 = Arm, 2 = Kill
      holdThrottle: true,
      armed: false,
      targetNode: 255,   // 255 = SWARM (ALL), 1 = Leader, 2 = Follower 2, etc.
      ws: null,
      lastPingSent: 0,
      pingMs: 0,
      failsafeTimer: null
    };

    // User Configurable Inversions, Trims, and Limits (Persisted in localStorage)
    const settings = {
      invertPitch: false,
      invertRoll: false,
      invertYaw: false,
      pitchTrim: 0,     // -100 to +100
      rollTrim: 0,      // -100 to +100
      yawTrim: 0,       // -100 to +100
      maxThrottle: 75,  // 40% to 100%
      maxAngle: 30      // 15 deg to 45 deg
    };

    function loadSettings() {
      try {
        const saved = localStorage.getItem('resqmesh_mobile_cfg');
        if (saved) {
          Object.assign(settings, JSON.parse(saved));
        }
      } catch (e) {
        console.warn("Could not load settings:", e);
      }
    }

    function saveSettings() {
      try {
        localStorage.setItem('resqmesh_mobile_cfg', JSON.stringify(settings));
      } catch (e) {
        console.warn("Could not save settings:", e);
      }
      updateSettingsUI();
    }

    // Cache DOM Elements
    const el = {
      connDot: document.getElementById('conn-dot'),
      valBatt: document.getElementById('val-batt'),
      valState: document.getElementById('val-state'),
      valPing: document.getElementById('val-ping'),
      btnKill: document.getElementById('btn-kill'),
      btnCalib: document.getElementById('btn-calib'),
      btnThrMode: document.getElementById('btn-thr-mode'),
      btnOpenTuning: document.getElementById('btn-open-tuning'),
      modalCloseBtn: document.getElementById('modal-close-btn'),
      tuningModal: document.getElementById('tuning-modal'),
      armTrack: document.getElementById('arm-track'),
      armHandle: document.getElementById('arm-handle'),
      armLabel: document.getElementById('arm-label'),
      readoutThr: document.getElementById('readout-thr'),
      readoutYaw: document.getElementById('readout-yaw'),
      readoutYawTrim: document.getElementById('readout-yaw-trim'),
      readoutPitch: document.getElementById('readout-pitch'),
      readoutRoll: document.getElementById('readout-roll'),
      readoutPrTrim: document.getElementById('readout-pr-trim'),
      pfdHorizon: document.getElementById('pfd-horizon'),
      pfdSky: document.getElementById('pfd-sky'),
      attPitch: document.getElementById('att-pitch'),
      attRoll: document.getElementById('att-roll'),
      boxSwarm: document.getElementById('hud-swarm-box'),
      valSwarm: document.getElementById('val-swarm-peers'),
      targetPillGroup: document.getElementById('target-pill-group'),
      targetBadge: document.getElementById('target-status-badge'),
      overlay: document.getElementById('disconnect-overlay'),
      btnReconnect: document.getElementById('btn-reconnect'),
      gimbalLeft: document.getElementById('gimbal-left-zone'),
      puckLeft: document.getElementById('puck-left'),
      gimbalRight: document.getElementById('gimbal-right-zone'),
      puckRight: document.getElementById('puck-right'),
      // Inlines Trims
      btnTrimPitchInc: document.getElementById('btn-trim-pitch-inc'),
      btnTrimPitchDec: document.getElementById('btn-trim-pitch-dec'),
      btnTrimRollInc: document.getElementById('btn-trim-roll-inc'),
      btnTrimRollDec: document.getElementById('btn-trim-roll-dec'),
      btnTrimYawInc: document.getElementById('btn-trim-yaw-inc'),
      btnTrimYawDec: document.getElementById('btn-trim-yaw-dec'),
      // Modal controls
      chkInvertPitch: document.getElementById('chk-invert-pitch'),
      chkInvertRoll: document.getElementById('chk-invert-roll'),
      chkInvertYaw: document.getElementById('chk-invert-yaw'),
      sliderTrimP: document.getElementById('slider-trim-p'),
      valTrimP: document.getElementById('val-trim-p'),
      btnModalPDec: document.getElementById('modal-p-dec'),
      btnModalPInc: document.getElementById('modal-p-inc'),
      sliderTrimR: document.getElementById('slider-trim-r'),
      valTrimR: document.getElementById('val-trim-r'),
      btnModalRDec: document.getElementById('modal-r-dec'),
      btnModalRInc: document.getElementById('modal-r-inc'),
      sliderTrimY: document.getElementById('slider-trim-y'),
      valTrimY: document.getElementById('val-trim-y'),
      btnModalYDec: document.getElementById('modal-y-dec'),
      btnModalYInc: document.getElementById('modal-y-inc'),
      btnResetTrims: document.getElementById('btn-reset-trims'),
      sliderMaxThr: document.getElementById('slider-max-thr'),
      valMaxThr: document.getElementById('val-max-thr'),
      sliderMaxAngle: document.getElementById('slider-max-angle'),
      valMaxAngle: document.getElementById('val-max-angle')
    };

    function updateSettingsUI() {
      el.chkInvertPitch.checked = settings.invertPitch;
      el.chkInvertRoll.checked = settings.invertRoll;
      el.chkInvertYaw.checked = settings.invertYaw;

      el.sliderTrimP.value = settings.pitchTrim;
      el.valTrimP.textContent = (settings.pitchTrim > 0 ? '+' : '') + settings.pitchTrim;

      el.sliderTrimR.value = settings.rollTrim;
      el.valTrimR.textContent = (settings.rollTrim > 0 ? '+' : '') + settings.rollTrim;

      el.sliderTrimY.value = settings.yawTrim;
      el.valTrimY.textContent = (settings.yawTrim > 0 ? '+' : '') + settings.yawTrim;

      el.sliderMaxThr.value = settings.maxThrottle;
      el.valMaxThr.textContent = `${settings.maxThrottle}%`;

      el.sliderMaxAngle.value = settings.maxAngle;
      el.valMaxAngle.textContent = `${settings.maxAngle}°`;

      el.readoutYawTrim.textContent = (settings.yawTrim > 0 ? '+' : '') + settings.yawTrim;
      el.readoutPrTrim.textContent = `P${settings.pitchTrim > 0 ? '+' : ''}${settings.pitchTrim} R${settings.rollTrim > 0 ? '+' : ''}${settings.rollTrim}`;
    }

    // Modal Events & Touch Isolation
    el.btnOpenTuning.addEventListener('click', () => {
      el.tuningModal.classList.add('show');
    });
    el.pfdHorizon.addEventListener('click', () => {
      el.tuningModal.classList.add('show');
    });
    el.modalCloseBtn.addEventListener('click', () => {
      el.tuningModal.classList.remove('show');
    });
    el.tuningModal.addEventListener('click', (e) => {
      if (e.target === el.tuningModal) {
        el.tuningModal.classList.remove('show');
      }
    });

    const modalContentEl = document.querySelector('.modal-content');
    if (modalContentEl) {
      modalContentEl.addEventListener('touchstart', (e) => {
        e.stopPropagation();
      }, { passive: true });
      modalContentEl.addEventListener('touchmove', (e) => {
        e.stopPropagation();
      }, { passive: true });
    }

    el.chkInvertPitch.addEventListener('change', (e) => { settings.invertPitch = e.target.checked; saveSettings(); });
    el.chkInvertRoll.addEventListener('change', (e) => { settings.invertRoll = e.target.checked; saveSettings(); });
    el.chkInvertYaw.addEventListener('change', (e) => { settings.invertYaw = e.target.checked; saveSettings(); });

    el.sliderTrimP.addEventListener('input', (e) => { settings.pitchTrim = parseInt(e.target.value); saveSettings(); });
    el.sliderTrimR.addEventListener('input', (e) => { settings.rollTrim = parseInt(e.target.value); saveSettings(); });
    el.sliderTrimY.addEventListener('input', (e) => { settings.yawTrim = parseInt(e.target.value); saveSettings(); });

    el.btnModalPDec.addEventListener('click', () => { settings.pitchTrim = Math.max(-100, settings.pitchTrim - 2); saveSettings(); });
    el.btnModalPInc.addEventListener('click', () => { settings.pitchTrim = Math.min(100, settings.pitchTrim + 2); saveSettings(); });
    el.btnModalRDec.addEventListener('click', () => { settings.rollTrim = Math.max(-100, settings.rollTrim - 2); saveSettings(); });
    el.btnModalRInc.addEventListener('click', () => { settings.rollTrim = Math.min(100, settings.rollTrim + 2); saveSettings(); });
    el.btnModalYDec.addEventListener('click', () => { settings.yawTrim = Math.max(-100, settings.yawTrim - 2); saveSettings(); });
    el.btnModalYInc.addEventListener('click', () => { settings.yawTrim = Math.min(100, settings.yawTrim + 2); saveSettings(); });

    el.btnResetTrims.addEventListener('click', () => {
      settings.pitchTrim = 0;
      settings.rollTrim = 0;
      settings.yawTrim = 0;
      saveSettings();
      if (navigator.vibrate) navigator.vibrate(30);
    });

    el.sliderMaxThr.addEventListener('input', (e) => { settings.maxThrottle = parseInt(e.target.value); saveSettings(); });
    el.sliderMaxAngle.addEventListener('input', (e) => { settings.maxAngle = parseInt(e.target.value); saveSettings(); });

    // Quick Gimbal Trim Buttons
    function stepTrim(key, delta) {
      settings[key] = Math.max(-100, Math.min(100, settings[key] + delta));
      saveSettings();
      if (navigator.vibrate) navigator.vibrate(15);
    }
    el.btnTrimPitchInc.addEventListener('click', () => stepTrim('pitchTrim', +2));
    el.btnTrimPitchDec.addEventListener('click', () => stepTrim('pitchTrim', -2));
    el.btnTrimRollInc.addEventListener('click', () => stepTrim('rollTrim', +2));
    el.btnTrimRollDec.addEventListener('click', () => stepTrim('rollTrim', -2));
    el.btnTrimYawInc.addEventListener('click', () => stepTrim('yawTrim', +2));
    el.btnTrimYawDec.addEventListener('click', () => stepTrim('yawTrim', -2));

    // =========================================================================
    // Dual Virtual Gimbal Touch Engine
    // =========================================================================
    function setupGimbal(zoneEl, puckEl, isThrottle, onChange) {
      let touchId = null;
      let maxRadius = 70;

      function updateRadius() {
        maxRadius = (zoneEl.clientWidth / 2) - 28;
      }
      updateRadius();
      window.addEventListener('resize', updateRadius);

      function handleMove(clientX, clientY) {
        const rect = zoneEl.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        let dx = clientX - centerX;
        let dy = clientY - centerY;

        // Clamp to circle boundary
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > maxRadius) {
          dx = (dx / dist) * maxRadius;
          dy = (dy / dist) * maxRadius;
        }

        puckEl.style.transform = `translate(${dx}px, ${dy}px)`;
        puckEl.classList.add('active');

        const normX = dx / maxRadius;
        const normY = -(dy / maxRadius); // up is +1

        onChange(normX, normY);
      }

      function handleRelease() {
        touchId = null;
        puckEl.classList.remove('active');

        if (isThrottle && ctrl.holdThrottle) {
          const currentY = parseFloat(puckEl.style.transform.split(',')[1]) || 0;
          puckEl.style.transform = `translate(0px, ${currentY}px)`;
          onChange(0, -(currentY / maxRadius));
        } else {
          puckEl.style.transform = 'translate(0px, 0px)';
          onChange(0, 0);
        }
      }

      zoneEl.addEventListener('touchstart', (e) => {
        if (touchId !== null) return;
        const touch = e.changedTouches[0];
        touchId = touch.identifier;
        handleMove(touch.clientX, touch.clientY);
      }, { passive: false });

      window.addEventListener('touchmove', (e) => {
        for (let i = 0; i < e.changedTouches.length; i++) {
          const t = e.changedTouches[i];
          if (t.identifier === touchId) {
            handleMove(t.clientX, t.clientY);
            break;
          }
        }
      }, { passive: false });

      window.addEventListener('touchend', (e) => {
        for (let i = 0; i < e.changedTouches.length; i++) {
          if (e.changedTouches[i].identifier === touchId) {
            handleRelease();
            break;
          }
        }
      });

      window.addEventListener('touchcancel', (e) => {
        for (let i = 0; i < e.changedTouches.length; i++) {
          if (e.changedTouches[i].identifier === touchId) {
            handleRelease();
            break;
          }
        }
      });
    }

    // Left Gimbal Bindings: Throttle (0 to 1000), Yaw (-500 to +500)
    setupGimbal(el.gimbalLeft, el.puckLeft, true, (nx, ny) => {
      ctrl.rawThrottle = Math.max(0, Math.min(1000, Math.round(((ny + 1) / 2) * 1000)));
      ctrl.rawYaw = Math.max(-500, Math.min(500, Math.round(nx * 500)));

      const thrPct = Math.round((ctrl.rawThrottle / 1000) * 100);
      el.readoutThr.textContent = `${thrPct}%`;
      el.readoutYaw.textContent = `${Math.round((ctrl.rawYaw / 500) * 200)}°/s`;
    });

    // Right Gimbal Bindings: Pitch (-500 to +500), Roll (-500 to +500)
    setupGimbal(el.gimbalRight, el.puckRight, false, (nx, ny) => {
      ctrl.rawPitch = Math.max(-500, Math.min(500, Math.round(ny * 500)));
      ctrl.rawRoll = Math.max(-500, Math.min(500, Math.round(nx * 500)));

      el.readoutPitch.textContent = `${((ctrl.rawPitch / 500) * settings.maxAngle).toFixed(1)}°`;
      el.readoutRoll.textContent = `${((ctrl.rawRoll / 500) * settings.maxAngle).toFixed(1)}°`;
    });

    // =========================================================================
    // Slide to Arm Mechanism
    // =========================================================================
    let armDrag = false;
    let armStartX = 0;
    const maxArmSlide = 120;

    el.armHandle.addEventListener('touchstart', (e) => {
      if (ctrl.armed) return;
      if (ctrl.rawThrottle > 50) {
        if (navigator.vibrate) navigator.vibrate(100);
        alert("Safety Lockout: Lower throttle to 0% before arming!");
        return;
      }
      armDrag = true;
      armStartX = e.touches[0].clientX;
    });

    window.addEventListener('touchmove', (e) => {
      if (!armDrag) return;
      const dx = Math.max(0, Math.min(maxArmSlide, e.touches[0].clientX - armStartX));
      el.armHandle.style.transform = `translateX(${dx}px)`;

      if (dx >= maxArmSlide - 8) {
        armDrag = false;
        armDrone();
      }
    });

    window.addEventListener('touchend', () => {
      if (!armDrag) return;
      armDrag = false;
      el.armHandle.style.transform = 'translateX(0px)';
    });

    function armDrone() {
      ctrl.armed = true;
      ctrl.arm = 1;
      el.armTrack.classList.add('armed');
      el.armLabel.textContent = "ARMED - TAP TO DISARM";
      el.armHandle.style.transform = `translateX(${maxArmSlide}px)`;
      if (navigator.vibrate) navigator.vibrate([40, 60, 40]);
    }

    function disarmDrone() {
      ctrl.armed = false;
      ctrl.arm = 0;
      el.armTrack.classList.remove('armed');
      el.armLabel.textContent = "Slide to Arm >>";
      el.armHandle.style.transform = 'translateX(0px)';
      if (navigator.vibrate) navigator.vibrate(50);
    }

    el.armTrack.addEventListener('click', () => {
      if (ctrl.armed) {
        disarmDrone();
      }
    });

    // Emergency Kill
    el.btnKill.addEventListener('click', () => {
      ctrl.arm = 2; // Emergency stop
      disarmDrone();
      if (navigator.vibrate) navigator.vibrate([100, 50, 100]);
    });

    // Toggle Throttle Hold / Spring
    el.btnThrMode.addEventListener('click', () => {
      ctrl.holdThrottle = !ctrl.holdThrottle;
      el.btnThrMode.textContent = ctrl.holdThrottle ? 'Hold Thr' : 'Spring Thr';
    });

    // Calibrate Level
    el.btnCalib.addEventListener('click', () => {
      if (ctrl.armed) {
        alert("Cannot calibrate while armed!");
        return;
      }
      if (confirm("Calibrate IMU? Drone must be flat and completely motionless.")) {
        sendPacket({ calib: 1 });
      }
    });

    // =========================================================================
    // Swarm Target Selection (All / Leader / Followers)
    // =========================================================================
    function setControlTarget(tgt) {
      ctrl.targetNode = tgt;
      document.querySelectorAll('.target-pill').forEach(b => {
        b.classList.toggle('active', parseInt(b.dataset.target, 10) === tgt);
      });
      if (el.targetBadge) {
        if (tgt === 255) {
          el.targetBadge.textContent = "ALL (SWARM)";
          el.targetBadge.className = "target-badge swarm";
        } else if (tgt === 1) {
          el.targetBadge.textContent = "LEADER (#1)";
          el.targetBadge.className = "target-badge leader";
        } else {
          el.targetBadge.textContent = `FOLLOWER (#${tgt})`;
          el.targetBadge.className = "target-badge follower";
        }
      }
      if (navigator.vibrate) navigator.vibrate(30);
    }

    document.querySelectorAll('.target-pill').forEach(btn => {
      btn.addEventListener('click', () => {
        const tgt = parseInt(btn.dataset.target, 10);
        setControlTarget(tgt);
      });
    });

    // =========================================================================
    // WebSocket High-Speed Engine (25 Hz)
    // =========================================================================
    function initWebSocket() {
      const wsUrl = `ws://${window.location.host}/ws`;
      ctrl.ws = new WebSocket(wsUrl);

      ctrl.ws.onopen = () => {
        el.connDot.className = 'status-dot online';
        el.overlay.classList.remove('show');
        clearTimeout(ctrl.failsafeTimer);
      };

      ctrl.ws.onmessage = (e) => {
        try {
          const telem = JSON.parse(e.data);
          // Ping calculation
          if (ctrl.lastPingSent) {
            ctrl.pingMs = Date.now() - ctrl.lastPingSent;
            el.valPing.textContent = `${ctrl.pingMs}ms`;
          }

          // Telemetry HUD updates
          if (telem.b !== undefined) el.valBatt.textContent = `${telem.b.toFixed(2)}V`;
          if (telem.s !== undefined) {
            el.valState.textContent = telem.s;
            if (telem.s === 'ARMED' || telem.s === 'FLIGHT') {
              el.connDot.className = 'status-dot armed';
            } else {
              el.connDot.className = 'status-dot online';
            }
          }
          if (telem.peers !== undefined && el.boxSwarm && el.valSwarm) {
            el.boxSwarm.style.display = 'inline-flex';
            el.valSwarm.textContent = telem.peers > 0 ? `${telem.peers} NODES` : 'STANDALONE';
          }

          // Dynamic Swarm Follower Node Discovery
          if (Array.isArray(telem.nodes) && el.targetPillGroup) {
            telem.nodes.forEach(nodeId => {
              if (nodeId > 1 && !document.querySelector(`.target-pill[data-target="${nodeId}"]`)) {
                const btn = document.createElement('button');
                btn.className = 'target-pill';
                btn.dataset.target = nodeId;
                btn.textContent = `🤖 FOLL (#${nodeId})`;
                btn.addEventListener('click', () => setControlTarget(nodeId));
                el.targetPillGroup.appendChild(btn);
              }
            });
          }

          // Live Follower Telemetry in badge if targeted
          if (ctrl.targetNode !== 255 && ctrl.targetNode !== 1 && el.targetBadge) {
            if (telem.fb > 0) {
              el.targetBadge.textContent = `FOLLOWER #${ctrl.targetNode}: ${telem.fb.toFixed(2)}V ${telem.fa ? '(ARMED)' : '(DISARMED)'}`;
            }
          }

          // Live Attitude Horizon
          if (telem.p !== undefined && telem.r !== undefined) {
            el.attPitch.textContent = `${telem.p >= 0 ? '+' : ''}${telem.p.toFixed(1)}°`;
            el.attRoll.textContent = `${telem.r >= 0 ? '+' : ''}${telem.r.toFixed(1)}°`;

            // Transform sky background: rotate by -roll, translate by pitch
            const pitchOffset = Math.max(-28, Math.min(28, telem.p * 1.2));
            el.pfdSky.style.transform = `rotate(${-telem.r}deg) translateY(${pitchOffset}px)`;
          }

          // Reset safety timeout (1500 ms to tolerate mobile Wi-Fi latency jitter)
          clearTimeout(ctrl.failsafeTimer);
          ctrl.failsafeTimer = setTimeout(() => {
            el.connDot.className = 'status-dot';
            el.overlay.classList.add('show');
          }, 1500);

        } catch (err) {
          console.warn("Telem parse error:", err);
        }
      };

      ctrl.ws.onerror = (err) => {
        console.warn("WebSocket error:", err);
      };

      ctrl.ws.onclose = () => {
        el.connDot.className = 'status-dot';
        el.overlay.classList.add('show');
        setTimeout(initWebSocket, 1500);
      };
    }

    function sendPacket(extra = {}) {
      if (ctrl.ws && ctrl.ws.readyState === WebSocket.OPEN) {
        ctrl.lastPingSent = Date.now();

        // 1. Apply Stick Inversions & Trim Adjustments
        let effectivePitch = (settings.invertPitch ? -1 : 1) * ctrl.rawPitch + (settings.pitchTrim * 5);
        let effectiveRoll  = (settings.invertRoll  ? -1 : 1) * ctrl.rawRoll  + (settings.rollTrim * 5);
        let effectiveYaw   = (settings.invertYaw   ? -1 : 1) * ctrl.rawYaw   + (settings.yawTrim * 5);

        // Clamp to [-500, +500]
        effectivePitch = Math.max(-500, Math.min(500, Math.round(effectivePitch)));
        effectiveRoll  = Math.max(-500, Math.min(500, Math.round(effectiveRoll)));
        effectiveYaw   = Math.max(-500, Math.min(500, Math.round(effectiveYaw)));

        // 2. Apply Max Throttle Ceiling Limit
        const throttleCeiling = (settings.maxThrottle / 100.0) * 1000.0;
        const effectiveThrottle = Math.min(ctrl.rawThrottle, throttleCeiling);

        const payload = Object.assign({
          t: Math.round(effectiveThrottle),
          y: effectiveYaw,
          p: effectivePitch,
          r: effectiveRoll,
          a: ctrl.arm,
          target: ctrl.targetNode
        }, extra);

        ctrl.ws.send(JSON.stringify(payload));
      }
    }

    // 25 Hz Control Loop (40 ms interval - rock solid on mobile Wi-Fi)
    setInterval(() => {
      sendPacket();
    }, 40);

    el.btnReconnect.addEventListener('click', () => {
      initWebSocket();
    });

    window.addEventListener('load', () => {
      loadSettings();
      updateSettingsUI();
      initWebSocket();
    });
  </script>
</body>
</html>
)rawliteral";
