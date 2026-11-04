#!/usr/bin/env python3
"""Offline 21-keypoint PICO right-hand skeleton viewer.

Reads the raw JSONL made by pico_udp_inspector.py, or the unextracted ZIP
containing it. Does not create sockets, ROS 2 nodes, or API2 clients.

The five groups of four points are an UNVERIFIED topology hypothesis.
Bend angles are geometric segment-direction changes, NOT calibrated joint angles.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import zipfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable

import numpy as np

# Imported only after the CLI has been parsed and the input checked.

GROUPS = {
    "Thumb? (1-4)": (1, 2, 3, 4),
    "Index? (5-8)": (5, 6, 7, 8),
    "Middle? (9-12)": (9, 10, 11, 12),
    "Ring? (13-16)": (13, 14, 15, 16),
    "Little? (17-20)": (17, 18, 19, 20),
}
# Group-to-finger anatomy is a hypothesis, not an SDK contract.
COLORS = ("#a36dc7", "#e8994a", "#3c9e95", "#5795d4", "#d16f82")


@dataclass
class Frame:
    elapsed: float
    seq: int | None
    tracked: bool
    deadman: bool
    input_mode: str
    valid: bool
    xyz: np.ndarray  # (21, 3), raw PICO position coordinates
    source_timestamp: float | None


def _finite_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _parse_positions(payload: dict) -> tuple[np.ndarray, bool]:
    positions = np.full((21, 3), np.nan, dtype=float)
    kp = payload.get("right_keypoints")
    if not isinstance(kp, list) or len(kp) != 21:
        return positions, False
    valid = True
    for j, point in enumerate(kp):
        if not isinstance(point, dict) or point.get("valid") is not True:
            valid = False
            continue
        pos = point.get("position")
        if not isinstance(pos, dict):
            valid = False
            continue
        vals = [pos.get(a) for a in "xyz"]
        if not all(_finite_number(v) for v in vals):
            valid = False
            continue
        positions[j] = vals
    return positions, valid


def _jsonl_lines(path: Path) -> Iterable[str]:
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as archive:
            names = [name for name in archive.namelist()
                     if name.lower().endswith(".jsonl") and not name.startswith("__MACOSX/")]
            if not names:
                raise ValueError("ZIP does not contain a .jsonl file")
            if len(names) > 1:
                raise ValueError("ZIP has multiple JSONL files. Extract and select one with --input")
            print(f"[INPUT] ZIP member: {names[0]}")
            with archive.open(names[0]) as src:
                with io.TextIOWrapper(src, encoding="utf-8-sig") as text_stream:
                    yield from text_stream
    else:
        with path.open("r", encoding="utf-8-sig") as source:
            yield from source


def load_capture(path: Path) -> tuple[list[Frame], dict]:
    frames: list[Frame] = []
    dropped = 0
    t0: float | None = None
    for line_no, line in enumerate(_jsonl_lines(path), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
            # Inspector's JSONL line is {capture_time_utc, raw_text, ...}.
            # A direct raw-packet JSONL is also accepted.
            payload = json.loads(row["raw_text"]) if isinstance(row.get("raw_text"), str) else row
            if not isinstance(payload, dict):
                raise ValueError("packet is not a JSON object")
            time_raw = row.get("capture_time_utc")
            if time_raw:
                capture_seconds = datetime.fromisoformat(time_raw.replace("Z", "+00:00")).timestamp()
            elif _finite_number(row.get("capture_time_unix")):
                capture_seconds = float(row["capture_time_unix"])
            elif _finite_number(payload.get("timestamp")):
                capture_seconds = float(payload["timestamp"])
            else:
                capture_seconds = float(line_no - 1) / 30.0  # fallback is display-only
            if t0 is None:
                t0 = capture_seconds
            elapsed = max(0.0, capture_seconds - t0)
            xyz, keypoints_ok = _parse_positions(payload)
            tracked = payload.get("right_hand_tracked") is True
            # Do not render stale coordinates from an invalid frame.
            valid = tracked and keypoints_ok
            seq_raw = payload.get("seq")
            seq = seq_raw if isinstance(seq_raw, int) and not isinstance(seq_raw, bool) else None
            src_ts = payload.get("timestamp")
            frames.append(Frame(
                elapsed=elapsed, seq=seq, tracked=tracked,
                deadman=payload.get("deadman") is True,
                input_mode=str(payload.get("input_mode", "unknown")),
                valid=valid, xyz=xyz,
                source_timestamp=float(src_ts) if _finite_number(src_ts) else None,
            ))
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            dropped += 1
            if dropped <= 5:
                print(f"[WARN] Invalid JSONL row {line_no}: {exc}")
    if not frames:
        raise ValueError("No valid UDP packets found. Check the input format")
    valid_indices = [i for i, f in enumerate(frames) if f.valid]
    if not valid_indices:
        raise ValueError("No valid tracked frames with 21 finite keypoints were found")
    return frames, {"invalid_rows": dropped, "valid_indices": valid_indices}


def bend_angle(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    """Angle between the forward segments p0->p1 and p1->p2 in degrees.

    Straight segments give 0 degrees. Not a signed angle or robot joint angle.
    """
    v1 = p1 - p0
    v2 = p2 - p1
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    if not np.isfinite(norm) or norm < 1e-12:
        return float("nan")
    return float(np.degrees(np.arccos(np.clip(np.dot(v1, v2) / norm, -1.0, 1.0))))


def compute_bends(frames: list[Frame]) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for name, joint_ids in GROUPS.items():
        series = np.full((len(frames), 2), np.nan, dtype=float)
        a, b, c, d = joint_ids
        for i, f in enumerate(frames):
            if f.valid:
                series[i, 0] = bend_angle(f.xyz[a], f.xyz[b], f.xyz[c])
                series[i, 1] = bend_angle(f.xyz[b], f.xyz[c], f.xyz[d])
        result[name] = series
    return result


def show_viewer(frames: list[Frame], stats: dict, *, first_frame: int | None = None,
                export_preview: Path | None = None) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, Slider

    times = np.array([f.elapsed for f in frames], dtype=float)
    bends = compute_bends(frames)
    names = list(GROUPS)
    selected = names[1]  # Index candidate, only for easier first inspection
    valid_idx = stats["valid_indices"]
    index = valid_idx[0] if first_frame is None else min(max(first_frame, 0), len(frames)-1)
    playing = False
    updating_slider = False

    # One fixed scale for every frame; wrist-relative coordinates remove global translation.
    valid_positions = np.array([frames[i].xyz - frames[i].xyz[0] for i in valid_idx])
    axis_radius = max(0.055, float(np.nanpercentile(np.abs(valid_positions), 99.7)) * 1.10)
    axis_radius = min(axis_radius, 3.0)  # protect display from pathological samples

    fig = plt.figure(figsize=(15, 8.5), facecolor="#fcfcfc")
    fig.canvas.manager.set_window_title("PICO 21-keypoint offline hand viewer") if hasattr(fig.canvas.manager, "set_window_title") else None
    ax3d = fig.add_axes((0.025, 0.23, 0.58, 0.62), projection="3d")
    angle_ax = fig.add_axes((0.66, 0.39, 0.32, 0.48))
    info_text = fig.text(0.04, 0.985, "", va="top", ha="left", fontsize=11, family="monospace")
    note_text = fig.text(0.04, 0.945,
                         "Topology = HYPOTHESIS (wrist 0 + five groups x4). No O7 commands.",
                         va="top", fontsize=9, color="#9b4636")

    ax3d.set_title("3D keypoints (drag to rotate, wheel to zoom)", fontsize=13, pad=16)
    ax3d.set_xlabel("X (source units)")
    ax3d.set_ylabel("Y (source units)")
    ax3d.set_zlabel("Z (source units)")
    ax3d.set_xlim(-axis_radius, axis_radius)
    ax3d.set_ylim(-axis_radius, axis_radius)
    ax3d.set_zlim(-axis_radius, axis_radius)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.view_init(elev=22, azim=-65)

    line_artists = []
    for color, (name, ids) in zip(COLORS, GROUPS.items()):
        (line,) = ax3d.plot([], [], [], "-o", lw=2.6, ms=5.5, color=color,
                            label=name, alpha=0.95)
        line_artists.append((line, (0, *ids)))
    (wrist_marker,) = ax3d.plot([], [], [], "ko", ms=8, label="Wrist 0")
    labels = [ax3d.text(0, 0, 0, str(i), fontsize=8.8, weight="bold", color="#222222")
              for i in range(21)]
    ax3d.legend(loc="lower left", fontsize=8, borderpad=0.6)

    (bend1_line,) = angle_ax.plot([], [], lw=1.8, color="#d77e31", label="bend at group[1]")
    (bend2_line,) = angle_ax.plot([], [], lw=1.8, color="#467ac0", label="bend at group[2]")
    cursor_line = angle_ax.axvline(times[index], color="#313131", ls="--", lw=1.3)
    angle_ax.set_xlabel("Elapsed capture time (s)")
    angle_ax.set_ylabel("Segment-direction bend (degrees)")
    angle_ax.set_xlim(0, max(1.0, times[-1]))
    angle_ax.set_ylim(0, 180)
    angle_ax.grid(True, alpha=0.24)
    angle_ax.legend(loc="upper right", fontsize=8)
    live_angle_text = fig.text(0.67, 0.325, "", family="monospace", fontsize=10)

    radio_ax = fig.add_axes((0.71, 0.075, 0.24, 0.19), facecolor="#f7f7f7")
    radio_ax.set_title("Candidate finger", fontsize=10, loc="left")
    radio = RadioButtons(radio_ax, names, active=1)
    for lab in radio.labels:
        lab.set_fontsize(9)

    slider_ax = fig.add_axes((0.12, 0.15, 0.45, 0.026))
    slider = Slider(slider_ax, "Frame", valmin=0, valmax=len(frames)-1,
                    valinit=index, valstep=1, valfmt="%0.0f")
    prev_ax = fig.add_axes((0.08, 0.065, 0.085, 0.044))
    play_ax = fig.add_axes((0.18, 0.065, 0.095, 0.044))
    next_ax = fig.add_axes((0.29, 0.065, 0.085, 0.044))
    prev_btn = Button(prev_ax, "< Prev")
    play_btn = Button(play_ax, "Play")
    next_btn = Button(next_ax, "Next >")
    fig.text(0.045, 0.025,
             "Keyboard: Left/Right = frame, Space = play/pause, Home/End = first/last  |  This is offline only",
             fontsize=9, color="#444444")

    def plot_selected() -> None:
        series = bends[selected]
        bend1_line.set_data(times, series[:, 0])
        bend2_line.set_data(times, series[:, 1])
        angle_ax.set_title(f"{selected} - geometric bend", fontsize=12)

    def render() -> None:
        nonlocal index
        f = frames[index]
        info_text.set_text(
            f"Frame {index + 1:>4}/{len(frames)} | t={f.elapsed:6.2f}s | seq={f.seq} | "
            f"tracked={f.tracked} | deadman={f.deadman} | mode={f.input_mode}"
        )
        if f.valid:
            rel = f.xyz - f.xyz[0]
            for line, ids in line_artists:
                line.set_data_3d(rel[list(ids), 0], rel[list(ids), 1], rel[list(ids), 2])
            wrist_marker.set_data_3d([0], [0], [0])
            for i, lab in enumerate(labels):
                xyz = rel[i]
                lab.set_position((float(xyz[0]), float(xyz[1])))
                lab.set_3d_properties(float(xyz[2]), zdir="z")
                lab.set_visible(True)
            note_text.set_text("Topology = HYPOTHESIS (wrist 0 + five groups x4). No O7 commands.")
            note_text.set_color("#9b4636")
        else:
            for line, _ in line_artists:
                line.set_data_3d([], [], [])
            wrist_marker.set_data_3d([], [], [])
            for lab in labels:
                lab.set_visible(False)
            note_text.set_text("INVALID TRACKING: skeleton hidden; stale/held positions are NOT displayed.")
            note_text.set_color("#b72525")
        cursor_line.set_xdata([f.elapsed, f.elapsed])
        b1, b2 = bends[selected][index]
        fmt = lambda x: f"{x:.1f} deg" if np.isfinite(x) else "N/A (invalid)"
        live_angle_text.set_text(f"Frame bends:  inner={fmt(b1)}  outer={fmt(b2)}")
        fig.canvas.draw_idle()

    def set_index(new_index: int) -> None:
        nonlocal index, updating_slider
        new_index = max(0, min(len(frames)-1, int(new_index)))
        if new_index == index:
            return
        index = new_index
        updating_slider = True
        try:
            slider.set_val(index)
        finally:
            updating_slider = False
        render()

    def on_slider(value: float) -> None:
        nonlocal index
        if updating_slider:
            return
        index = int(value)
        render()

    def on_finger(name: str) -> None:
        nonlocal selected
        selected = name
        plot_selected()
        render()

    def on_play(event=None) -> None:
        nonlocal playing
        playing = not playing
        play_btn.label.set_text("Pause" if playing else "Play")
        fig.canvas.draw_idle()

    def on_timer() -> None:
        nonlocal playing
        if not playing:
            return
        if index >= len(frames)-1:
            playing = False
            play_btn.label.set_text("Play")
        else:
            set_index(index + 1)

    def on_key(event) -> None:
        if event.key in ("left", "right", "home", "end", " "):
            if event.key == "left":
                set_index(index-1)
            elif event.key == "right":
                set_index(index+1)
            elif event.key == "home":
                set_index(0)
            elif event.key == "end":
                set_index(len(frames)-1)
            elif event.key == " ":
                on_play()

    slider.on_changed(on_slider)
    radio.on_clicked(on_finger)
    prev_btn.on_clicked(lambda _: set_index(index-1))
    next_btn.on_clicked(lambda _: set_index(index+1))
    play_btn.on_clicked(on_play)
    fig.canvas.mpl_connect("key_press_event", on_key)
    deltas = np.diff(times)
    deltas = deltas[(deltas > 0.0) & (deltas < 0.25)]
    timer_ms = int(np.clip(np.median(deltas) * 1000 if len(deltas) else 40, 20, 150))
    timer = fig.canvas.new_timer(interval=timer_ms)
    timer.add_callback(on_timer)
    timer.start()

    plot_selected()
    render()
    print(f"[VIEWER] frames={len(frames)}, valid={len(valid_idx)}, "
          f"time={times[-1]:.2f}s, median_dt={timer_ms}ms, "
          f"invalid_JSONL_rows={stats['invalid_rows']}")
    print("[VIEWER] Skeleton groups and joint bends are hypotheses. No commands will be sent.")
    if export_preview is not None:
        export_preview.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(export_preview, dpi=150, bbox_inches="tight")
        print(f"[VIEWER] Preview saved to {export_preview}")
        plt.close(fig)
        return
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline PICO right-hand keypoint viewer (no robot access)")
    ap.add_argument("--input", type=Path, required=True,
                    help="pico_raw_*.jsonl or pico_udp_captures.zip")
    ap.add_argument("--frame", type=int, default=None,
                    help="Zero-based starting frame; default is first valid tracked frame")
    ap.add_argument("--save-preview", type=Path,
                    help="Save one static PNG preview and exit (headless mode)")
    args = ap.parse_args()
    if not args.input.is_file():
        ap.error(f"file not found: {args.input}")
    if args.save_preview is not None:
        import matplotlib
        matplotlib.use("Agg")
    frames, stats = load_capture(args.input)
    show_viewer(frames, stats, first_frame=args.frame, export_preview=args.save_preview)


if __name__ == "__main__":
    main()
