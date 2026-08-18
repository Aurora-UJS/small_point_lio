#!/usr/bin/env python3
"""把同一段录制数据喂给两个版本的 small_point_lio，比轨迹。

用途：改滤波器之后回答「改动到底动了什么、动了多少」。
这类改动通常不会崩，只会让轨迹慢慢飘，不做定量对比根本发现不了。

    tools/compare_runs.py --bag run.mcap --config config/mid360_real.yaml \\
                          --ref ros2 --ref fix/eskf-satu-acc-index

每个 ref 会在临时 git worktree 里独立构建（无 ROS 构建，快），跑同一份录制，
输出 TUM 轨迹，然后逐时刻对齐比较。

也可以跳过构建，直接比两个已有的可执行文件：

    tools/compare_runs.py --bag run.mcap --config cfg.yaml \\
                          --binary A=/path/a --binary B=/path/b
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent


def run(cmd, **kwargs):
    result = subprocess.run(cmd, capture_output=True, text=True, **kwargs)
    if result.returncode != 0:
        sys.exit(f"命令失败: {' '.join(map(str, cmd))}\n{result.stdout}\n{result.stderr}")
    return result


def build_ref(ref, workdir, index):
    """在临时 worktree 里 checkout 某个 ref 并构建无 ROS 版本，返回可执行文件路径。"""
    slug = f"{index}_" + (re.sub(r"[^A-Za-z0-9]", "_", ref) or "ref")
    tree = workdir / f"src_{slug}"
    build = workdir / f"build_{slug}"
    print(f"  [{ref}] 建 worktree ...", flush=True)
    run(["git", "worktree", "add", "--detach", str(tree), ref], cwd=REPO)
    print(f"  [{ref}] 构建 ...", flush=True)
    run(["cmake", "-S", str(tree), "-B", str(build),
         "-DSPL_WITH_ROS2=OFF", "-DCMAKE_BUILD_TYPE=Release"])
    run(["cmake", "--build", str(build), "-j", str(os.cpu_count() or 4)])
    binary = build / "small_point_lio_standalone"
    if not binary.exists():
        sys.exit(f"[{ref}] 构建产物不存在: {binary}")
    return binary


def make_config(base_config, bag, out_tum, dest):
    """把基准配置改成 standalone 回放模式，指向给定的 bag 和输出。"""
    text = Path(base_config).read_text()
    text = re.sub(r"^\s*transport:.*$", "        transport: standalone", text, flags=re.M)
    if "transport:" not in text:
        text = text.replace("ros__parameters:", "ros__parameters:\n        transport: standalone", 1)
    for key, value in (("replay_path", bag), ("odometry_output_path", out_tum), ("record_path", "")):
        pattern = rf"^\s*{key}:.*$"
        line = f'        {key}: "{value}"'
        text = re.sub(pattern, line, text, flags=re.M) if re.search(pattern, text, re.M) else \
            text.replace("ros__parameters:", f"ros__parameters:\n{line}", 1)
    Path(dest).write_text(text)
    return dest


SATU_RE = re.compile(r"陀螺饱和 (\d+) 次.*?加速度饱和 (\d+) 次")


def run_replay(binary, config):
    result = subprocess.run([str(binary), str(config)], capture_output=True, text=True)
    output = result.stdout + result.stderr
    if result.returncode != 0:
        sys.exit(f"回放失败:\n{output}")
    match = SATU_RE.search(output)
    saturation = (int(match.group(1)), int(match.group(2))) if match else (None, None)
    return saturation


def load_tum(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        parts = line.split()
        if len(parts) >= 8:
            rows.append([float(v) for v in parts[:8]])
    if not rows:
        sys.exit(f"轨迹文件为空: {path}")
    data = np.array(rows)
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def align(t_a, p_a, t_b, p_b):
    """按时间戳取交集，把 B 插值到 A 的时刻上。"""
    lo = max(t_a[0], t_b[0])
    hi = min(t_a[-1], t_b[-1])
    mask = (t_a >= lo) & (t_a <= hi)
    t = t_a[mask]
    if t.size == 0:
        sys.exit("两条轨迹时间没有重叠，没法比较")
    interp = np.stack([np.interp(t, t_b, p_b[:, i]) for i in range(3)], axis=1)
    return t, p_a[mask], interp


def plot(t, pa, pb, names, satu, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    # 图里有中文，挑一个系统里真有的中文字体，否则全是豆腐块
    available = {f.name for f in font_manager.fontManager.ttflist}
    # 图里中英混排，得挑一个中英都全的字体（matplotlib 的跨字体回退不可靠）。
    # 本机实测：AR PL UMing CN 零缺字，Droid Sans Fallback 缺 108 个。
    for candidate in ("Noto Sans CJK SC", "Source Han Sans SC", "AR PL UMing CN",
                      "WenQuanYi Zen Hei", "LXGW WenKai", "Droid Sans Fallback"):
        if candidate in available:
            plt.rcParams["font.sans-serif"] = [candidate]
            break
    plt.rcParams["axes.unicode_minus"] = False

    diff = pb - pa
    norm = np.linalg.norm(diff, axis=1)
    rel = t - t[0]
    headline = (f"{names[0]} vs {names[1]}：末端差 {norm[-1] * 100:.2f}cm，"
                f"最大 {norm.max() * 100:.2f}cm，Z 向最大 {np.abs(diff[:, 2]).max() * 100:.2f}cm")
    subtitle = "两条线贴合 => 改动没有改变轨迹；分离 => 改动生效了，看分离发生在什么时候"
    if satu[1] == 0:
        subtitle += "\n注意：这段数据加速度计从未饱和，satu 相关的改动在此为空操作"

    fig = plt.figure(figsize=(14, 9.6))
    fig.text(0.5, 0.975, headline, ha="center", va="top", fontsize=15, fontweight="bold")
    fig.text(0.5, 0.938, subtitle, ha="center", va="top", fontsize=10, style="italic", color="#444")

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.plot(pa[:, 0], pa[:, 1], pa[:, 2], lw=5, alpha=0.35, color="#1f77b4", label=names[0])
    ax3d.plot(pb[:, 0], pb[:, 1], pb[:, 2], lw=1.2, color="#d62728", label=names[1])
    ax3d.scatter(*pa[0], color="green", s=45, label="起点")
    ax3d.scatter(*pa[-1], color="black", s=45, marker="X", label="终点")
    ax3d.set_xlabel("x [m]"), ax3d.set_ylabel("y [m]"), ax3d.set_zlabel("z [m]")
    ax3d.set_title("轨迹总览（粗=基准，细=对比）", fontsize=10)
    ax3d.legend(fontsize=8)

    for index, (axis_name, row) in enumerate((("x", 0), ("y", 1), ("z", 2))):
        ax = fig.add_subplot(3, 2, 2 * (index + 1))
        ax.plot(rel, pa[:, row], lw=4, alpha=0.35, color="#1f77b4", label=names[0])
        ax.plot(rel, pb[:, row], lw=1.0, color="#d62728", label=names[1])
        ax.set_ylabel(f"{axis_name} [m]")
        ax.grid(alpha=0.3)
        if index == 0:
            ax.legend(fontsize=8, ncol=2)
        if index == 2:
            ax.set_xlabel("时间 [s]")

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(rel, norm * 100, color="#111", lw=1.4, label="总差值 |Δp|")
    ax.plot(rel, np.abs(diff[:, 2]) * 100, color="#ff7f0e", lw=1.2, ls="--", label="Z 向差值")
    ax.axhline(0, color="gray", lw=0.6)
    ax.annotate(f"末端 {norm[-1] * 100:.2f}cm", xy=(rel[-1], norm[-1] * 100),
                xytext=(-90, 12), textcoords="offset points", fontsize=9,
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.set_xlabel("时间 [s]"), ax.set_ylabel("差值 [cm]")
    ax.set_title("两版轨迹的分离随时间的增长", fontsize=10)
    ax.grid(alpha=0.3), ax.legend(fontsize=8)

    fig.subplots_adjust(left=0.07, right=0.97, top=0.875, bottom=0.07, hspace=0.45, wspace=0.22)
    fig.savefig(out_png, dpi=130)
    print(f"\n图已存到 {out_png}")
    return headline, norm, diff


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", required=True, help="录制文件（MCAP / rosbag2）")
    parser.add_argument("--config", required=True, help="基准配置 yaml")
    parser.add_argument("--ref", action="append", default=[], help="要比较的 git ref，给两次")
    parser.add_argument("--binary", action="append", default=[],
                        help="跳过构建，直接用现成可执行文件，格式 名字=路径")
    parser.add_argument("--out", default=None, help="输出目录（默认建临时目录）")
    args = parser.parse_args()

    targets = []
    for spec in args.binary:
        name, _, path = spec.partition("=")
        targets.append((name, Path(path)))
    pending_refs = args.ref if len(targets) < 2 else []
    if len(targets) + len(pending_refs) != 2:
        sys.exit("需要正好两个比较对象：给两个 --ref，或两个 --binary")

    out_dir = Path(args.out) if args.out else Path(tempfile.mkdtemp(prefix="spl_compare_"))
    out_dir.mkdir(parents=True, exist_ok=True)
    work = out_dir / "work"
    work.mkdir(exist_ok=True)

    created_worktrees = []
    try:
        for offset, ref in enumerate(pending_refs):
            index = len(targets)
            targets.append((ref, build_ref(ref, work, index)))
            slug = f"{index}_" + (re.sub(r"[^A-Za-z0-9]", "_", ref) or "ref")
            created_worktrees.append(work / f"src_{slug}")

        results = []
        for index, (name, binary) in enumerate(targets):
            # 加序号：名字可能是中文或只差符号，压成 ASCII 后会撞名，
            # 两次跑写进同一个文件就会「比出零差异」这种假结果。
            safe = f"{index}_" + (re.sub(r"[^A-Za-z0-9]", "_", name) or "run")
            tum = out_dir / f"{safe}.tum"
            config = make_config(args.config, Path(args.bag).resolve(), tum, out_dir / f"{safe}.yaml")
            print(f"  [{name}] 回放 ...", flush=True)
            satu = run_replay(binary, config)
            results.append((name, tum, satu))

        (name_a, tum_a, satu_a), (name_b, tum_b, satu_b) = results
        if tum_a == tum_b:
            sys.exit("两次运行的输出路径相同，比较无意义")
        t_a, p_a, _ = load_tum(tum_a)
        t_b, p_b, _ = load_tum(tum_b)
        t, pa, pb = align(t_a, p_a, t_b, p_b)

        print("\n========== 结果 ==========")
        print(f"对比时长 {t[-1] - t[0]:.1f}s，对齐后 {len(t)} 个时刻")
        for name, _, satu in results:
            print(f"  [{name}] 陀螺饱和 {satu[0]} 次，加速度饱和 {satu[1]} 次")
        if satu_a[1] == 0 and satu_b[1] == 0:
            print("  ⚠ 两次都没有加速度计饱和 —— 任何 satu_check 相关的改动在这段数据上都是空操作")

        headline, norm, diff = plot(t, pa, pb, (name_a, name_b), satu_b, out_dir / "compare.png")
        print(f"\n{headline}")
        print(f"  逐轴最大差值: x {np.abs(diff[:, 0]).max() * 100:.3f}cm  "
              f"y {np.abs(diff[:, 1]).max() * 100:.3f}cm  z {np.abs(diff[:, 2]).max() * 100:.3f}cm")
        print(f"  RMSE {np.sqrt((norm ** 2).mean()) * 100:.3f}cm")
        if norm.max() == 0.0:
            print("  => 两版轨迹逐位相同，改动在这段数据上没有任何影响")
        print(f"\n输出目录: {out_dir}")
    finally:
        for tree in created_worktrees:
            subprocess.run(["git", "worktree", "remove", "--force", str(tree)],
                           cwd=REPO, capture_output=True)


if __name__ == "__main__":
    main()
