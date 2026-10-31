#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_v2 综合性能表
--------------------------------------------------------------------------
用法：
    python perf_table.py --port COM4
    python perf_table.py --port COM4 --angles 30,60,90,120,180,270
    python perf_table.py --port COM4 --param profA=40000

每个角度只走一次，同一次录波里同时算：
    上升时间 / 超调 / 调节时间 / 稳态误差 / 到位后静置抖动

「到位后静置抖动」用最后 600ms 的峰峰值，和 jitter_survey 的判定口径一致。
"""

import argparse
import sys
import time

import serial

from step_test import read_frames, metrics

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def tail_ptp(t, y, last_ms=600):
    """最后 last_ms 内的峰峰值，代表静置抖动"""
    if not y:
        return float("nan")
    t_end = t[-1]
    seg = [yi for ti, yi in zip(t, y) if ti >= t_end - last_ms / 1000.0]
    if len(seg) < 2:
        return float("nan")
    return max(seg) - min(seg)


def verdict(ptp):
    if ptp < 0.02:
        return "静止"
    if ptp < 0.06:
        return "极微"
    if ptp < 0.15:
        return "微抖"
    return "抖动"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--angles", default="30,60,90,120,180,270")
    ap.add_argument("--ms", type=int, default=1500, help="每次录多久（ms）")
    ap.add_argument("--param", action="append", default=[])
    args = ap.parse_args()

    angles = [float(s) for s in args.angles.split(",") if s.strip()]

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    buf = bytearray()
    time.sleep(0.3)
    ser.reset_input_buffer()

    for kv in args.param:
        ser.write(("$%s!" % kv).encode())
        time.sleep(0.06)
        print("  设置 %s" % kv)

    ser.write(b"$wave=2!")
    time.sleep(0.15)

    # 先回 0 做统一起点
    ser.write(b"=0!")
    time.sleep(1.2)

    print()
    print("  目标    起点      转角    上升(ms)  超调(%)  调节(ms)  稳态差(°)  抖动(°)  判定")
    print("  " + "-" * 84)

    rows = []
    for ang in angles:
        ser.reset_input_buffer()
        buf.clear()
        # 抓一下当前位置，用于算真实转角（最短路径可能反向）
        pre = read_frames(ser, time.time() + 0.12, buf)
        y0 = pre[-1][1] if pre else 0.0

        ser.reset_input_buffer()
        buf.clear()
        ser.write(("=%g!" % ang).encode())
        data = read_frames(ser, time.time() + args.ms / 1000.0, buf)
        if len(data) < 100:
            print("  %6.1f  录数据失败" % ang)
            continue

        t = [k * 0.002 for k in range(len(data))]
        ref = [d[0] for d in data]
        y = [d[1] for d in data]

        # 转角按最短路径算：目标 270 从 0 出发实际走 -90
        d = (ang - y0 + 180.0) % 360.0 - 180.0
        tgt = y0 + d

        m = metrics(t, y, tgt, 0.0)
        if m is None:
            print("  %6.1f  阶跃量太小" % ang)
            continue
        jit = tail_ptp(t, y)
        v = verdict(jit)
        rows.append((ang, d, m, jit, v))
        print("  %6.1f  %7.2f  %+7.2f  %8.1f  %7.2f  %8.1f  %+9.4f  %7.4f  %s"
              % (ang, y0, d, m["rise"], m["overshoot"], m["settle"], m["sse"], jit, v))

    if rows:
        print()
        print("  平均上升时间 : %.1f ms" % (sum(r[2]["rise"] for r in rows) / len(rows)))
        print("  最大超调     : %.2f %%" % max(r[2]["overshoot"] for r in rows))
        print("  最大调节时间 : %.1f ms" % max(r[2]["settle"] for r in rows))
        print("  最大稳态误差 : %.4f °" % max(abs(r[2]["sse"]) for r in rows))
        print("  最大抖动     : %.4f °" % max(r[3] for r in rows))
    ser.close()


if __name__ == "__main__":
    main()
