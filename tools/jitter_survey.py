#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_v2 保持抖动普查工具
--------------------------------------------------------------------------
用法：
    python jitter_survey.py --port COM4
    python jitter_survey.py --port COM4 --angles 0,45,90,135,180,225,270,315
    python jitter_survey.py --port COM4 --settle 900 --hold 1500
    python jitter_survey.py --port COM4 --param profV=800 --param profA=20000

做的事：
    对每个目标角度：下发一次 =angle!，等它走完并稳定，然后只"看"不动，
    统计保持窗口内的位置波动。全程单向逐个走位，不做参数扫描、不来回抖。

    判定用峰值-峰值（ptp），因为「抖不抖」是人眼和耳朵的感受，
    标准差会把偶发的蹦一下平均掉，ptp 不会。

串口参数固定 115200 8N1，与固件一致。
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：python -m pip install pyserial")

TAIL = b"\x00\x00\x80\x7f"      # JustFloat 帧尾：float(+Inf)
CH = 2
FRAME = CH * 4 + 4
DT = 0.002                      # 固件固定 2ms 定拍，实测 500.0 帧/秒


def read_frames(ser, deadline, buf):
    """从串口流里切出完整的 JustFloat 帧，返回 [(c0, c1), ...]"""
    out = []
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(TAIL)
            if i < 0:
                if len(buf) > 4096:
                    del buf[:-FRAME]
                break
            if i < CH * 4:
                del buf[: i + 4]
                continue
            out.append(struct.unpack("<ff", buf[i - CH * 4 : i]))
            del buf[: i + 4]
    return out


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def stats(frames):
    """frames: [(ref_deg, act_deg)] -> 抖动统计"""
    act = [f[1] for f in frames]
    ref = [f[0] for f in frames]
    n = len(act)
    if n == 0:
        return None
    mean = sum(act) / n
    ptp = max(act) - min(act)
    var = sum((a - mean) ** 2 for a in act) / n
    std = var ** 0.5
    # 跟踪误差用连续域相减，不受跨圈影响
    err = sum(a - r for a, r in zip(act, ref)) / n
    return dict(n=n, mean=mean, ptp=ptp, std=std, err=err)


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
    ap.add_argument("--port", required=True, help="串口号，例如 COM4")
    ap.add_argument("--angles", default="0,45,90,135,180,225,270,315",
                    help="要普查的角度列表，逗号分隔")
    ap.add_argument("--settle", type=int, default=900, help="走位后等多久再开始统计（ms）")
    ap.add_argument("--hold", type=int, default=1200, help="静置观察多久（ms）")
    ap.add_argument("--param", action="append", default=[],
                    help="开始前在线改参数，可重复，例如 --param posDb=0.02")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    angles = [float(s) for s in args.angles.split(",") if s.strip()]

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    buf = bytearray()
    time.sleep(0.3)
    ser.reset_input_buffer()

    for kv in args.param:
        ser.write(("$%s!" % kv).encode())
        time.sleep(0.06)
        print("  设置 %s" % kv)

    ser.write(b"$wave=2!")          # 参考位置 / 实际位置，连续角度域
    time.sleep(0.15)

    print()
    print("  角度    帧数   均值(°)      峰峰(°)    标准差(°)   跟踪误差(°)  判定")
    print("  " + "-" * 74)

    rows = []
    worst = None
    for ang in angles:
        ser.reset_input_buffer()
        buf.clear()
        ser.write(("=%g!" % ang).encode())

        # 等它走完并稳定
        time.sleep(args.settle / 1000.0)

        # 清掉等待期间积压的数据，只留观察窗口
        ser.reset_input_buffer()
        buf.clear()
        frames = read_frames(ser, time.time() + args.hold / 1000.0, buf)
        s = stats(frames)
        if s is None:
            print("  %6.1f   无数据" % ang)
            continue

        v = verdict(s["ptp"])
        rows.append((ang, s, v))
        print("  %6.1f  %5d  %9.4f  %9.4f  %9.4f  %+10.4f  %s"
              % (ang, s["n"], s["mean"], s["ptp"], s["std"], s["err"], v))
        if worst is None or s["ptp"] > worst[1]["ptp"]:
            worst = (ang, s, v)

    print()
    if worst is not None:
        print("  最差位置：%.1f°  峰峰 %.4f°  跟踪误差 %+.4f°  →  %s"
              % (worst[0], worst[1]["ptp"], worst[1]["err"], worst[2]))
        bad = [r for r in rows if r[1]["ptp"] >= 0.06]
        print("  达到「微抖」及以上的位置：%s"
              % ("无" if not bad else ", ".join("%.0f°" % r[0] for r in bad)))
    ser.close()


if __name__ == "__main__":
    main()
