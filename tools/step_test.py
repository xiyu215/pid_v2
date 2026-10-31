#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_v2 阶跃响应测量工具
--------------------------------------------------------------------------
用法：
    python step_test.py --port COM7 --angle 90
    python step_test.py --port COM7 --angle 90 --param profA=30000 --param posKd=0.3
    python step_test.py --port COM7 --angle 0 --ms 1200 --csv run1.csv

做的事：
    1. 依次下发 $name=value! 参数（在线整定，不用重新烧录）
    2. 切到 wave=2 通道组（参考位置 / 实际位置，连续角度域，无跨圈跳变）
    3. 下发 =angle! 触发阶跃，同时开始录 500Hz 的 JustFloat 数据
    4. 算上升时间 / 超调量 / 调节时间 / 稳态误差，可选存 CSV

串口参数固定 115200 8N1，与固件一致。VOFA+ 可以同时开着看波形，
但不要让它往串口发数据，否则会干扰命令帧。
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial：python -m pip install pyserial")

# JustFloat 帧尾：float(+Inf) = 0x7F800000 的小端写法
TAIL = b"\x00\x00\x80\x7f"
CH = 2                      # 固件固定发 2 个通道
FRAME = CH * 4 + 4


def read_frames(ser, deadline, buf):
    """从串口流里切出完整的 JustFloat 帧，返回 [(c0, c1), ...]

    时间轴不用读循环打的时间戳：串口一次 read 会带回来好几帧，那些帧会被
    打上同一个时刻，抖动可达十几毫秒，而整个阶跃才两三百毫秒。
    固件按固定 2ms 定拍发送，实测 500.0 帧/秒且不丢帧，
    所以直接用帧序号 × 采样周期做时间轴，精度就是一个采样点。
    """
    out = []
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(TAIL)
            if i < 0:
                # 没有帧尾，只保留可能是半帧的尾巴，其余丢掉防止无限增长
                if len(buf) > 4096:
                    del buf[:-FRAME]
                break
            if i < CH * 4:
                del buf[: i + 4]
                continue
            body = buf[i - CH * 4 : i]
            out.append(struct.unpack("<ff", body))
            del buf[: i + 4]
    return out


def metrics(t, y, target, t_step):
    """按阶跃响应算四个指标。y 是实际角度序列，t 是秒。"""
    if not y:
        return None

    y0 = y[0]
    span = target - y0
    if abs(span) < 0.5:
        return None
    sign = 1.0 if span > 0 else -1.0

    def pct(p):                     # 相对起点的百分比位置
        return y0 + span * p

    # 上升时间：10% -> 90%
    t10 = t90 = None
    for ti, yi in zip(t, y):
        if t10 is None and sign * (yi - pct(0.10)) >= 0:
            t10 = ti
        if t10 is not None and t90 is None and sign * (yi - pct(0.90)) >= 0:
            t90 = ti
            break
    rise = (t90 - t10) * 1000.0 if (t10 is not None and t90 is not None) else float("nan")

    # 超调：越过目标的最大幅度，占阶跃量的百分比
    over = max((sign * (yi - target)) for yi in y)
    overshoot = max(0.0, over / abs(span) * 100.0)

    # 调节时间：进入 ±0.1° 且此后不再出去
    band = 0.1
    settle = float("nan")
    for k in range(len(y) - 1, -1, -1):
        if abs(y[k] - target) > band:
            if k + 1 < len(y):
                settle = (t[k + 1] - t_step) * 1000.0
            break
    else:
        settle = (t[0] - t_step) * 1000.0

    # 稳态误差：最后 100ms 的均值
    tail = [(ti, yi) for ti, yi in zip(t, y) if ti >= t[-1] - 0.1]
    sse = (sum(yi for _, yi in tail) / len(tail) - target) if tail else float("nan")

    return dict(rise=rise, overshoot=overshoot, settle=settle, sse=sse,
                peak=max(y, key=lambda v: sign * v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口号，例如 COM7")
    ap.add_argument("--angle", type=float, required=True, help="目标角度（度，0~359）")
    ap.add_argument("--ms", type=int, default=800, help="录多久，默认 800ms")
    ap.add_argument("--param", action="append", default=[],
                    help="在线改参数，可重复，例如 --param profA=30000")
    ap.add_argument("--csv", help="把原始波形存成 CSV")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    buf = bytearray()
    time.sleep(0.3)
    ser.reset_input_buffer()

    # ---- 1. 下发参数 ----
    for kv in args.param:
        frame = ("$%s!" % kv).encode()
        ser.write(frame)
        time.sleep(0.05)
        print("  设置 %s" % kv)

    # ---- 2. 切到 wave=2（参考 / 实际，连续角度域）----
    ser.write(b"$wave=2!")
    time.sleep(0.15)

    # ---- 3. 先抓一段静止数据，确认链路通、电机没在动 ----
    ser.reset_input_buffer()
    buf.clear()
    quiet = read_frames(ser, time.time() + 0.25, buf)
    if len(quiet) < 20:
        sys.exit("只收到 %d 帧波形，检查串口是否接对、固件是否在跑" % len(quiet))
    y_now = quiet[-1][1]
    print("  当前位置 %.2f°，目标 %.2f°" % (y_now, args.angle))

    # ---- 4. 触发阶跃并录数据 ----
    ser.reset_input_buffer()
    buf.clear()
    ser.write(("=%g!" % args.angle).encode())

    data = read_frames(ser, time.time() + args.ms / 1000.0, buf)
    if len(data) < 50:
        sys.exit("只收到 %d 帧，录数据失败" % len(data))

    # 时间轴：帧序号 × 2ms。第 0 帧就在命令发出后第一拍，即 t=0
    dt = 0.002
    t = [k * dt for k in range(len(data))]
    ref = [d[0] for d in data]
    y = [d[1] for d in data]
    t_step = 0.0

    m = metrics(t, y, args.angle, t_step)
    if m is None:
        sys.exit("阶跃量太小或数据异常，无法评估")

    print()
    print("  上升时间 (10-90%%) : %7.1f ms" % m["rise"])
    print("  超调量            : %7.2f %%" % m["overshoot"])
    print("  调节时间 (±0.1°)  : %7.1f ms" % m["settle"])
    print("  稳态误差          : %+7.3f °" % m["sse"])
    print("  峰值              : %7.2f °" % m["peak"])
    print("  采样帧数          : %d" % len(data))

    if args.csv:
        with open(args.csv, "w", encoding="utf-8") as f:
            f.write("t_ms,ref_deg,act_deg\n")
            for ti, r, a in zip(t, ref, y):
                f.write("%.1f,%.4f,%.4f\n" % (ti * 1000.0, r, a))
        print("  原始波形已存 %s" % args.csv)

    ser.close()


if __name__ == "__main__":
    main()
