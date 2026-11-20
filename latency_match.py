#!/usr/bin/env python3
"""
latency_match.py — ghép xung TRIG TX (kênh D0, N4) với xung RX (kênh D1, gateway) để tính độ trễ
một chiều, trên CÙNG trục thời gian của PPK2.

Mã xung (khớp main_tn8_ppk2.c):
    TELEMETRY : TX = 1 xung (D0)  ->  RX = 3 xung (D1)
    ALARM     : TX = 4 xung (D0)  ->  RX = 5 xung (D1)
    (2 xung = relay chuyển tiếp, bỏ qua)

Đầu ra: <out>_alarm.csv và <out>_telem.csv theo định dạng log TN8
        (t_local_us,node_id,event,src,dst,seq,...) -> dùng thẳng với analyze_v2.py / tn8_auto.py.

Ví dụ:
    python latency_match.py --edges data/T1_edges.csv --out data/T1_lat
    python latency_match.py --edges data/T2_edges.csv --skew        # kiểm tra lệch kênh
    python latency_match.py --selftest
"""
import argparse, csv, os, sys, tempfile
import numpy as np

GROUP_GAP_US = 700       # sườn lên cách nhau <= giá trị này thuộc cùng một bộ xung (bước xung trong bộ = 400 us)
# lớp: (số xung TX, số xung RX, cửa sổ ghép mặc định ms)
CLASSES = {"alarm": (4, 5, 2000.0), "telem": (1, 3, 500.0)}


def load_rising(path):
    r = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row["edge"] == "R":
                r.setdefault(int(row["ch"]), []).append(float(row["t_us"]))
    return {k: np.array(sorted(v)) for k, v in r.items()}


def group_pulses(t):
    """Gộp sườn lên thành bộ xung. Trả về list (t_đầu_tiên_us, số_xung)."""
    out = []
    if len(t) == 0:
        return out
    start, n, last = t[0], 1, t[0]
    for x in t[1:]:
        if x - last <= GROUP_GAP_US:
            n += 1
        else:
            out.append((start, n))
            start, n = x, 1
        last = x
    out.append((start, n))
    return out


def match(tx, rx, window_us):
    """Ghép tuần tự: mỗi TX lấy RX đầu tiên (chưa dùng) trong (0, window]."""
    pairs, used, j = [], set(), 0
    for i, t in enumerate(tx):
        while j < len(rx) and rx[j] <= t:
            j += 1
        k = j
        while k < len(rx) and rx[k] - t <= window_us:
            if k not in used:
                used.add(k)
                pairs.append((i, t, rx[k]))
                break
            k += 1
    return pairs, used


def hist(g):
    return {n: sum(1 for _, c in g if c == n) for n in sorted({c for _, c in g})}


def write_csv(path, tx, pairs, cls, tx_src, rx_src):
    if os.path.dirname(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
    cname = "ALARM" if cls == "alarm" else "TELEMETRY"
    matched = {i: r_ for i, _, r_ in pairs}
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_local_us", "node_id", "event", "src", "dst", "seq", "ttl_tx", "ttl_rx",
                    "rssi", "class", "payload_len", "DROP_CACHE"])
        for i, t in enumerate(tx):
            w.writerow([f"{t:.0f}", tx_src, "TX", tx_src, rx_src, i, 0, 0, 0, cname, 5, 0])
            if i in matched:
                w.writerow([f"{matched[i]:.0f}", rx_src, "RX", tx_src, rx_src, i, 0, 0, 0, cname, 5, 0])


def analyze(edges, out, tx_src, rx_src, ch_tx, ch_rx, only=None, windows=None):
    r = load_rising(edges)
    gt, gr = group_pulses(r.get(ch_tx, [])), group_pulses(r.get(ch_rx, []))
    print(f"D{ch_tx} (TX): {len(gt)} bộ xung, phân bố số xung = {hist(gt)}")
    print(f"D{ch_rx} (RX): {len(gr)} bộ xung, phân bố số xung = {hist(gr)}")
    known_tx = {c[0] for c in CLASSES.values()} | {2}
    known_rx = {c[1] for c in CLASSES.values()} | {2}
    odd = [c for _, c in gt if c not in known_tx] + [c for _, c in gr if c not in known_rx]
    if odd:
        print(f"[!] {len(odd)} bộ xung có số xung lạ {sorted(set(odd))}: nghi nhiễu / dây lỏng / mất mẫu / dính xung.")
    res = {}
    for cls, (ntx, nrx, win) in CLASSES.items():
        if only and cls != only:
            continue
        win = (windows or {}).get(cls, win)
        tx = [t for t, c in gt if c == ntx]
        rx = [t for t, c in gr if c == nrx]
        if not tx:
            continue
        if len(tx) > 1 and np.min(np.diff(tx)) < win * 1000:
            print(f"[!] {cls}: có hai TX cách nhau < cửa sổ ghép {win:.0f} ms, có thể ghép nhầm.")
        pairs, used = match(tx, rx, win * 1000.0)
        lat = np.array([(r_ - t) / 1000.0 for _, t, r_ in pairs])
        print(f"\n[{cls.upper()}] TX={len(tx)} | ghép được={len(pairs)} | RX mồ côi={len(rx)-len(used)} "
              f"| PDR={100.0*len(pairs)/len(tx):.1f} %  (cửa sổ {win:.0f} ms)")
        if lat.size:
            print(f"   độ trễ ms: min {lat.min():.2f} | median {np.median(lat):.2f} | mean {lat.mean():.2f} "
                  f"| p95 {np.percentile(lat,95):.2f} | max {lat.max():.2f}")
        matched_idx = {i for i, _, _ in pairs}
        missed_t = [f"{t/1e6:.2f}s" for i, t in enumerate(tx) if i not in matched_idx]
        if missed_t:
            print(f"   [-] Thời điểm TX không nhận được RX ({len(missed_t)} gói): {', '.join(missed_t)}")
        if len(rx) - len(used):
            print("   [!] RX mồ côi: xung RX không có TX tương ứng (nhiễu, hoặc cửa sổ quá hẹp).")
        if out:
            p = f"{out}_{cls}.csv"
            write_csv(p, tx, pairs, cls, tx_src, rx_src)
            print("   ->", p)
        res[cls] = lat
    return res


def skew(edges, ch_a, ch_b):
    """Bước 2: nối cùng một nguồn xung vào hai kênh -> hiệu thời gian phải ~0."""
    r = load_rising(edges)
    a, b = r.get(ch_a, np.array([])), r.get(ch_b, np.array([]))
    if len(a) == 0 or len(b) == 0:
        sys.exit("Thiếu sườn ở một trong hai kênh.")
    d = np.array([b[np.argmin(np.abs(b - t))] - t for t in a])
    print(f"Lệch D{ch_b}-D{ch_a}: median {np.median(d):.0f} us, min {d.min():.0f}, max {d.max():.0f} "
          f"(n={len(d)}). Đạt nếu |lệch| <= 10 us (1 mẫu).")


def burst(w, t, n):
    return [(t + 400 * k, w) for k in range(n)]


def selftest():
    rng = np.random.default_rng(1)
    rows, true_a, true_t = [], [], []
    t = 1e6
    for k in range(60):                                   # ALARM: TX 4 xung, RX 5 xung
        t += rng.uniform(5e6, 15e6)
        lat = rng.uniform(8e3, 60e3)
        rows += burst(0, t, 4)
        if k % 10 != 3:                                    # mất 6 gói
            rows += burst(1, t + lat, 5)
            true_a.append(lat / 1000)
    for k in range(300):                                  # TELEMETRY mỗi giây
        t0 = 1e6 + k * 1e6 + 123
        lat = rng.uniform(5e3, 40e3)
        rows += burst(0, t0, 1)
        clash = any(abs((t0 + lat) - r_t) < 5000 for r_t, c in rows if c == 1)
        if k % 25 != 0 and not clash:
            rows += burst(1, t0 + lat, 3)
            true_t.append(lat / 1000)
    rows.sort()
    tmp_dir = tempfile.gettempdir()
    p = os.path.join(tmp_dir, "_selftest_edges.csv")
    out_prefix = os.path.join(tmp_dir, "_selftest_lat")
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_us", "ch", "edge"])
        for c, t_ in [(c, t_) for t_, c in rows]:
            w.writerow([f"{round(t_/10)*10:.0f}", c, "R"])
    res = analyze(p, out_prefix, "0x0004", "0x0001", 0, 1)
    ok = True
    for cls, true in (("alarm", true_a), ("telem", true_t)):
        lat = res.get(cls, np.array([]))
        good = len(lat) == len(true) and np.max(np.abs(lat - np.array(true))) < 0.02
        ok &= good
        print(f"SELFTEST {cls}:", "PASS" if good else f"FAIL (ghép {len(lat)}/{len(true)})")
    print("SELFTEST", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--edges")
    ap.add_argument("--out", help="tiền tố file CSV đầu ra")
    ap.add_argument("--only", choices=list(CLASSES), help="chỉ xử lý một lớp")
    ap.add_argument("--window-alarm", type=float)
    ap.add_argument("--window-telem", type=float)
    ap.add_argument("--tx-src", default="0x0004")
    ap.add_argument("--rx-src", default="0x0001")
    ap.add_argument("--ch-tx", type=int, default=0)
    ap.add_argument("--ch-rx", type=int, default=1)
    ap.add_argument("--skew", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    elif a.skew:
        skew(a.edges, a.ch_tx, a.ch_rx)
    else:
        if not a.edges:
            sys.exit("Cần --edges")
        wins = {k: v for k, v in (("alarm", a.window_alarm), ("telem", a.window_telem)) if v}
        analyze(a.edges, a.out, a.tx_src, a.rx_src, a.ch_tx, a.ch_rx, a.only, wins)