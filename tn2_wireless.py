#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tn2_wireless_sync.py — Can chinh 2 diem (offset tuong doi so voi N4) cho TN2 khong day.
- Doc truc tiep dinh dang PPK2 cua ppk2_edges.py (t_us,ch,edge), loc edge == 'R'.
- Triet tieu hoan toan goc thoi gian rieng cua 2 file PPK2 (cal_pre va cal_post).
- Bat buoc N4 (D0) du DUNG cal_count (mac dinh 20) xung/goi; cac node Relay/GW ghep theo
  cua so thoi gian + net_seq16, chap nhan mat vai goi CAL mien la ghep du >= min_match (15).
- Tuong thich CSV 12 cot cua main_tn2.c (11 cot dau giu nguyen, net_seq16 o cot cuoi).

Vi du:
    python tn2_wireless_sync.py --selftest
    python tn2_wireless_sync.py --pre-ppk2 data/cal_pre.csv --post-ppk2 data/cal_post.csv \
        --nodes data/N4.csv data/N3.csv data/N2.csv data/N1.csv data/GW.csv \
        --hops 4 --out results/EXP2_4hop_run1.csv
"""

import argparse
import csv
import os
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple
import numpy as np

CAL_SEQ_BASE = 30000
WRAP_U32_US = 2**32
GROUP_GAP_US = 700.0  # Khop latency_match.py (buoc xung trong nhom = 400 us)


def unwrap_u32_us(vals: List[int]) -> np.ndarray:
    """Chong tran 32-bit (~71.5 phut) cho cot t_local_us."""
    out = []
    offset = 0
    prev = None
    for v in vals:
        if prev is not None and v < prev and (prev - v) > (WRAP_U32_US // 2):
            offset += WRAP_U32_US
        out.append(v + offset)
        prev = v
    return np.array(out, dtype=np.float64)


def load_node_csv(path: str) -> List[Dict]:
    """Doc log CSV cua node theo ten cot (bo qua dong # CFG / # TRIG / ---)."""
    header = None
    raw_rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#") or s.startswith("---"):
                continue
            if s.startswith("t_local_us,"):
                header = [c.strip() for c in s.split(",")]
                continue
            if header is None:
                continue
            parts = [c.strip() for c in s.split(",")]
            if len(parts) < 11:
                continue
            row = dict(zip(header, parts))
            try:
                raw_rows.append({
                    "t_raw_us": int(row["t_local_us"]),
                    "node_id": row["node_id"],
                    "event": row["event"],
                    "src": row["src"],
                    "dst": row["dst"],
                    "seq": int(row["seq"]),
                    "ttl_tx": int(row["ttl_tx"]),
                    "ttl_rx": int(row["ttl_rx"]),
                    "rssi": int(row["rssi"]),
                    "cls": row["class"],
                    "net_seq16": int(row["net_seq16"]) if "net_seq16" in row else -1,
                })
            except (ValueError, KeyError):
                continue

    t_unwrapped = unwrap_u32_us([r["t_raw_us"] for r in raw_rows])
    for r, tu in zip(raw_rows, t_unwrapped):
        r["t_us"] = float(tu)
    return raw_rows


def load_ppk2_bursts(path: str, channels: List[int]) -> Dict[int, List[Tuple[float, int]]]:
    """
    Doc file canh PPK2 do ppk2_edges.py xuat (t_us,ch,edge).
    Chi lay suon len (edge == 'R'), sap xep theo thoi gian, gom nhom theo GROUP_GAP_US.
    """
    rising: Dict[int, List[float]] = {c: [] for c in channels}
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        fields = [c.strip() for c in (reader.fieldnames or [])]
        has_edge = "edge" in fields
        for row in reader:
            if has_edge and row["edge"].strip().upper() != "R":
                continue
            ch_raw = row.get("ch", row.get("channel", "")).strip()
            if not ch_raw:
                continue
            ch = int(ch_raw.replace("D", ""))
            if ch not in rising:
                continue
            t_us = float(row["t_us"]) if "t_us" in fields else float(row["t_s"]) * 1e6
            rising[ch].append(t_us)

    grouped: Dict[int, List[Tuple[float, int]]] = {}
    for ch, ts in rising.items():
        ts.sort()
        evts = []
        if ts:
            start, cnt, last = ts[0], 1, ts[0]
            for x in ts[1:]:
                if x - last <= GROUP_GAP_US:
                    cnt += 1
                else:
                    evts.append((start, cnt))
                    start, cnt = x, 1
                last = x
            evts.append((start, cnt))
        grouped[ch] = evts
    return grouped


def compute_relative_offset(
    burst_name: str,
    ch_id: int,
    node_name: str,
    tx_cal_rows: List[Dict],
    node_cal_map: Dict[int, float],
    tx_ppk2_us: np.ndarray,
    node_ppk2_us: np.ndarray,
    cal_count: int,
    min_match: int,
    max_lat_us: float = 500_000.0,
) -> Tuple[float, float, float, int]:
    """
    Tinh offset tuong doi cua node k so voi N4 trong CUNG 1 lan thu PPK2:
        o_k(i) = (t_local_k(i) - t_local_N4(i)) - (t_ppk2_k(i) - t_ppk2_N4(i))
    """
    if len(tx_cal_rows) != cal_count or len(tx_ppk2_us) != cal_count:
        raise SystemExit(
            f"[LOI {burst_name}] N4 KHONG DU {cal_count} mau! "
            f"(N4_log={len(tx_cal_rows)}, N4_ppk2={len(tx_ppk2_us)}). Hay cham moc lai burst nay!"
        )

    offsets = []
    t_n4_matched = []
    used_rx = set()
    j = 0
    for i, r_tx in enumerate(tx_cal_rows):
        ns = r_tx["net_seq16"]
        if ns not in node_cal_map:
            continue
        t_ppk_tx = tx_ppk2_us[i]
        while j < len(node_ppk2_us) and node_ppk2_us[j] <= t_ppk_tx - 500.0:
            j += 1
        k_idx = j
        matched_ppk_rx = None
        while k_idx < len(node_ppk2_us) and (node_ppk2_us[k_idx] - t_ppk_tx) <= max_lat_us:
            if k_idx not in used_rx:
                used_rx.add(k_idx)
                matched_ppk_rx = node_ppk2_us[k_idx]
                break
            k_idx += 1
        if matched_ppk_rx is None:
            continue

        ppk_lat = matched_ppk_rx - t_ppk_tx
        local_diff = node_cal_map[ns] - r_tx["t_us"]
        offsets.append(local_diff - ppk_lat)
        t_n4_matched.append(r_tx["t_us"])

    if len(offsets) < min_match:
        raise SystemExit(
            f"[LOI {burst_name}] Node D{ch_id} ({node_name}): chi ghep duoc {len(offsets)}/{cal_count} "
            f"goi CAL (< nguong {min_match}). Kiem tra ket noi day TRIG/PPK2 hoac lam lai burst!"
        )

    arr = np.array(offsets, dtype=np.float64)
    med = float(np.median(arr))
    inliers = arr[np.abs(arr - med) <= 2000.0]
    if len(inliers) < min_match:
        raise SystemExit(
            f"[LOI {burst_name}] Node D{ch_id} ({node_name}): qua nhieu mau lech > 2 ms "
            f"(chi con {len(inliers)}/{len(arr)} mau sach)."
        )
    med = float(np.median(inliers))
    std_us = float(np.std(inliers - med, ddof=1)) if len(inliers) > 1 else 0.0
    if std_us > 1000.0:
        raise SystemExit(
            f"[LOI {burst_name}] Node D{ch_id} ({node_name}): sai so noi bo burst qua lon "
            f"(sd={std_us/1000:.3f} ms > 1.0 ms)."
        )
    return float(np.median(t_n4_matched)), med, std_us, len(inliers)


def run_sync(pre_ppk2, post_ppk2, node_paths, hops, channels, cal_count, min_match, ttl_tx, out_path):
    n_nodes = hops + 1
    if len(node_paths) != n_nodes or len(channels) != n_nodes:
        raise SystemExit(f"Voi {hops} chang can truyen dung {n_nodes} file CSV va {n_nodes} kenh PPK2.")

    pre_bursts = load_ppk2_bursts(pre_ppk2, channels)
    post_bursts = load_ppk2_bursts(post_ppk2, channels)
    node_logs = [load_node_csv(p) for p in node_paths]

    exp_pulses = [1] + [2] * (hops - 1) + [3]
    pre_edges = []
    post_edges = []
    for ch, ep in zip(channels, exp_pulses):
        pe = np.array([t for t, c in pre_bursts[ch] if c == ep], dtype=np.float64)
        qe = np.array([t for t, c in post_bursts[ch] if c == ep], dtype=np.float64)
        odd_pre = [c for _, c in pre_bursts[ch] if c != ep]
        odd_post = [c for _, c in post_bursts[ch] if c != ep]
        if odd_pre or odd_post:
            print(f"[!] Canh bao kenh D{ch}: co nhom xung la (pre={odd_pre[:5]}, post={odd_post[:5]}, ky vong {ep} xung).")
        pre_edges.append(pe)
        post_edges.append(qe)

    tx_rows = node_logs[0]
    cal_rows_tx = [r for r in tx_rows if r["cls"] == "CAL" or r["seq"] >= CAL_SEQ_BASE]
    main_rows_tx = [r for r in tx_rows if r["cls"] != "CAL" and 0 < r["seq"] < CAL_SEQ_BASE]

    if not main_rows_tx:
        raise SystemExit("[LOI] Khong tim thay goi MAIN trong log cua N4!")

    t_main_start = main_rows_tx[0]["t_us"]
    t_main_end = main_rows_tx[-1]["t_us"]
    cal_pre_tx = [r for r in cal_rows_tx if r["t_us"] < t_main_start]
    cal_post_tx = [r for r in cal_rows_tx if r["t_us"] > t_main_end]

    if len(cal_pre_tx) != cal_count or len(cal_post_tx) != cal_count:
        raise SystemExit(
            f"[LOI] So goi CAL trong log N4 khong dung {cal_count}! "
            f"(CAL_PRE={len(cal_pre_tx)}, CAL_POST={len(cal_post_tx)})"
        )

    pre_set = {r["net_seq16"] for r in cal_pre_tx}
    post_set = {r["net_seq16"] for r in cal_post_tx}
    main_netseq_to_seq = {r["net_seq16"]: r["seq"] for r in main_rows_tx}

    print(f"=== HIEU CHUAN 2 DIEM TUONG DOI SO VOI N4 ({hops} CHANG | {len(main_rows_tx)} GOI CHINH) ===")
    print(f"N4 (D{channels[0]}): CAL_PRE={len(cal_pre_tx)}/{cal_count}, CAL_POST={len(cal_post_tx)}/{cal_count}\n")

    rel_models: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

    for k in range(1, n_nodes):
        name = Path(node_paths[k]).name
        pre_map = {}
        post_map = {}
        for r in node_logs[k]:
            ns = r["net_seq16"]
            if ns in pre_set and ns not in pre_map:
                pre_map[ns] = r["t_us"]
            if ns in post_set:
                post_map[ns] = r["t_us"]

        t0_n4, o0, std0, n0 = compute_relative_offset(
            "CAL_PRE", channels[k], name, cal_pre_tx, pre_map, pre_edges[0], pre_edges[k], cal_count, min_match
        )
        t1_n4, o1, std1, n1 = compute_relative_offset(
            "CAL_POST", channels[k], name, cal_post_tx, post_map, post_edges[0], post_edges[k], cal_count, min_match
        )

        dt = t1_n4 - t0_n4
        if dt <= 0:
            raise SystemExit(f"[LOI] Khoang thoi gian giua CAL_PRE va CAL_POST khong duong (dt={dt} us)!")

        slope = (o1 - o0) / dt
        drift_ppm = slope * 1e6
        warn = " [CANH BAO: TROI TUONG DOI > 80 ppm — KIEM TRA LFCLK XTAL!]" if abs(drift_ppm) > 80.0 else ""
        print(
            f"Node D{channels[k]} ({name}): PRE {n0}/{cal_count} (sd={std0/1000:.3f}ms) | "
            f"POST {n1}/{cal_count} (sd={std1/1000:.3f}ms) | "
            f"rel_drift={drift_ppm:+.2f} ppm (tong troi={(o1 - o0)/1000:+.2f} ms){warn}"
        )
        rel_models.append((t0_n4, o0, slope))

    def to_n4_clock_us(node_idx: int, t_local_k_us: float, t_n4_ref_us: float) -> float:
        if node_idx == 0:
            return t_local_k_us
        t0_n4, o0, slope = rel_models[node_idx]
        return t_local_k_us - (o0 + slope * (t_n4_ref_us - t0_n4))

    node_main_raw = []
    for k in range(n_nodes):
        m = {}
        for r in node_logs[k]:
            ns = r["net_seq16"]
            if ns in main_netseq_to_seq and ns not in m:
                m[ns] = (r["t_us"], r["ttl_rx"], r["rssi"])
        node_main_raw.append(m)

    e2e_ms = []
    hop_ms = [[] for _ in range(hops)]
    exp_ttl_rx = ttl_tx - (hops - 1)
    ttl_match = 0
    rx_total = 0

    if out_path:
        Path(out_path).parent.mkdir(parents=True, exist_ok=True)
        fout = open(out_path, "w", newline="", encoding="utf-8")
        w = csv.writer(fout)
        w.writerow(["seq", "net_seq16", "e2e_ms"] + [f"hop{h+1}_ms" for h in range(hops)] + ["gw_ttl_rx", "gw_rssi"])
    else:
        fout, w = None, None

    try:
        for r in main_rows_tx:
            ns = r["net_seq16"]
            seq = r["seq"]
            if ns not in node_main_raw[-1]:
                continue

            rx_total += 1
            t_n4 = r["t_us"]
            _, gw_ttl, gw_rssi = node_main_raw[-1][ns]
            if gw_ttl == exp_ttl_rx:
                ttl_match += 1

            t_nodes = [t_n4]
            for k in range(1, n_nodes):
                if ns in node_main_raw[k]:
                    t_nodes.append(to_n4_clock_us(k, node_main_raw[k][ns][0], t_n4))
                else:
                    t_nodes.append(None)

            lat_e2e = (t_nodes[-1] - t_nodes[0]) / 1000.0
            e2e_ms.append(lat_e2e)

            per_hop = []
            for h in range(hops):
                if t_nodes[h] is not None and t_nodes[h + 1] is not None:
                    dh = (t_nodes[h + 1] - t_nodes[h]) / 1000.0
                    hop_ms[h].append(dh)
                    per_hop.append(f"{dh:.3f}")
                else:
                    per_hop.append("")
            if w:
                w.writerow([seq, ns, f"{lat_e2e:.3f}"] + per_hop + [gw_ttl, gw_rssi])
    finally:
        if fout:
            fout.close()

    pdr = 100.0 * rx_total / len(main_rows_tx)
    ttl_pct = 100.0 * ttl_match / max(1, rx_total)
    e2e_arr = np.array(e2e_ms, dtype=np.float64)

    print("\n================ KET QUA TN2 (HIEU CHUAN TUONG DOI SO VOI N4) ================")
    print(f"PDR       : {rx_total}/{len(main_rows_tx)} = {pdr:.2f} %")
    print(f"TTL check : {ttl_match}/{rx_total} = {ttl_pct:.2f} % (du kien ttl_rx={exp_ttl_rx})")
    if len(e2e_arr):
        p95 = float(np.percentile(e2e_arr, 95))
        mean = float(np.mean(e2e_arr))
        sd = float(np.std(e2e_arr, ddof=1)) if len(e2e_arr) > 1 else 0.0
        print(f"E2E       : mean={mean:.2f} ms | sd={sd:.2f} ms | p95={p95:.2f} ms | p95/mean={p95/mean:.2f}")
    for h in range(hops):
        arr = np.array(hop_ms[h], dtype=np.float64)
        if len(arr):
            print(
                f"  Hop {h+1}   : mean={np.mean(arr):.2f} ms | sd={np.std(arr, ddof=1):.2f} ms | "
                f"p95={np.percentile(arr, 95):.2f} ms (n={len(arr)})"
            )
    if out_path:
        print(f"Da luu bang chi tiet: {out_path}")
    return e2e_arr, hop_ms


def selftest():
    rng = np.random.default_rng(42)
    tmp = tempfile.mkdtemp()
    offs = [0.0, 40_000_000.0, -12_000_000.0]
    drifts = [0.0, 35e-6, -25e-6]
    cal_n, main_n = 20, 100

    node_rows = [[], [], []]
    pre_ppk, post_ppk = [], []
    true_e2e, true_h1, true_h2 = [], [], []

    def add_ppk_burst(lst, t_us, ch, n_pulses):
        for p in range(n_pulses):
            lst.append((t_us + p * 400.0, ch, "R"))
            lst.append((t_us + p * 400.0 + 200.0, ch, "F"))

    def local_t(k, t_real):
        return (t_real * (1.0 + drifts[k]) + offs[k]) % WRAP_U32_US

    net_seq = 500
    for i in range(cal_n):
        t_real = 10_000_000.0 + i * 1_000_000.0
        h1 = rng.uniform(18_000.0, 26_000.0)
        h2 = rng.uniform(20_000.0, 28_000.0)
        ns = net_seq; net_seq += 1
        app_seq = CAL_SEQ_BASE + 1 + i
        add_ppk_burst(pre_ppk, t_real - 5_000_000.0, 0, 1)
        add_ppk_burst(pre_ppk, t_real + h1 - 5_000_000.0, 1, 2)
        add_ppk_burst(pre_ppk, t_real + h1 + h2 - 5_000_000.0, 2, 3)
        node_rows[0].append([int(local_t(0, t_real)), "0x0004", "TX", "0x0004", "0xC000", app_seq, 4, 0, 0, "CAL", 5, ns])
        node_rows[1].append([int(local_t(1, t_real + h1)), "0x0003", "RELAY", "0x0004", "0xC000", 0, 3, 4, -65, "RELAY", 5, ns])
        node_rows[2].append([int(local_t(2, t_real + h1 + h2)), "0x0001", "RX", "0x0004", "0x0001", app_seq, 0, 3, -68, "CAL", 5, ns])

    for i in range(main_n):
        t_real = 120_000_000.0 + i * 15_000_000.0
        h1 = rng.uniform(18_000.0, 35_000.0)
        h2 = rng.uniform(20_000.0, 40_000.0)
        ns = net_seq; net_seq += 1
        node_rows[0].append([int(local_t(0, t_real)), "0x0004", "TX", "0x0004", "0xC000", i + 1, 4, 0, 0, "TELEMETRY", 5, ns])
        node_rows[1].append([int(local_t(1, t_real + h1)), "0x0003", "RELAY", "0x0004", "0xC000", 0, 3, 4, -70, "RELAY", 5, ns])
        node_rows[2].append([int(local_t(2, t_real + h1 + h2)), "0x0001", "RX", "0x0004", "0x0001", i + 1, 0, 3, -72, "TELEMETRY", 5, ns])
        true_h1.append(h1 / 1000.0); true_h2.append(h2 / 1000.0); true_e2e.append((h1 + h2) / 1000.0)

    for i in range(cal_n):
        t_real = 1_660_000_000.0 + i * 1_000_000.0
        h1 = rng.uniform(18_000.0, 26_000.0)
        h2 = rng.uniform(20_000.0, 28_000.0)
        ns = net_seq; net_seq += 1
        app_seq = CAL_SEQ_BASE + 1 + cal_n + i
        add_ppk_burst(post_ppk, t_real - 1_650_000_000.0, 0, 1)
        add_ppk_burst(post_ppk, t_real + h1 - 1_650_000_000.0, 1, 2)
        node_rows[0].append([int(local_t(0, t_real)), "0x0004", "TX", "0x0004", "0xC000", app_seq, 4, 0, 0, "CAL", 5, ns])
        node_rows[1].append([int(local_t(1, t_real + h1)), "0x0003", "RELAY", "0x0004", "0xC000", 0, 3, 4, -65, "RELAY", 5, ns])
        if i not in (3, 14):
            add_ppk_burst(post_ppk, t_real + h1 + h2 - 1_650_000_000.0, 2, 3)
            node_rows[2].append([int(local_t(2, t_real + h1 + h2)), "0x0001", "RX", "0x0004", "0x0001", app_seq, 0, 3, -68, "CAL", 5, ns])

    paths = []
    for k, fn in enumerate(["N4.csv", "N3.csv", "GW.csv"]):
        p = os.path.join(tmp, fn)
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_local_us", "node_id", "event", "src", "dst", "seq", "ttl_tx", "ttl_rx", "rssi", "class", "payload_len", "net_seq16"])
            w.writerows(node_rows[k])
        paths.append(p)
    p_pre = os.path.join(tmp, "pre.csv")
    p_post = os.path.join(tmp, "post.csv")
    for p, data in [(p_pre, pre_ppk), (p_post, post_ppk)]:
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_us", "ch", "edge"])
            for t_, c_, e_ in data:
                w.writerow([f"{round(t_/10)*10:.0f}", c_, e_])

    e2e, hops = run_sync(p_pre, p_post, paths, 2, [0, 1, 2], cal_n, 15, 4, None)
    err_e2e = float(np.max(np.abs(e2e - np.array(true_e2e))))
    err_h1 = float(np.max(np.abs(np.array(hops[0]) - np.array(true_h1))))
    err_h2 = float(np.max(np.abs(np.array(hops[1]) - np.array(true_h2))))
    print(f"\n[SELFTEST] Max error: E2E={err_e2e:.4f} ms | Hop1={err_h1:.4f} ms | Hop2={err_h2:.4f} ms")
    assert err_e2e < 0.05 and err_h1 < 0.05 and err_h2 < 0.05
    print("[SELFTEST] PASS!")


def main():
    ap = argparse.ArgumentParser(description="Hieu chuan 2 diem tuong doi so voi N4 cho TN2 khong day")
    ap.add_argument("--pre-ppk2", help="File canh PPK2 luc cham moc DAU buoi (t_us,ch,edge)")
    ap.add_argument("--post-ppk2", help="File canh PPK2 luc cham moc CUOI buoi (t_us,ch,edge)")
    ap.add_argument("--nodes", nargs="+", help="Danh sach CSV theo thu tu chuoi, VD: N4.csv N3.csv GW.csv")
    ap.add_argument("--channels", default=None, help="Cac kenh PPK2 tuong ung, mac dinh 0,1,...,hops")
    ap.add_argument("--hops", type=int, choices=[1, 2, 3, 4])
    ap.add_argument("--cal-count", type=int, default=20, help="So goi CAL bat buoc o N4 (mac dinh 20)")
    ap.add_argument("--min-match", type=int, default=15, help="So goi CAL toi thieu ghep duoc o Relay/GW (mac dinh 15)")
    ap.add_argument("--ttl-tx", type=int, default=4)
    ap.add_argument("--out", default="results/tn2_wireless_synced.csv")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    if not (args.pre_ppk2 and args.post_ppk2 and args.nodes and args.hops):
        ap.error("Can truyen du --pre-ppk2, --post-ppk2, --nodes va --hops (hoac dung --selftest).")

    chans = [int(c) for c in args.channels.split(",")] if args.channels else list(range(args.hops + 1))
    run_sync(args.pre_ppk2, args.post_ppk2, args.nodes, args.hops, chans, args.cal_count, args.min_match, args.ttl_tx, args.out)


if __name__ == "__main__":
    main()