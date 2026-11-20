#!/usr/bin/env python3
"""
ppk2_edges.py — thu sườn xung TRIG (D0, D1...) và dòng điện từ PPK2, ghi ra CSV nhỏ.

Chỉ ghi sườn (vài trăm dòng/run) thay vì 120 triệu mẫu. Trục thời gian = chỉ số mẫu / 100 kSa/s.
Ví dụ:
    python ppk2_edges.py --out data/T1_edges.csv --duration 300 --mode ampere
    python ppk2_edges.py --out data/EXP8_paarc_open_run1_edges.csv --duration 1260 \
                         --mode source --mv 3300 --current data/EXP8_paarc_open_run1_current.csv
Phụ thuộc: pip install ppk2-api numpy
"""
import argparse, csv, os, sys, time
import numpy as np
from ppk2_api.ppk2_api import PPK2_API

FS = 100_000          # PPK2 lấy mẫu 100 kSa/s -> 10 us / mẫu
US_PER_SAMPLE = 1e6 / FS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="Cổng COM của PPK2 (mặc định: tự tìm)")
    ap.add_argument("--mode", choices=["ampere", "source"], default="ampere",
                    help="ampere: chỉ đo; source: PPK2 cấp nguồn cho DUT (N3)")
    ap.add_argument("--mv", type=int, default=3300, help="Điện áp nguồn/tham chiếu (mV), mặc định 3300")
    ap.add_argument("--channels", default="0,1", help="Các kênh logic cần theo dõi, vd 0,1")
    ap.add_argument("--duration", type=float, required=True, help="Thời lượng thu (s)")
    ap.add_argument("--out", required=True, help="CSV sườn: t_us,ch,edge")
    ap.add_argument("--current", default=None, help="CSV dòng TB theo giây (tuỳ chọn)")
    ap.add_argument("--monitor", action="store_true", help="In số xung từng kênh mỗi giây")
    a = ap.parse_args()
    chans = [int(c) for c in a.channels.split(",")]

    # Tự động tạo thư mục chứa file đầu ra nếu chưa có
    if os.path.dirname(a.out):
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
    if a.current and os.path.dirname(a.current):
        os.makedirs(os.path.dirname(a.current), exist_ok=True)

    port = a.port
    if port is None:
        devs = PPK2_API.list_devices()
        if not devs:
            sys.exit("Không thấy PPK2. Kiểm tra cáp USB / cổng.")
        port = devs[0]
        if isinstance(port, (tuple, list)):
            port = port[0]

    ppk = PPK2_API(port)
    ppk.get_modifiers()
    if a.mode == "source":
        ppk.use_source_meter()
        ppk.set_source_voltage(a.mv)
        ppk.toggle_DUT_power("ON")
    else:
        ppk.set_source_voltage(a.mv)  # Thiết lập điện áp tham chiếu cho thư viện ppk2_api
        ppk.use_ampere_meter()

    ppk.start_measuring()
    time.sleep(0.2)
    d = ppk.get_data()
    if d:
        ppk.get_samples(d)   # bỏ kết quả nhưng giữ đồng bộ khung 4 byte

    prev = {c: None for c in chans}
    n_total = 0
    cnt = {c: 0 for c in chans}
    cur_sum, cur_n, cur_rows = 0.0, 0, []
    t0 = time.time()
    last_print = t0
    fedge = open(a.out, "w", newline="")
    w = csv.writer(fedge)
    w.writerow(["t_us", "ch", "edge"])

    try:
        while time.time() - t0 < a.duration:
            data = ppk.get_data()
            if not data:
                time.sleep(0.001)
                continue
            samples, bits = ppk.get_samples(data)
            if not bits:
                continue
            b = np.asarray(bits, dtype=np.uint8)
            for c in chans:
                s = (b >> c) & 1
                p = s[0] if prev[c] is None else prev[c]
                d = np.diff(np.concatenate(([p], s)).astype(np.int8))
                for idx in np.flatnonzero(d == 1):
                    w.writerow([f"{(n_total + idx) * US_PER_SAMPLE:.0f}", c, "R"])
                    cnt[c] += 1
                for idx in np.flatnonzero(d == -1):
                    w.writerow([f"{(n_total + idx) * US_PER_SAMPLE:.0f}", c, "F"])
                prev[c] = s[-1]
            if a.current and samples:
                cur_sum += float(np.sum(samples))
                cur_n += len(samples)
            n_total += len(b)
            now = time.time()
            if now - last_print >= 1.0:
                if a.current and cur_n:
                    cur_rows.append([f"{now - t0:.1f}", f"{cur_sum / cur_n / 1000.0:.4f}"])
                    cur_sum, cur_n = 0.0, 0
                if a.monitor:
                    el = now - t0
                    print(f"t={el:6.1f}s samples={n_total} (kỳ vọng ~{int(el*FS)}) "
                          + " ".join(f"D{c}={cnt[c]}" for c in chans), end="\r")
                fedge.flush()
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        ppk.stop_measuring()
        if a.mode == "source":
            ppk.toggle_DUT_power("OFF")
        fedge.close()

    el = time.time() - t0
    exp = el * FS
    print(f"\n[✓] {n_total} mẫu trong {el:.1f}s (kỳ vọng ~{exp:.0f}, lệch {100*(n_total-exp)/exp:+.2f} %)")
    if abs(n_total - exp) / exp > 0.02:
        print("[!] Số mẫu lệch > 2 % so với đồng hồ máy: có thể MẤT MẪU, trục thời gian không tin cậy. "
              "Thu lại (cổng USB trực tiếp, đóng chương trình khác).")
    print("    Số sườn lên:", {f"D{c}": cnt[c] for c in chans}, "->", a.out)
    if a.current:
        with open(a.current, "w", newline="") as f:
            cw = csv.writer(f)
            cw.writerow(["t_s", "mean_mA"])
            cw.writerows(cur_rows)
        print("    Dòng TB theo giây ->", a.current)


if __name__ == "__main__":
    main()