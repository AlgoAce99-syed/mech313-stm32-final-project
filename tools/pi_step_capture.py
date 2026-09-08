"""
pi_step_capture.py — FOC PI 튜닝용 스텝 응답 캡처 & 플롯
==========================================================
사용법:
    uv run --with pyserial --with matplotlib --with numpy python pi_step_capture.py COM5

동작:
    - 항상 2000ms 링버퍼 유지
    - iq_ref 0→非0 전환 감지 후 1000ms 더 수집
    - 스텝 기준 -1000ms ~ +1000ms 구간만 플롯/저장

CAN 프레임 (main.c 기준):
    ID=0x100: [iq_ref×100, iq_meas×100, id_ref×100, id_meas×100] int16 big-endian
    ID=0x101: [vd×100, vq×100, elec_theta×10000(uint16), cnt(uint16)]
"""

import sys, time, struct, threading, csv, os
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ── 한글 폰트 (Windows: 맑은 고딕) ─────────────────────────────────────────
plt.rcParams['font.family'] = 'Malgun Gothic'
plt.rcParams['axes.unicode_minus'] = False

# ── 설정 ─────────────────────────────────────────────────────────────────────
CAN_RATE_HZ        = 500      # 전송 주파수 (40분주 @ 20kHz)
BUFFER_MS          = 2000     # 상시 유지 링버퍼 크기 [ms]
PRE_STEP_MS        = 1000     # 스텝 이전 플롯 구간 [ms]
POST_STEP_MS       = 1000     # 스텝 이후 플롯 구간 [ms]
STEP_DETECT_THRESH = 0.05     # iq_ref 이 값 초과 시 스텝으로 판정 [A]
SETTLE_BAND_PCT    = 5        # 정착 시간 계산 기준 ±N%
# ─────────────────────────────────────────────────────────────────────────────

BUFFER_FRAMES    = int(BUFFER_MS    * CAN_RATE_HZ / 1000)
PRE_FRAMES       = int(PRE_STEP_MS  * CAN_RATE_HZ / 1000)
POST_FRAMES      = int(POST_STEP_MS * CAN_RATE_HZ / 1000)


def find_fdcanusb_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 or 'fdcan' in (p.description or '').lower():
            return p.device
    ports = list(serial.tools.list_ports.comports())
    if ports:
        print("[!] fdcanusb 자동 감지 실패. 사용 가능한 포트:")
        for p in ports: print(f"    {p.device}  —  {p.description}")
        return ports[0].device
    return None


class StepCapture:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.1)

        # 상시 2000ms 링버퍼
        self._ring = deque(maxlen=BUFFER_FRAMES)

        self.data       = []
        self.iq_target  = 0.0
        self._step_detected = False
        self._post_count    = 0
        self._latest_101    = None
        self._t0            = time.time()
        self._running       = True
        self.done_event     = threading.Event()

        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.write(b'can on\n')
        time.sleep(0.2)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[OK] {port} 연결. 스텝 입력 대기 중 (iq_ref > {STEP_DETECT_THRESH}A)...")

    def _parse(self, line: str):
        parts = line.strip().split()
        if len(parts) < 3 or parts[0] != 'rcv': return None
        try:    can_id = int(parts[1], 16)
        except: return None
        if can_id not in (0x100, 0x101): return None
        if len(parts[2]) != 16: return None
        try:    return can_id, bytes.fromhex(parts[2])
        except: return None

    def _read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if not line: continue
                result = self._parse(line)
                if result is None: continue

                can_id, raw = result
                t = time.time() - self._t0

                if can_id == 0x101:
                    vd  = struct.unpack('>h', raw[0:2])[0] / 100.0
                    vq  = struct.unpack('>h', raw[2:4])[0] / 100.0
                    elec= struct.unpack('>H', raw[4:6])[0] / 10000.0
                    self._latest_101 = (vd, vq, elec)
                    continue

                # 0x100
                iq_ref  = struct.unpack('>h', raw[0:2])[0] / 100.0
                iq_meas = struct.unpack('>h', raw[2:4])[0] / 100.0
                id_ref  = struct.unpack('>h', raw[4:6])[0] / 100.0
                id_meas = struct.unpack('>h', raw[6:8])[0] / 100.0
                vd, vq, elec = self._latest_101 if self._latest_101 else (0., 0., 0.)

                frame = dict(t=t,
                             iq_ref=iq_ref, iq_meas=iq_meas,
                             id_ref=id_ref, id_meas=id_meas,
                             vd=vd, vq=vq, elec=elec)

                # 항상 링버퍼에 쌓음
                self._ring.append(frame)

                if not self._step_detected:
                    if abs(iq_ref) > STEP_DETECT_THRESH:
                        self._step_detected = True
                        self.iq_target = iq_ref
                        self._post_count = 0
                        print(f"[STEP] t={t:.3f}s  iq_ref={iq_ref:.3f}A 감지 → "
                              f"이후 {POST_STEP_MS}ms 수집 중...")
                else:
                    self._post_count += 1
                    if self._post_count >= POST_FRAMES:
                        # 링버퍼 전체 = 스텝 전 ~1000ms + 스텝 후 1000ms
                        all_frames = list(self._ring)

                        # 링버퍼 안에서 스텝 위치 찾기 (iq_ref 처음 非0인 프레임)
                        step_idx = next(
                            (i for i, f in enumerate(all_frames)
                             if abs(f['iq_ref']) > STEP_DETECT_THRESH),
                            len(all_frames) - POST_FRAMES
                        )

                        start = max(0, step_idx - PRE_FRAMES)
                        end   = min(len(all_frames), step_idx + POST_FRAMES)
                        slice_ = all_frames[start:end]

                        # 시간을 스텝 기준 상대 시간으로 변환
                        t_step = all_frames[step_idx]['t']
                        for f in slice_:
                            f = dict(f)
                            f['t'] = f['t'] - t_step
                        self.data = [{**f, 't': f['t'] - t_step} for f in slice_]

                        print(f"[완료] {len(self.data)}프레임 추출 "
                              f"({self.data[0]['t']*1000:.0f}ms ~ "
                              f"{self.data[-1]['t']*1000:.0f}ms)")
                        self._running = False
                        self.done_event.set()

            except Exception:
                pass

    def close(self):
        self._running = False
        try:
            self.ser.write(b'can off\n')
            self.ser.close()
        except: pass


def compute_metrics(data, iq_target):
    t = np.array([d['t'] * 1000 for d in data])   # ms
    y = np.array([d['iq_meas']  for d in data])
    mask = t >= 0
    if not mask.any() or abs(iq_target) < 1e-6:
        return {}
    t_post, y_post = t[mask], y[mask]

    y_ss      = float(np.mean(y_post[int(len(y_post)*0.8):]))
    y_peak    = float(np.max(y_post)) if iq_target > 0 else float(np.min(y_post))
    overshoot = (y_peak - iq_target) / abs(iq_target) * 100.0

    lo, hi = iq_target * 0.1, iq_target * 0.9
    try:
        i_lo = next(i for i, v in enumerate(y_post) if v >= lo)
        i_hi = next(i for i, v in enumerate(y_post) if v >= hi)
        rise_ms = float(t_post[i_hi] - t_post[i_lo])
    except StopIteration:
        rise_ms = float('nan')

    band = abs(iq_target) * SETTLE_BAND_PCT / 100.0
    settle_ms = float('nan')
    for i in range(len(y_post)-1, -1, -1):
        if abs(y_post[i] - iq_target) > band:
            settle_ms = float(t_post[min(i+1, len(t_post)-1)])
            break

    return dict(y_ss=y_ss, overshoot=overshoot, rise_ms=rise_ms, settle_ms=settle_ms)


def save_csv(data, path):
    keys = ['t','iq_ref','iq_meas','id_ref','id_meas','vd','vq','elec']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in data:
            w.writerow({k: row.get(k, 0.0) for k in keys})
    print(f"[저장] {path}")


def plot(data, iq_target, csv_path):
    t       = [d['t']*1000  for d in data]
    iq_ref  = [d['iq_ref']  for d in data]
    iq_meas = [d['iq_meas'] for d in data]
    id_ref  = [d['id_ref']  for d in data]
    id_meas = [d['id_meas'] for d in data]
    vd      = [d['vd']      for d in data]
    vq      = [d['vq']      for d in data]
    iq_err  = [m - r for m, r in zip(iq_meas, iq_ref)]

    m    = compute_metrics(data, iq_target)
    band = abs(iq_target) * SETTLE_BAND_PCT / 100.0

    fig = plt.figure(figsize=(13, 10))
    fig.suptitle(
        f'PI 스텝 응답  |  iq_ref={iq_target:.2f}A  |  {os.path.basename(csv_path)}',
        fontsize=12
    )
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.48, wspace=0.35)

    ax_iq  = fig.add_subplot(gs[0, :])
    ax_id  = fig.add_subplot(gs[1, 0])
    ax_v   = fig.add_subplot(gs[1, 1])
    ax_err = fig.add_subplot(gs[2, 0])
    ax_txt = fig.add_subplot(gs[2, 1])
    ax_txt.axis('off')

    # ── iq 응답 ──────────────────────────────────────────────────────────────
    ax_iq.axvline(0, color='gray', lw=0.8, ls='--', label='Step t=0')
    ax_iq.axhline(iq_target, color='gray', lw=0.6, ls=':')
    ax_iq.fill_between(t, iq_target-band, iq_target+band,
                       color='green', alpha=0.12, label=f'±{SETTLE_BAND_PCT}% band')
    ax_iq.plot(t, iq_ref,  'k--', lw=1.2, label='iq_ref')
    ax_iq.plot(t, iq_meas, 'b-',  lw=1.5, label='iq_meas')
    if not np.isnan(m.get('settle_ms', float('nan'))):
        ax_iq.axvline(m['settle_ms'], color='green', lw=1.0, ls='-.',
                      label=f"정착 {m['settle_ms']:.1f}ms")
    title_parts = []
    if not np.isnan(m.get('rise_ms',    float('nan'))): title_parts.append(f"상승={m['rise_ms']:.1f}ms")
    if not np.isnan(m.get('overshoot',  float('nan'))): title_parts.append(f"오버슈트={m['overshoot']:.1f}%")
    if not np.isnan(m.get('settle_ms',  float('nan'))): title_parts.append(f"정착={m['settle_ms']:.1f}ms")
    if title_parts:
        ax_iq.set_title('  |  '.join(title_parts), fontsize=10)
    ax_iq.set_ylabel('iq [A]')
    ax_iq.set_xlabel('Time [ms]  (0 = 스텝)')
    ax_iq.set_xlim(t[0], t[-1])
    ax_iq.legend(fontsize=9)
    ax_iq.grid(True, alpha=0.3)

    # ── id 응답 ──────────────────────────────────────────────────────────────
    ax_id.plot(t, id_ref,  'k--', lw=1.0, label='id_ref')
    ax_id.plot(t, id_meas, 'r-',  lw=1.2, label='id_meas')
    ax_id.axvline(0, color='gray', lw=0.8, ls='--')
    ax_id.axhline(0, color='gray', lw=0.5)
    ax_id.set_ylabel('id [A]')
    ax_id.set_xlabel('Time [ms]')
    ax_id.set_title('d축 전류')
    ax_id.set_xlim(t[0], t[-1])
    ax_id.legend(fontsize=9)
    ax_id.grid(True, alpha=0.3)

    # ── vd, vq ───────────────────────────────────────────────────────────────
    ax_v.plot(t, vd, 'r-', lw=1.0, label='vd')
    ax_v.plot(t, vq, 'b-', lw=1.0, label='vq')
    ax_v.axvline(0, color='gray', lw=0.8, ls='--')
    ax_v.axhline(0, color='gray', lw=0.5)
    ax_v.set_ylabel('전압 [V]')
    ax_v.set_xlabel('Time [ms]')
    ax_v.set_title('PI 출력 전압 (vd, vq)')
    ax_v.set_xlim(t[0], t[-1])
    ax_v.legend(fontsize=9)
    ax_v.grid(True, alpha=0.3)

    # ── iq 오차 ──────────────────────────────────────────────────────────────
    ax_err.plot(t, iq_err, color='purple', lw=1.0)
    ax_err.axvline(0, color='gray', lw=0.8, ls='--')
    ax_err.axhline(0, color='gray', lw=0.5)
    ax_err.fill_between(t, -band, band, color='green', alpha=0.12)
    ax_err.set_ylabel('iq 오차 [A]')
    ax_err.set_xlabel('Time [ms]')
    ax_err.set_title(f'iq 오차 (meas - ref),  ±{SETTLE_BAND_PCT}% = ±{band:.3f}A')
    ax_err.set_xlim(t[0], t[-1])
    ax_err.grid(True, alpha=0.3)

    # ── 지표 텍스트 ──────────────────────────────────────────────────────────
    lines = [
        "── 스텝 응답 지표 ──",
        f"  iq_ref      : {iq_target:.3f} A",
        f"  정상상태(ss) : {m.get('y_ss', float('nan')):.4f} A",
        f"  오버슈트     : {m.get('overshoot', float('nan')):.2f} %",
        f"  상승 시간    : {m.get('rise_ms', float('nan')):.2f} ms",
        f"  정착 시간    : {m.get('settle_ms', float('nan')):.2f} ms",
        f"                  (±{SETTLE_BAND_PCT}% 기준)",
        "",
        f"  총 프레임    : {len(data)}",
        f"  구간         : {t[0]:.0f} ~ {t[-1]:.0f} ms",
    ]
    ax_txt.text(0.05, 0.95, '\n'.join(lines),
                transform=ax_txt.transAxes, va='top',
                fontsize=10, family='Malgun Gothic',
                bbox=dict(boxstyle='round', facecolor='#f0f0f0', alpha=0.8))

    png_path = csv_path.replace('.csv', '.png')
    plt.savefig(png_path, dpi=150, bbox_inches='tight')
    print(f"[저장] {png_path}")
    plt.show()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_fdcanusb_port()
    if port is None:
        print("[ERROR] 포트를 찾을 수 없습니다.  python pi_step_capture.py COMx")
        sys.exit(1)

    cap = StepCapture(port)
    timeout = (BUFFER_MS + POST_STEP_MS + 5000) / 1000.0

    try:
        cap.done_event.wait(timeout=timeout)
    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C")
    finally:
        cap.close()

    if not cap.data:
        print("[경고] 캡처된 데이터 없음. 스텝이 발생했는지 확인하세요.")
        return

    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(os.path.dirname(__file__), f'pi_step_{ts}.csv')
    save_csv(cap.data, csv_path)
    plot(cap.data, cap.iq_target, csv_path)


if __name__ == '__main__':
    main()
