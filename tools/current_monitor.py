"""
current_monitor.py — 실시간 전기각 진단용 모니터링 스크립트
==================================================================
수신 CAN 프레임 규격 (펌웨어 main.c 기준):
  0x100: [iq_ol, id_ol, iq_enc, id_enc] (int16, x100, 0.01A)
  0x101: [theta_ol, theta_enc, enc_raw, cnt] (uint16, x10000 / enc_raw 및 cnt는 그대로)
  0x102: [ia, ib, ic, cnt] (int16, x100, 0.01A)
"""

import sys
import time
import struct
import threading
from collections import deque

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 설정 매개변수
CAN_ID_DQ    = '100'
CAN_ID_THETA = '101'
CAN_ID_ABC   = '102'

MAX_POINTS   = 5000
PLOT_WINDOW  = 5.0        # X축 데이터 출력 범위 (sec)
CURRENT_YLIM = (-10.0, 10.0)
THETA_YLIM   = (-0.5, 6.5) # 0 ~ 2*pi 범위를 수용하기 위한 한계값
UPDATE_MS    = 10

def find_fdcanusb_port():
    ports = list(serial.tools.list_ports.comports())
    print(f"[포트 탐색] 연결된 포트 {len(ports)}개:")
    for p in ports:
        print(f"  {p.device}  VID={p.vid:#06x}  desc={p.description}")
    for port in ports:
        if port.vid == 0x0483 or 'fdcan' in (port.description or '').lower():
            print(f"[자동 선택] {port.device}")
            return port.device
    if ports:
        print(f"[자동 선택] 첫 번째 포트: {ports[0].device}")
        return ports[0].device
    print("[오류] 연결된 포트 없음. fdcanusb가 꽂혀 있는지 확인하세요.")
    return None

class FdcanusbReader:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.lock = threading.Lock()

        # 0x100 버퍼: dq 전류
        self.dq_times  = deque(maxlen=MAX_POINTS)
        self.iq_ol_buf = deque(maxlen=MAX_POINTS)
        self.id_ol_buf = deque(maxlen=MAX_POINTS)
        self.iq_en_buf = deque(maxlen=MAX_POINTS)
        self.id_en_buf = deque(maxlen=MAX_POINTS)

        # 0x101 버퍼: 전기각
        self.th_times  = deque(maxlen=MAX_POINTS)
        self.th_ol_buf = deque(maxlen=MAX_POINTS)
        self.th_en_buf = deque(maxlen=MAX_POINTS)
        self.en_raw_buf= deque(maxlen=MAX_POINTS)

        # 0x102 버퍼: 3상 전류
        self.abc_times = deque(maxlen=MAX_POINTS)
        self.ia_buf    = deque(maxlen=MAX_POINTS)
        self.ib_buf    = deque(maxlen=MAX_POINTS)
        self.ic_buf    = deque(maxlen=MAX_POINTS)

        self.frame_count  = 0
        self.missed_count = 0
        self._last_counter = None
        self._t0 = time.time()
        self._running = True

        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.write(b'can on\n')
        time.sleep(0.2)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _parse_line(self, line: str):
        parts = line.strip().split()
        if len(parts) < 4 or parts[0] != 'rcv':
            return None

        can_id = parts[1].upper().lstrip('0') or '0'
        if can_id not in {CAN_ID_DQ, CAN_ID_THETA, CAN_ID_ABC}:
            return None

        hex_data = parts[2]
        if len(hex_data) != 16:
            return None

        raw = bytes.fromhex(hex_data)

        if can_id == CAN_ID_DQ:
            v0 = struct.unpack('>h', raw[0:2])[0] / 100.0
            v1 = struct.unpack('>h', raw[2:4])[0] / 100.0
            v2 = struct.unpack('>h', raw[4:6])[0] / 100.0
            v3 = struct.unpack('>h', raw[6:8])[0] / 100.0
            return ('dq', v0, v1, v2, v3, None)

        elif can_id == CAN_ID_THETA:
            v0 = struct.unpack('>H', raw[0:2])[0] / 10000.0
            v1 = struct.unpack('>H', raw[2:4])[0] / 10000.0
            v2 = struct.unpack('>H', raw[4:6])[0]
            cnt = struct.unpack('>H', raw[6:8])[0]
            return ('theta', v0, v1, v2, None, cnt)

        elif can_id == CAN_ID_ABC:
            v0 = struct.unpack('>h', raw[0:2])[0] / 100.0
            v1 = struct.unpack('>h', raw[2:4])[0] / 100.0
            v2 = struct.unpack('>h', raw[4:6])[0] / 100.0
            cnt = struct.unpack('>H', raw[6:8])[0]
            return ('abc', v0, v1, v2, None, cnt)

    def _read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if not line:
                    continue
                result = self._parse_line(line)
                if result is None:
                    continue

                kind = result[0]
                t = time.time() - self._t0

                with self.lock:
                    if kind == 'abc':
                        ia, ib, ic, _, cnt = result[1], result[2], result[3], result[4], result[5]
                        self.abc_times.append(t)
                        self.ia_buf.append(ia)
                        self.ib_buf.append(ib)
                        self.ic_buf.append(ic)
                        
                        if self._last_counter is not None:
                            expected = (self._last_counter + 1) & 0xFFFF
                            if cnt != expected:
                                self.missed_count += 1
                        self._last_counter = cnt
                        self.frame_count += 1

                    elif kind == 'dq':
                        iq_ol, id_ol, iq_en, id_en = result[1], result[2], result[3], result[4]
                        self.dq_times.append(t)
                        self.iq_ol_buf.append(iq_ol)
                        self.id_ol_buf.append(id_ol)
                        self.iq_en_buf.append(iq_en)
                        self.id_en_buf.append(id_en)

                    elif kind == 'theta':
                        th_ol, th_en, en_raw, _ = result[1], result[2], result[3], result[4]
                        self.th_times.append(t)
                        self.th_ol_buf.append(th_ol)
                        self.th_en_buf.append(th_en)
                        self.en_raw_buf.append(en_raw)

            except Exception:
                pass

    def snapshot(self):
        with self.lock:
            return (
                list(self.dq_times), list(self.iq_ol_buf), list(self.id_ol_buf), list(self.iq_en_buf), list(self.id_en_buf),
                list(self.th_times), list(self.th_ol_buf), list(self.th_en_buf), list(self.en_raw_buf),
                list(self.abc_times), list(self.ia_buf), list(self.ib_buf), list(self.ic_buf)
            )

    def close(self):
        self._running = False
        self.ser.write(b'can off\n')
        self.ser.close()

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_fdcanusb_port()
    if port is None:
        input("포트를 찾을 수 없습니다. Enter 키를 눌러 종료...")
        sys.exit(1)
    print(f"[연결] {port} 열기 시도...")

    reader = FdcanusbReader(port)

    fig, (ax_dq, ax_th, ax_abc) = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    fig.subplots_adjust(hspace=0.3)

    # 1. d-q축 전류 (오픈루프 vs 엔코더)
    ax_dq.set_title('Park Transform Output: d-q Currents (0x100)')
    ax_dq.set_ylabel('Current [A]')
    ax_dq.set_ylim(*CURRENT_YLIM)
    ax_dq.grid(True, alpha=0.3)
    line_iq_ol, = ax_dq.plot([], [], 'r-', label='i_q (OpenLoop)', linewidth=1.5)
    line_id_ol, = ax_dq.plot([], [], 'r--', label='i_d (OpenLoop)', linewidth=1.0)
    line_iq_en, = ax_dq.plot([], [], 'b-', label='i_q (Encoder)', linewidth=1.5, alpha=0.6)
    line_id_en, = ax_dq.plot([], [], 'b--', label='i_d (Encoder)', linewidth=1.0, alpha=0.6)
    ax_dq.legend(loc='upper right', fontsize=9)
    dq_text = ax_dq.text(0.01, 0.95, '', transform=ax_dq.transAxes, va='top', fontsize=9)

    # 2. 전기각 비교
    ax_th.set_title('Electrical Angle Synchronization (0x101)')
    ax_th.set_ylabel('Angle [rad]')
    ax_th.set_ylim(*THETA_YLIM)
    ax_th.grid(True, alpha=0.3)
    line_th_ol, = ax_th.plot([], [], 'r-', label='Theta OpenLoop', linewidth=1.5)
    line_th_en, = ax_th.plot([], [], 'b-', label='Theta Encoder', linewidth=1.5)
    
    ax_th2 = ax_th.twinx()
    ax_th2.set_ylabel('Encoder Raw', color='orange')
    ax_th2.set_ylim(0, 16383)
    line_en_raw, = ax_th2.plot([], [], color='orange', linestyle=':', label='Raw Count', linewidth=1.0)
    
    lines_th = [line_th_ol, line_th_en, line_en_raw]
    ax_th.legend(lines_th, [l.get_label() for l in lines_th], loc='upper right', fontsize=9)
    th_text = ax_th.text(0.01, 0.95, '', transform=ax_th.transAxes, va='top', fontsize=9)

    # 3. 3상 원시 전류
    ax_abc.set_title('Raw Phase Currents (0x102)')
    ax_abc.set_ylabel('Current [A]')
    ax_abc.set_xlabel('Time [s]')
    ax_abc.set_ylim(*CURRENT_YLIM)
    ax_abc.grid(True, alpha=0.3)
    line_ia, = ax_abc.plot([], [], 'r-', label='Phase A')
    line_ib, = ax_abc.plot([], [], 'g-', label='Phase B')
    line_ic, = ax_abc.plot([], [], 'b-', label='Phase C')
    ax_abc.legend(loc='upper right', fontsize=9)

    def _set_xlim(ax, t_list):
        if not t_list: return
        t_now = t_list[-1]
        ax.set_xlim(max(0.0, t_now - PLOT_WINDOW), t_now + 0.1)

    def update(_):
        data = reader.snapshot()
        t_dq, iq_ol, id_ol, iq_en, id_en = data[0:5]
        t_th, th_ol, th_en, en_raw       = data[5:9]
        t_abc, ia, ib, ic                = data[9:13]

        artists = [line_iq_ol, line_id_ol, line_iq_en, line_id_en, dq_text,
                   line_th_ol, line_th_en, line_en_raw, th_text,
                   line_ia, line_ib, line_ic]

        if t_dq:
            _set_xlim(ax_dq, t_dq)
            line_iq_ol.set_data(t_dq, iq_ol)
            line_id_ol.set_data(t_dq, id_ol)
            line_iq_en.set_data(t_dq, iq_en)
            line_id_en.set_data(t_dq, id_en)
            dq_text.set_text(f"OpenLoop: Iq={iq_ol[-1]:+.2f}A, Id={id_ol[-1]:+.2f}A | Encoder: Iq={iq_en[-1]:+.2f}A, Id={id_en[-1]:+.2f}A")

        if t_th:
            _set_xlim(ax_th, t_th)
            line_th_ol.set_data(t_th, th_ol)
            line_th_en.set_data(t_th, th_en)
            line_en_raw.set_data(t_th, en_raw)
            th_text.set_text(f"Theta OL={th_ol[-1]:.3f}rad | Theta EN={th_en[-1]:.3f}rad")

        if t_abc:
            _set_xlim(ax_abc, t_abc)
            line_ia.set_data(t_abc, ia)
            line_ib.set_data(t_abc, ib)
            line_ic.set_data(t_abc, ic)

        return artists

    ani = animation.FuncAnimation(fig, update, interval=UPDATE_MS, blit=True, cache_frame_data=False)
    try:
        plt.show()
    finally:
        reader.close()

if __name__ == '__main__':
    main()