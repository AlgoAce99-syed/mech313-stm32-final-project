import serial
import time
import struct
import re

PORT = 'COM16'
BAUDRATE = 115200

# DRV8323 Fault Status 1 (레지스터 0x00) 비트 정의
FAULT_STATUS1 = {
    10: "FAULT",
    9:  "VGS_HA", 8: "VGS_LA",
    7:  "VGS_HB", 6: "VGS_LB",
    5:  "VGS_HC", 4: "VGS_LC",
    3:  "OTW",    2: "OTSD",
    1:  "UVP",    0: "GDF",
}

# DRV8323 Fault Status 2 (레지스터 0x01) 비트 정의
FAULT_STATUS2 = {
    10: "SA_OC", 9: "SB_OC", 8: "SC_OC",
    7:  "OCP_HA", 6: "OCP_LA",
    5:  "OCP_HB", 4: "OCP_LB",
    3:  "OCP_HC", 2: "OCP_LC",
    1:  "VDS_HA", 0: "VDS_LA",
}

def decode_fault(reg_val, bit_map):
    active = [name for bit, name in bit_map.items() if reg_val & (1 << bit)]
    return ', '.join(active) if active else "None"

def parse_can_line(line_bytes):
    try:
        line_str = line_bytes.decode('ascii', errors='ignore').strip()
        if not line_str:
            return

        match = re.search(r'(200|201|202)[^\dA-Fa-f]*([0-9A-Fa-f]{16})', line_str)
        if not match:
            return

        can_id  = int(match.group(1), 16)
        payload = bytes.fromhex(match.group(2))

        if can_id == 0x200:
            id_meas, iq_meas, vd_out, vq_out = struct.unpack('>hhhh', payload)
            print(f"[0x200] Id: {id_meas/100.0:6.3f} A | Iq: {iq_meas/100.0:6.3f} A | "
                  f"Vd: {vd_out/100.0:6.3f} V | Vq: {vq_out/100.0:6.3f} V")

        elif can_id == 0x201:
            # 펌웨어: tenc(uint16), encoder_raw(uint16), cnt(uint16), 0, 0
            tenc, enc_raw, cnt = struct.unpack('>HHH', payload[:6])
            theta_deg = tenc / 10000.0 * 57.2958
            print(f"[0x201] θ_e: {tenc/10000.0:.4f} rad ({theta_deg:.1f}°) | "
                f"encoder_raw: {enc_raw} | Cnt: {cnt}")

        elif can_id == 0x202:
            ia, ib, ic, cnt = struct.unpack('>hhhh', payload)
            print(f"[0x202] Ia: {ia/100.0:6.3f} A | Ib: {ib/100.0:6.3f} A | "
                  f"Ic: {ic/100.0:6.3f} A | Cnt: {cnt}")

    except Exception as e:
        pass

try:
    s = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    print(f"포트 열림: {PORT}")

    s.write(b'can on\r\n')
    time.sleep(0.1)
    s.write(b'can on\n')
    time.sleep(0.1)

    print("--- 실시간 FOC 텔레메트리 수신 시작 (Ctrl+C로 종료) ---")
    s.reset_input_buffer()

    while True:
        line = s.readline()
        if line:
            parse_can_line(line)

except KeyboardInterrupt:
    print("\n종료")
except serial.SerialException as e:
    print(f"\n포트 오류: {e}")
finally:
    if 's' in locals() and s.is_open:
        s.close()