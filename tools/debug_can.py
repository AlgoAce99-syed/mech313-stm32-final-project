import serial
import time
import struct
import re

PORT = 'COM8'
BAUDRATE = 115200

def parse_can_line(line_bytes):
    try:
        # 바이트열을 문자열로 디코딩
        line_str = line_bytes.decode('ascii', errors='ignore').strip()
        if not line_str:
            return

        # CAN ID(200, 201, 202)와 8바이트 페이로드(16자리 16진수) 추출 정규식
        match = re.search(r'(200|201|202)[^\dA-Fa-f]*([0-9A-Fa-f]{16})', line_str)
        if not match:
            # 파싱할 수 없는 일반 문자열(어댑터 상태 메시지 등) 출력 시 아래 주석 해제
            # print(f"Raw: {line_str}")
            return

        can_id = int(match.group(1), 16)
        payload = bytes.fromhex(match.group(2))

        # 빅 엔디안(>) 규격으로 C 펌웨어와 동일한 바이트 구조 언패킹
        if can_id == 0x200:
            # id, iq, vd, vq (모두 int16_t)
            id_meas, iq_meas, vd_out, vq_out = struct.unpack('>hhhh', payload)
            print(f"[0x200] Id: {id_meas/100.0:5.2f} A | Iq: {iq_meas/100.0:5.2f} A | Vd: {vd_out/100.0:5.2f} V | Vq: {vq_out/100.0:5.2f} V")
        
        elif can_id == 0x201:
            # vel_meas, vel_ref, iq_ref (int16_t), fault (uint8_t), cnt (uint8_t)
            vel_meas, vel_ref, iq_ref, fault, cnt = struct.unpack('>hhhBB', payload)
            print(f"[0x201] Vel: {vel_meas/100.0:6.2f} rad/s | Ref: {vel_ref/100.0:6.2f} rad/s | Iq_ref: {iq_ref/100.0:5.2f} A | Fault: 0x{fault:02X} | Cnt: {cnt}")

        elif can_id == 0x202:
            # ia, ib, ic (int16_t), cnt (uint16_t)
            ia, ib, ic, cnt = struct.unpack('>hhhh', payload)
            print(f"[0x202] Ia: {ia/100.0:5.2f} A | Ib: {ib/100.0:5.2f} A | Ic: {ic/100.0:5.2f} A | Cnt: {cnt}")

    except Exception:
        pass

try:
    s = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    print(f"포트 열림: {PORT}")

    print("\n--- CAN 어댑터 활성화 명령 전송 ---")
    s.write(b'can on\r\n')
    time.sleep(0.1)
    s.write(b'can on\n')
    time.sleep(0.1)

    print("\n--- 실시간 FOC 텔레메트리 수신 시작 (Ctrl+C로 종료) ---")
    s.reset_input_buffer()
    
    while True:
        line = s.readline()
        if line:
            parse_can_line(line)

except KeyboardInterrupt:
    print("\n사용자에 의해 데이터 수신이 종료되었습니다.")
except serial.SerialException as e:
    print(f"\n직렬 포트 접근 오류: {e}\n어댑터 연결 상태 및 포트 번호를 확인하십시오.")
finally:
    if 's' in locals() and s.is_open:
        s.close()
        print("포트가 닫혔습니다.")