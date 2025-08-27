import serial
import time

SERIAL_PORT = "/dev/tty.usbserial-0001"
COMMON_BAUDS = [9600, 10417, 19200]
SWEEP_PERCENT = 15
READ_TIME = 1.5


def lin_pid_parity_ok(pid):
    id_bits = [(pid >> i) & 1 for i in range(6)]
    p0 = id_bits[0] ^ id_bits[1] ^ id_bits[2] ^ id_bits[4]
    p1 = not (id_bits[1] ^ id_bits[3] ^ id_bits[4] ^ id_bits[5])
    p1 = int(p1)
    return ((p0 | (p1 << 1)) == (pid >> 6))


def try_detect_frames(data):
    frames = []
    for i in range(len(data)-3):
        if data[i] in (0x55, 0x54):  # SYNC or near-sync
            pid = data[i+1]
            if lin_pid_parity_ok(pid):
                frames.append((pid, data[i+2:i+10]))
    return frames


def sweep_baud():
    best_frames = []
    best_baud = None

    for base in COMMON_BAUDS:
        sweep_range = range(int(base*(1-SWEEP_PERCENT/100)),
                            int(base*(1+SWEEP_PERCENT/100)), int(base*0.01))
        for baud in sweep_range:
            try:
                with serial.Serial(SERIAL_PORT, baud, timeout=0.5) as ser:
                    raw = ser.read(512)
                    data = list(raw)
                    frames = try_detect_frames(data)
                    if frames:
                        print(f"Candidate baud {baud}, frames: {len(frames)}")
                        if len(frames) > len(best_frames):
                            best_frames = frames
                            best_baud = baud
            except:
                continue
    return best_baud, best_frames


if __name__ == "__main__":
    print("Sweeping baud rates...")
    baud, frames = sweep_baud()
    if baud:
        print(f"\n✅ Best match: {baud} baud")
        for pid, payload in frames[:5]:
            print(f"PID: 0x{pid:02X}, Data: {[hex(b) for b in payload]}")
    else:
        print("No LIN frames detected.")
