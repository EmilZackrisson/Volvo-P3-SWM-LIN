import serial

SERIAL_PORT = "/dev/tty.usbserial-0001"
BAUD = 8448
SWM_PID = 0x20


def lin_pid_parity_ok(pid):
    id_bits = [(pid >> i) & 1 for i in range(6)]
    p0 = id_bits[0] ^ id_bits[1] ^ id_bits[2] ^ id_bits[4]
    p1 = not (id_bits[1] ^ id_bits[3] ^ id_bits[4] ^ id_bits[5])
    p1 = int(p1)
    return ((p0 | (p1 << 1)) == (pid >> 6))


def classic_checksum(data):
    return (0xFF - (sum(data) & 0xFF)) & 0xFF


def enhanced_checksum(data, pid):
    return (0xFF - ((pid + sum(data)) & 0xFF)) & 0xFF


def parse_lin_frames(buffer):
    frames = []
    i = 0
    while i < len(buffer) - 3:
        if buffer[i] == 0x55:  # SYNC
            pid = buffer[i+1]
            if lin_pid_parity_ok(pid):
                # Assume max 8 data bytes, look ahead
                for dl in range(1, 9):
                    if i + 2 + dl < len(buffer):
                        data = buffer[i+2:i+2+dl]
                        checksum = buffer[i+2+dl]
                        if checksum in [classic_checksum(data), enhanced_checksum(data, pid)]:
                            frames.append((pid, data, checksum))
                            i += 3 + dl
                            break
        i += 1
    return frames


def listen_lin():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0.5) as ser:
        print(f"Listening on {SERIAL_PORT} at {BAUD} baud for LIN frames...")
        buffer = []
        while True:
            data = ser.read(256)
            if data:
                buffer.extend(data)
                frames = parse_lin_frames(buffer)
                if frames:
                    for pid, data_bytes, checksum in frames:
                        if pid != SWM_PID:
                            continue

                        # Filter out empty frames
                        if [hex(b) for b in data_bytes] == ['0x0'] * len(data_bytes):
                            continue

                        print(
                            f"PID: 0x{pid:02X}, Data: {[hex(b) for b in data_bytes]}, Checksum: 0x{checksum:02X}")

                    buffer.clear()  # Reset after parsing


if __name__ == "__main__":
    listen_lin()
