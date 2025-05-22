import socket

ESP32_HOST = "192.168.217.106"  # Update this IP if needed
ESP32_PORT = 12345

def main():
    print(f"[INFO] Connecting to ESP32 at {ESP32_HOST}:{ESP32_PORT}...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((ESP32_HOST, ESP32_PORT))
            print("[INFO] Connected to ESP32.")

            # Send command to start streaming
            sock.sendall(b"startCubeStream\n")
            print("[INFO] Sent 'startCubeStream' command.")

            buffer = ""

            while True:
                data = sock.recv(1024)
                if not data:
                    break  # Connection closed
                buffer += data.decode()

                # Handle multiple lines in buffer
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line and "CUBE_STREAM_START" not in line:
                        try:
                            gx, gy, gz = map(float, line.split(','))
                            print(f"gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}")
                        except ValueError:
                            print(f"[WARN] Invalid line: {line}")

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        print("[INFO] Connection closed.")

if __name__ == "__main__":
    main()
