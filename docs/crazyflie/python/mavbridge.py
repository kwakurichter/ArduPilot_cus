import socket
import struct
import threading
import time

# --- Configuration ---
DECK_IP = "192.168.4.1"
DECK_PORT = 5000

# --- QGC Configuration ---
# QGC will listen on this port. Our script will SEND to this port.
QGC_LISTEN_PORT = 14550

# Our script will listen on this port. QGC will automatically SEND to this port
# after it receives the first message from us.
BRIDGE_LISTEN_PORT = 14551

GCS_IP = "127.0.0.1"

# --- CPX/WIFI Protocol Definitions ---
CPX_T_WIFI_HOST = 3
CPX_T_GAP8 = 4
CPX_VERSION = 0
WIFI_MAV_DATA = 0x41
CPX_F_CONSOLE = 2
CPX_F_MAVLINK_DOWNLINK = 7

# --- Global Variables ---
deck_socket = None
stop_event = threading.Event()
socket_lock = threading.Lock()

def build_cpx_packet_for_gcs(mavlink_payload):
    byte1 = (CPX_T_GAP8 & 0x7) | ((CPX_T_WIFI_HOST & 0x7) << 3)
    byte2 = (CPX_F_MAVLINK_DOWNLINK & 0x3F) | ((CPX_VERSION & 0x3) << 6)
    cpx_routing_header = struct.pack('<BB', byte1, byte2)
    command_payload = struct.pack('<B', WIFI_MAV_DATA) + mavlink_payload
    payload_length = len(cpx_routing_header) + len(command_payload)
    transport_header = struct.pack('<H', payload_length)
    return transport_header + cpx_routing_header + command_payload

def deck_to_gcs_thread(gcs_udp_socket):
    print("[Deck->GCS] Thread started.")
    buffer = bytearray()
    while not stop_event.is_set():
        try:
            with socket_lock:
                deck_socket.setblocking(False)
                try:
                    data = deck_socket.recv(4096)
                    if not data:
                        print("[Deck->GCS] Deck connection closed.")
                        stop_event.set()
                        break
                    buffer.extend(data)
                except BlockingIOError:
                    pass
                finally:
                    deck_socket.setblocking(True)

            while len(buffer) >= 4:
                payload_len = struct.unpack('<H', buffer[:2])[0]
                total_packet_len = payload_len + 2
                if len(buffer) >= total_packet_len:
                    packet_data = buffer[:total_packet_len]
                    buffer = buffer[total_packet_len:]
                    routing_byte, function_byte = struct.unpack('<BB', packet_data[2:4])
                    function = function_byte & 0x3F
                    if function == CPX_F_CONSOLE:
                        mavlink_payload = packet_data[4:]
                        # Send to the port QGC is listening on
                        gcs_udp_socket.sendto(mavlink_payload, (GCS_IP, QGC_LISTEN_PORT))
                else:
                    break
            time.sleep(0.001)
        except Exception as e:
            if not stop_event.is_set():
                print(f"[Deck->GCS] An unexpected error occurred: {e}")
            break
    print("[Deck->GCS] Thread finished.")

def gcs_to_deck_thread(gcs_udp_socket):
    print("[GCS->Deck] Thread started.")
    while not stop_event.is_set():
        try:
            mavlink_payload, addr = gcs_udp_socket.recvfrom(2048)
            packet_to_send = build_cpx_packet_for_gcs(mavlink_payload)
            with socket_lock:
                deck_socket.sendall(packet_to_send)
        except Exception as e:
            if not stop_event.is_set():
                print(f"[GCS->Deck] An error occurred: {e}")
            break
    print("[GCS->Deck] Thread finished.")

def main():
    global deck_socket
    # This socket is for both sending and receiving from the GCS perspective
    gcs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # bind to the port QGC will send to
    gcs_socket.bind((GCS_IP, BRIDGE_LISTEN_PORT))
    # Add SO_REUSEADDR to prevent "Address already in use" errors on quick restarts
    gcs_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


    try:
        print(f"[*] Connecting to AI-Deck at {DECK_IP}:{DECK_PORT}...")
        deck_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        deck_socket.connect((DECK_IP, DECK_PORT))
        print("[*] Connection to AI-Deck successful!")
        print(f"[*] Bridge is listening for QGC on UDP port {BRIDGE_LISTEN_PORT}")
        print(f"[*] Forwarding telemetry to QGC on UDP port {QGC_LISTEN_PORT}")

    except Exception as e:
        print(f"[ERROR] Could not connect to AI-Deck: {e}")
        return

    d2g_thread = threading.Thread(target=deck_to_gcs_thread, args=(gcs_socket,), daemon=True)
    g2d_thread = threading.Thread(target=gcs_to_deck_thread, args=(gcs_socket,), daemon=True)
    
    d2g_thread.start()
    g2d_thread.start()
    
    print("\n[*] MAVLink bridge is running. Configure QGC and press Ctrl+C here to stop.")
    
    try:
        d2g_thread.join()
    except KeyboardInterrupt:
        print("\n[*] Shutdown requested.")
    finally:
        stop_event.set()
        if deck_socket:
            deck_socket.close()
        gcs_socket.close()
        print("[*] Bridge shut down.")

if __name__ == "__main__":
    main()
