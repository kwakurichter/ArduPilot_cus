import socket
import struct
import threading
import time

# --- Configuration ---
IP_ADDRESS = "192.168.4.1"
PORT = 5000

# --- CPX/WIFI Protocol Definitions ---
CPX_T_WIFI_HOST = 3
CPX_T_GAP8 = 4
CPX_F_WIFI_CTRL = 4
CPX_F_CONSOLE = 2 # The GAP8 sends log messages on the CONSOLE function
CPX_VERSION = 0
WIFI_CTRL_USER_COMMAND = 0x40

def parse_cpx_packet(data):
    """
    Parses an incoming CPX packet from the AI-deck.
    If the packet is a console message, it decodes and returns the string.
    """
    try:
        # First 2 bytes are the transport layer payload length
        payload_length = struct.unpack('<H', data[:2])[0]
        
        # Next 2 bytes are the CPX routing header
        routing_header = data[2:4]
        
        # Unpack the routing header to identify the packet's function
        byte1, byte2 = struct.unpack('<BB', routing_header)
        function = byte2 & 0x3F # Mask to get the function bits

        # The rest of the data is the payload
        payload = data[4:2 + payload_length]

        # We only care about console messages coming from the deck
        if function == CPX_F_CONSOLE:
            return payload.decode('utf-8', errors='ignore').strip()

    except Exception as e:
        print(f"Error parsing packet: {e}")
        
    return None

def build_command_packet(command_str):
    """
    Builds a CPX packet ready to be sent over the socket
    containing the user-specified command string.
    """
    # 1. Build the CPX routing header
    #    - Destination: GAP8
    #    - Source: WIFI_HOST
    #    - Function: WIFI_CTRL
    byte1 = (CPX_T_GAP8 & 0x7) | ((CPX_T_WIFI_HOST & 0x7) << 3)
    byte2 = (CPX_F_WIFI_CTRL & 0x3F) | ((CPX_VERSION & 0x3) << 6)
    cpx_routing_header = struct.pack('<BB', byte1, byte2)

    # 2. Build the command payload
    #    - A byte indicating it's a user command, followed by the command string
    command_payload = struct.pack('<B', WIFI_CTRL_USER_COMMAND) + command_str.encode('utf-8')
    
    # 3. Build the final transport packet
    #    - The total length of the CPX header + payload
    #    - Followed by the header and payload themselves
    payload_length = len(cpx_routing_header) + len(command_payload)
    transport_header = struct.pack('<H', payload_length)
    
    return transport_header + cpx_routing_header + command_payload


def listener_thread(sock):
    """
    This function runs in a separate thread and continuously listens
    for incoming data from the socket, printing any received messages.
    """
    print("Listener thread started.")
    try:
        while True:
            # Block and wait for data from the AI-deck (up to 2048 bytes)
            data = sock.recv(2048)
            if not data:
                print("Connection closed by AI-deck.")
                break
            
            # Parse the incoming packet to extract the message
            message = parse_cpx_packet(data)
            if message:
                print(f"<<< Received from AI-deck: {message}")

    except ConnectionResetError:
        print("Connection was forcibly closed by the remote host.")
    except Exception:
        # This will happen when the main thread closes the socket
        print("Listener thread exiting.")


# --- Main Execution ---
def main():
    """
    Main function to connect, start the listener, and handle user input.
    """
    # Use a 'with' statement for automatic socket cleanup
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"Connecting to {IP_ADDRESS}:{PORT}...")
            s.connect((IP_ADDRESS, PORT))
            print("Connection successful. Ready to send commands.")

            # --- Start the Listener Thread ---
            # 'daemon=True' ensures the thread exits when the main program does
            listener = threading.Thread(target=listener_thread, args=(s,), daemon=True)
            listener.start()

            # --- Main Command Loop ---
            while True:
                # Wait for the user to type a command
                command = input(">>> Enter command ('start_mission', 'reset_mission', 'terminate') or 'exit': ")

                # Check for exit condition
                if command.lower() == 'exit':
                    break
                
                # Build and send the packet for the entered command
                if command:
                    packet = build_command_packet(command)
                    print(f"--- Sending command: '{command}' ---")
                    s.sendall(packet)
                
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    except ConnectionRefusedError:
        print("Connection failed. Is the AI-deck running and connected to the network?")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Closing connection.")

if __name__ == "__main__":
    main()