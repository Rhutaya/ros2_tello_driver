import socket

def main():
    tello_ip = '192.168.10.1'  # Tello drone's IP address
    tello_port = 8889          # Tello drone's command port
    local_port = 8889          # Local port for receiving responses

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', local_port))

    # Set a timeout for the socket
    sock.settimeout(10)

    try:
        # Send the "command" message to the Tello drone
        message = "command"
        sock.sendto(message.encode(), (tello_ip, tello_port))
        print(f"Sent message: {message}")

        # Receive the response from the Tello drone
        response, _ = sock.recvfrom(1024)
        print(f"Received response: {response.decode()}")

    except socket.timeout:
        print("Error: No response received within 10 seconds.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
