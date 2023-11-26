import socket
import signal

global conn,s

IP      = ""
PORT    = xxxx

msg     = ""
# flag    = True


def send():
    global msg
    msg = f""
    teleport()

def teleport():
    conn.sendall(msg.encode())
    conn.recv(1)

def establish():
    global conn,s
    print("Starting from",IP)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((IP,PORT))
    print("Binded successfully.")
    s.listen()
    conn,addr = s.accept()

    print(f"Connected by {addr} at {IP}:{PORT}")


def main():

    establish()
    rclpy.init_node('socket_node')
    rclpy.Subscriber('/joint_states',xxxx,xxxx)
    send()


if __name__=="__main__":
    main()
