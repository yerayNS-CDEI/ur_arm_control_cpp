import socket

HOST = '0.0.0.0'
HOST = '192.168.1.102'    # '172.16.103.8' #IP del PC
PORT = 40000
# pose = '(111, 222, 333)'
msg = "True"

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, address = s.accept()
    with conn:
        print('Connected by', address)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            if "ready" in str(data):
                print(str(data))
                conn.sendall(msg.encode())
                print(str(msg.encode()))
                print('Data sent.')
