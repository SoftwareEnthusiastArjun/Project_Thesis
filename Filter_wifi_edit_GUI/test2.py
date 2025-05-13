import socket
s = socket.socket()
s.settimeout(3)
s.connect(("192.168.211.106", 12345))
s.sendall(b"get\n")
print(s.recv(1024).decode())
s.close()
