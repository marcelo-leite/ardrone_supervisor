#!/usr/bin/env python
# -*- coding: utf-8 -*-


import socket
import sys

server_address = ('192.168.1.1', 5555)
SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
SOCKET.connect(server_address)

data = SOCKET.recv(4024)
print(data)


