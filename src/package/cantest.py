#!/usr/bin/env python
# -*- coding: utf-8 -*-

import MCP2515
import time

print("--------------------------------------------------------")
# try:
# This setup is referred to CAN SPI click mounted on flip n click device slot A 
can = MCP2515.MCP2515()
print("init...")
can.Init()
print("send data...")
id = 0x0 #max 7ff
data = [1, 2, 3, 4, 5, 6, 7, 8]
dlc = 8
can.Send(id, data, dlc)

readbuf = []
# while(1):
while(1):
	readbuf = can.Receive()
	print(readbuf)
	time.sleep(0.5)

print("--------------------------------------------------------")
# except Exception as e:
    # print(e)




