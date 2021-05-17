#!/usr/bin/python

import sys

ptype = 3
mfrid = 80
country = 0

data = bytearray()

def int16(x):
    return bytearray([x & 0xff, (x >> 8) & 0xff])

def int24(x):
    return bytearray([x & 0xff, (x >> 8) & 0xff, (x >> 16) & 0xff])

def int32(x):
    return bytearray([x & 0xff, (x >> 8) & 0xff, (x >> 16) & 0xff, (x >> 24) & 0xff])

data.append(0)
data.append(0b00000011)
data.append(0)
data += int16(ptype)
data += int16(mfrid)
data.append(country)

# interrupt status pointers
data.append(2)
data += int24(0x3c00)
data.append(1)
data += int24(0x3c00)

assert(len(data) == 16)

chunks = []

class Chunk:
    def __init__(self, osid, type):
        self.id = ((osid | 8) << 4) | type
        self.data = bytearray()
        
    def GetSize(self):
        return len(self.data)

    def GetID(self):
        return self.id

    def GetData(self):
        return self.data

class StringChunk(Chunk):
    def __init__(self, type, s):
        Chunk.__init__(self, 7, type)
        self.data = bytearray(s + "\000")

chunks.append(StringChunk(5, "EtherZ (c) 2021 Phil Blundell"))
        
address = 16 + (8 * len(chunks)) + 4

for c in chunks:
    size = c.GetSize()
    data.append(c.GetID())
    data += int24(size)
    data += int32(address)
    address += size

data += int32(0)

for c in chunks:
    data += c.GetData()

sys.stdout.write(data)
