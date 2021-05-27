#!/usr/bin/python3

import sys
import random

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

class Chunk:
    def __init__(self, osid, type):
        self.id = ((osid | 8) << 4) | type
        self.data = bytearray()
        self.address = None
        
    def GetSize(self):
        return len(self.data)

    def GetID(self):
        return self.id

    def GetData(self):
        return self.data

    def GetAddress(self):
        return self.address

class StringChunk(Chunk):
    def __init__(self, type, s):
        Chunk.__init__(self, 7, type)
        self.data = bytearray(bytes(s + '\000', encoding='utf-8'))

class LoaderChunk(Chunk):
    def __init__(self, filename):
        Chunk.__init__(self, 0, 0)
        f = open(filename, "rb")
        self.data = bytearray(f.read())
        f.close()

class ModuleChunk(Chunk):
    def __init__(self, filename):
        Chunk.__init__(self, 0, 1)
        f = open(filename, "rb")
        self.data = bytearray(f.read())
        f.close()

class DirectoryChunk(Chunk):
    def __init__(self, address, nchunks):
        Chunk.__init__(self, 7, 0)
        self.address = address
        self.size = (8 * nchunks) + 4

    def GetSize(self):
        return self.size

def WriteChunkDirectoryAt(start, chunks):
    data = bytearray()
    address = start + (8 * len(chunks)) + 4
    for c in chunks:
        size = c.GetSize()
        data.append(c.GetID())
        data += int24(size)
        a = c.GetAddress()
        if a != None:
            data += int32(a)
        else:
            data += int32(address)
            address += size

    data += int32(0)

    for c in chunks:
        data += c.GetData()

    return data

def MakeEthernetID():
    ba = bytearray(random.randbytes(6))
    ba[0] |= 0x2
    return "%02x:%02x:%02x:%02x:%02x:%02x" % (ba[0], ba[1], ba[2], ba[3], ba[4], ba[5])

# first chunk directory in podule space
chunks = []
chunks.append(StringChunk(5, "EtherZ (c) 2021 Phil Blundell"))
chunks.append(StringChunk(7, MakeEthernetID()))
chunks.append(LoaderChunk("loader.bin"))
data += WriteChunkDirectoryAt(16, chunks)

# pad to page boundary
data += bytearray(0x800 - len(data))

# second chunk directory in code space
chunks = []
chunks.append(ModuleChunk("EtherZ/EtherZ,ffa"))
chunks.append(ModuleChunk("IDEWedge/IDEWedge,ffa"))
data += WriteChunkDirectoryAt(0, chunks)

sys.stdout.buffer.write(data)
