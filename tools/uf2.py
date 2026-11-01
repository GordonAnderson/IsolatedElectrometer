# Convert a PlatformIO firmware.bin to a UF2 file for the Adafruit SAMD21 UF2 bootloader (QT Py M0 and similar).
# Usage: python tools/uf2.py .pio/build/adafruit_qtpy_m0/firmware.bin fw.uf2 0x2000
# Then double-tap reset (green LED, QTPY_BOOT drive) and copy fw.uf2 onto the drive; the board reboots into it.
# Used when `pio run -t upload` (bossac) fails partway on a flaky USB port.
import struct,sys
src,dst,base=sys.argv[1],sys.argv[2],int(sys.argv[3],16)
d=open(src,'rb').read()
if len(d)%256: d+=b'\0'*(256-len(d)%256)
n=len(d)//256
out=b''
for i in range(n):
    chunk=d[i*256:(i+1)*256]
    hdr=struct.pack('<IIIIIIII',0x0A324655,0x9E5D5157,0x00002000,base+i*256,256,i,n,0x68ed2b88)
    out+=hdr+chunk+b'\0'*(476-256)+struct.pack('<I',0x0AB16F30)
open(dst,'wb').write(out); print(n,'blocks',len(out),'bytes')
