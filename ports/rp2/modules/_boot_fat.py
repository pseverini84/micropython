import os
import machine, rp2
import aps6408l
import oscilloscope

machine.freq(250000000)
oscilloscope.sample(100,8168)#call sampe to reset the pio at 250Mhz
oscilloscope.range(33000)#set range acording my filter

# Try to mount the filesystem, and format the flash if it doesn't exist.
bdev = rp2.Flash()
try:
    vfs = os.VfsFat(bdev)
except:
    os.VfsFat.mkfs(bdev)
    vfs = os.VfsFat(bdev)
os.mount(vfs, "/")

bdev=aps6408l
try:
    vfs = os.VfsFat(bdev)
except:
    os.VfsFat.mkfs(bdev)
    vfs = os.VfsFat(bdev)
os.mount(vfs, "/ram")

del bdev, vfs, aps6408l
