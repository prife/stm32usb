Import('RTT_ROOT')
Import('rtconfig')
from building import *

cwd     = os.path.join(str(Dir('#')), 'usb')
src	= Glob('*.c')
CPPPATH = [cwd, str(Dir('#'))]

group = DefineGroup('usb', src, depend = [''], CPPPATH = CPPPATH)

Return('group')
