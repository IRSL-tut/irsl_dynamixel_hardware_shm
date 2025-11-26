import sys
install_space = '/usr/local'
# install_space = '../../../install'
sys.path.append(f'{intsall_space}/share/irsl_shm_libs')

import irsl_shm
import numpy as np
import time

sm_client = irsl_shm.ShmManager()
sm_client.settings().shm_key = 8888
sm_client.settings().hash = 8888
sm_client.openSharedMemory(False)

res = sm_client.isOpen()
print(res)

delta = 0.05

for i in range(100):
    pos = sm_client.readPositionCurrent()
    print(sm_client.getFrame(), pos)
    for j in range(len(pos)):
        if abs(pos[j]) <= delta:
            pos[j] = 0.0
        elif pos[j] > 0:
            pos[j] -= delta
        else:
            pos[j] += delta
    print(pos)
    ret = sm_client.writePositionCommand(pos)
    print(ret)
    time.sleep(0.01)
