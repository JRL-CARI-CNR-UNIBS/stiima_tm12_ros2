import matplotlib.pyplot as plt
import numpy
import sys
import csv


fold = sys.argv[1]

act_time = []
act_jnt0 = []
act_jnt1 = []
act_jnt2 = []
act_jnt3 = []
act_jnt4 = []
act_jnt5 = []

act_vel0 = []
act_vel1 = []
act_vel2 = []
act_vel3 = []
act_vel4 = []
act_vel5 = []

trg_time = []
trg_jnt0 = []
trg_jnt1 = []
trg_jnt2 = []
trg_jnt3 = []
trg_jnt4 = []
trg_jnt5 = []

with open(fold + "jnt_actual_.txt", 'r') as f:
      reader=csv.reader(f , delimiter=' ')
      for item in reader:
        act_time.append(float(item[0]))
        act_jnt0.append(float(item[1]))
        act_vel0.append(float(item[2]))
        act_jnt1.append(float(item[4]))
        act_vel1.append(float(item[5]))
        act_jnt2.append(float(item[7]))
        act_vel2.append(float(item[8]))
        act_jnt3.append(float(item[10]))
        act_vel3.append(float(item[11]))
        act_jnt4.append(float(item[13]))
        act_vel4.append(float(item[14]))
        act_jnt5.append(float(item[17]))
        act_vel5.append(float(item[18]))

with open(fold + "jnt_target_.txt", 'r') as f:
      reader=csv.reader(f , delimiter=' ')
      for item in reader:
        trg_time.append(float(item[0]))
        trg_jnt0.append(float(item[1]))
        trg_jnt1.append(float(item[2]))
        trg_jnt2.append(float(item[3]))
        trg_jnt3.append(float(item[4]))
        trg_jnt4.append(float(item[5]))
        trg_jnt5.append(float(item[6]))


fig, axs = plt.subplots(6,sharex=True)
fig.suptitle('pos')
axs[0].scatter(act_time,act_jnt0,label="act")
axs[0].scatter(trg_time,trg_jnt0,label="trg")
axs[1].scatter(act_time,act_jnt1,label="act")
axs[1].scatter(trg_time,trg_jnt1,label="trg")
axs[2].scatter(act_time,act_jnt2,label="act")
axs[2].scatter(trg_time,trg_jnt2,label="trg")
axs[3].scatter(act_time,act_jnt3,label="act")
axs[3].scatter(trg_time,trg_jnt3,label="trg")
axs[4].scatter(act_time,act_jnt4,label="act")
axs[4].scatter(trg_time,trg_jnt4,label="trg")
axs[5].scatter(act_time,act_jnt5,label="act")
axs[5].scatter(trg_time,trg_jnt5,label="trg")
axs[0].legend()

fig2, axs2 = plt.subplots(6,sharex=True)
fig2.suptitle('vel')
axs2[0].scatter(act_time,act_vel0,label="act")
axs2[1].scatter(act_time,act_vel1,label="act")
axs2[2].scatter(act_time,act_vel2,label="act")
axs2[3].scatter(act_time,act_vel3,label="act")
axs2[4].scatter(act_time,act_vel4,label="act")
axs2[5].scatter(act_time,act_vel5,label="act")
plt.show()

