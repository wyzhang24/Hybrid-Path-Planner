import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

car_len, car_wid = 1, 0.5
dir_len = 1
with open("hybrid_a_star_path4.txt") as f:
  lineList = f.read().split()
with open("rrt_connect_ob4.txt") as f:
  obList = f.read().split()

list_pos = np.array(list(map(float, lineList))).reshape(-1,4)
list_ob = np.array(list(map(float, obList))).reshape(-1,2)
#list_pos = list_pos[::-1]
fig, ax = plt.subplots(1,1)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
patch = patches.Rectangle((list_pos[0,0],list_pos[0,1]),car_len, car_wid,fc ='y',angle = np.rad2deg(list_pos[0,2]))
line, = ax.plot([0,0], [0,0], color='k', linewidth=2)
#line, = ax.plot([160,180],[160,160])
def cal_transform(vec, ang):
    vec = np.array(vec)
    vec = vec.reshape(-1,1)

    rot_mat = np.zeros((2,2))

    rot_mat[0,0] = np.cos(ang)
    rot_mat[0,1] = -np.sin(ang)
    rot_mat[1,0] = np.sin(ang)
    rot_mat[1,1] = np.cos(ang)

    return np.dot(rot_mat, vec)


def init():
    ax.add_patch(patch)
    ax.add_line(line)
    return patch, line,


def animate(i):
    patch.set_width(car_len)
    patch.set_height(car_wid)

    angle = list_pos[i,2]
    # if list_pos[i,3] == 0:
    #     angle = list_pos[i,2] + np.pi
    # else:
    #     angle = list_pos[i,2]

    pos_list = cal_transform([-car_len/2, -car_wid/2], angle)
    patch.set_xy([list_pos[i, 0] + pos_list[0], list_pos[i, 1] + pos_list[1]])

    ang_list = cal_transform([dir_len, 0], angle)

    line.set_data([list_pos[i,0], list_pos[i,0]+ang_list[0]], [list_pos[i,1], list_pos[i,1]+ang_list[1]])


    patch.angle = np.rad2deg(angle)
    #print(np.rad2deg((list_pos[i,2])))
    return patch, line,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(list_pos),
                               interval=400,
                               blit=True)


plt.plot(list_pos[:,0], list_pos[:,1], ".")
plt.plot(list_ob[:,0], list_ob[:,1], ".")
plt.plot(list_pos[0,0], list_pos[0,1], "o", color='blue', linewidth=5)
plt.plot(list_pos[-1,0], list_pos[-1,1], "X", color ='red', linewidth=5)

# # save the video files.
# # Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
# anim.save('eg_video5.mp4', writer=writer)

plt.figure()
# plt.plot(list_pos1[:,0], list_pos1[:,1], ".")
plt.plot(list_pos[:,0], list_pos[:,1], ".")
plt.plot(list_ob[:,0], list_ob[:,1], ".")
plt.plot(list_pos[0,0], list_pos[0,1], "o", color='blue', linewidth=5)
plt.plot(list_pos[-1,0], list_pos[-1,1], "X", color ='red', linewidth=5)

plt.show()



# x = [0, 1, 2]
# y = [0, 1, 2]
# yaw = [0.0, 1.44, 1.3]
# fig = plt.figure()
# plt.axis('equal')
# plt.grid()
# ax = fig.add_subplot(111)
# ax.set_xlim(-10, 10)
# ax.set_ylim(-10, 10)
#
# patch = patches.Rectangle((0, 0), 0, 0, angle=0)
#
# def init():
#     ax.add_patch(patch)
#     return patch,
#
# def animate(i):
#     patch.set_width(5)
#     patch.set_height(1.0)
#     patch.set_xy([x[i], y[i]])
#     patch.angle = -np.rad2deg(yaw[i])
#
#     return patch,
#
# anim = animation.FuncAnimation(fig, animate,
#                                init_func=init,
#                                frames=len(x),
#                                interval=500,
#                                blit=True)
# plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)
# r = patches.Rectangle((0.5, 1.0), 2, 1, angle=45.0)
# ax.add_patch(r)
# ax.set_xlim(0, 3)
# ax.set_ylim(0, 3)
# ax.set_aspect('equal')
# plt.show()
