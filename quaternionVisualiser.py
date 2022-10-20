from hashlib import new
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
# Fixing random state for reproducibility
np.random.seed(19680801)

# Assumes v and u are np arrays
def vector_dot(v,u):
    return np.dot(v,u)
    
def vector_cross(v,u):
    return np.cross(v,u)

def qv_mult(q,p):
    w1,x1,y1,z1 = p
    w2,x2,y2,z2 = q
    v1 = np.array([x1,y1,z1])
    v2 = np.array([x2,y2,z2])
    vector = w2*v1 + w1*v2 + vector_cross(v1,v2)
    x3, y3, z3 = vector
    ret = [w1*w2-vector_dot(v1,v2), x3,y3,z3]
    magnitude = pow(ret[0], 2) + pow(ret[1], 2) + pow(ret[2],2) + pow(ret[3],2)
    ret = ret * 1/magnitude
    return ret

def q_inv(q):

    w,x,y,z = q
    return np.array([w, -x, -y, -z])

def random_walk(num_steps, max_step=0.05):
    """Return a 3D random walk as (num_steps, 3) array."""
    start_pos = np.random.random(3)
    steps = np.random.uniform(-max_step, max_step, size=(num_steps, 3))
    walk = start_pos + np.cumsum(steps, axis=0)
    return walk


def update_lines(num, walks, lines):
    for line, walk in zip(lines, walks):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(walk[:num, :2].T)
        line.set_3d_properties(walk[:num, 2])
    return lines


f = open("./log_data/14-10-22-17-21-33.csv")
reader_obj = csv.reader(f)

v = np.array([0,1,0,0])
u = np.array([0,0,1,0])
# print(vector_dot(v,u))
# print(vector_cross(v,u))
# print(qv_mult(v,u))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim([-2,2])
ax.set_ylim([-2,2])
ax.set_zlim([-2,2])

xlin = np.linspace(0,1,10)
ylin = np.linspace(0,0,10)
zlin = np.linspace(0,0,10)
(line,) = ax.plot3D(xlin,ylin,zlin,'gray')
plt.show()
oldVector = np.array([0,0,0,1])
i = 0
for row in reader_obj:
    try:
        float(row[0])
    except:
        continue
    
    w = float(row[1])
    x = float(row[2])
    y = float(row[3])
    z = float(row[4])
    q = np.array([w,x,y,z])
    qi = q_inv(q)
    # print(q)
    # print(qi)
    newq = qv_mult(qv_mult(q,oldVector), qi)
    # Grab end of vector from newq
    w, nx, ny, nz = newq
    xlin = np.linspace(0,nx,10)
    ylin = np.linspace(0,ny,10)
    zlin = np.linspace(0,nz,10)
    # ax.set_xdata
    # ax.set('xdata', xlin)
    # ax.set('ydata', ylin)
    # ax.set('zdata', zlin)
    line.set_xdata(xlin)
    line.set_ydata(ylin)
    # line.set_zdata(zlin)
    plt.show()
    print(newq)
    
    oldVector = newq
    

# Setting the axes properties

# # Creating the Animation object
# ani = animation.FuncAnimation(
#     fig, update_lines, num_steps, fargs=(walks, lines), interval=100)

# plt.show()


# Multiply quaternion q by quaternion p

    

# import matplotlib.pyplot as plt
# import numpy as np

# def main():
#     u = np.array([1,0,0])
#     v = np.array([0,1,0])
#     q = np.array([0,0,1])

#     # x = [np.linspace(0,1,100),0,0]
#     # y = np.linspace()
#     fig = plt.figure()
#     ax = fig.axes
#     # ax = plt.axes(projection='3d')
#     # xvector =
#     ax.set_xlim([-2,2])
#     ax.set_ylim([-2,2])
#     ax.set_zlim([-2,2])

#     start=[0,0,0]
#     # xvector = ax.quiver(start[0],start[1],start[2], u[0], u[1], u[2], color='r')
#     # yvector = ax.quiver(start[0],start[1],start[2], v[0], v[1], v[2], color='g')
#     # zvector = ax.quiver(start[0],start[1],start[2], q[0], q[1], q[2], color='b')
    
#     x = np.linspace(0,0,50)
#     y = np.linspace(0,0,50)
#     z = np.linspace(0,1,50)

#     ax.plot(x,y,z, color=(1,0,0,1))

#     plt.show()
#     # print(xvector)

# main()

