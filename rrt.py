# -*- coding: utf-8 -*-
import numpy as np
import time
import random
import matplotlib
import matplotlib.pyplot as plt
import math
import copy
import curv


'''
函数：读取从bmp文件读取二值图像得到二值矩阵
输入：二值图文件路径
返回：二值矩阵
'''
def get_map(image):
    ima = matplotlib.image.imread(image)
    return ima


'''
函数：获取图像随机点
输入：无
返回：map区域内随机点r_x，r_y
'''
def random_point():
    r_x = random.random() * (m_map.shape[0] - 1)
    r_y = random.random() * (m_map.shape[1] - 1)
    return [r_x, r_y]


'''
函数：碰撞检测
输入：点p
返回：可达（True）；不可达（False）
'''
def feasible_point(p):
    # print(p[0],p[1])
    if 0 <= p[0] < m_map.shape[0] and 0 <= p[1] < m_map.shape[1] and m_map[int(p[0])][int(p[1])][0] == 255:
        return True
    else:
        return False


'''
函数：计算距离，这里采用曼哈顿距离
输入：点p，q
返回：曼哈顿距离
'''
def distance(p, q):
    d = abs(p[0] - q[0]) + abs(p[1] - q[1])  # 曼哈顿距离
    # d = math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)  # 欧式距离
    return d


'''
函数：寻找树tree上与点p的最近点
输入：点p，树tree
返回：树上离点p最近的点q，和q的行数row
'''
def nearest(p, tree):
    row, min_d = -1, float('INF')  # p在tree上最近点
    for i in range(len(tree)):
        p_n = [tree[i][0], tree[i][1]]
        td = distance(p_n, p)
        if td < min_d:
            min_d = td
            row = i
    p_near = [tree[row][0], tree[row][1]]
    return p_near, row


'''
函数：p朝q生长一个步长
输入：点p，点q
返回：生长的新点p_new
'''
def extend(p,q,step):
    if distance(p,q) < thresh_hold:
        return q
    else:
        theta = math.atan2((q[1] - p[1]), (q[0] - p[0]))  # p朝q生长
        p_new = [p[0] + step * math.cos(theta), p[1] + step * math.sin(theta)]
        return p_new


'''
函数：计算当前点到起始点的路径
输入：tree中点p对应下标i,树tree
返回：路径总长度cost
'''
def cost(i, tree):
    c = 0
    while True:
        t = tree[i][2]
        if t == -1:
            break
        c += distance(tree[i][:2], tree[t][:2])
        i = t
    return c


'''
函数：检查点p、q之间路径是否碰撞
输入：点p、q
返回：可行（True），不可行（False）
'''
def checkpath(p, q, step):
    if distance(p, q) < thresh_hold:
        return True
    theta = math.atan2((q[1] - p[1]), (q[0] - p[0]))  # p朝q生长
    t = copy.deepcopy(p)
    while distance(t, q) > thresh_hold:
        t = [t[0] + step * math.cos(theta), t[1] + step * math.sin(theta)]
        if not feasible_point(t):
            return False
    return True


'''
函数：剪枝优化
输入：点path
返回：剪枝后的new_path
'''
def pruning(path, step):
    new_path = [path[0]]
    i = 0
    j = 1
    while i < len(path):
        if checkpath(path[i], path[j], step):
            if distance(path[j], path[-1]) < thresh_hold:
                new_path.append(path[-1])
                break
            j += 1
        else:
            new_path.append(path[j-1])
            i = j-1
    new_cost = 0
    for i in range(len(new_path)-1):
        new_cost += distance(new_path[i], new_path[i+1])
    return new_path, new_cost


'''
函数：路径插值
输入：剪枝后路径path
返回：插值后路径i_path
'''
def interp(path, step):
    i_path = [path[0]]
    i = 0
    t = copy.deepcopy(path[0])
    while i < len(path)-1:
        theta = math.atan2((path[i+1][1] - path[i][1]), (path[i+1][0] - path[i][0]))  # p朝q生长
        while distance(t, path[i+1]) > thresh_hold:
            t = [t[0] + step * math.cos(theta), t[1] + step * math.sin(theta)]
            i_path.append(t)
        t = copy.deepcopy(path[i+1])
        i += 1
    return i_path


'''
函数：rrt主体
'''
def rrt(step, thresh_hold, init, goal):
    max_attempts = 100000
    tree = list()
    tree.append(init + [-1])
    flag = 0

    count = 0
    while count < max_attempts:
        if random.random() < 0.2:
            p_rand = goal
        else:
            p_rand = random_point()

        p_near, row = nearest(p_rand, tree)

        p_new = extend(p_near, p_rand,step)

        # if not feasible_point(p_new) or not check_path:
        if not feasible_point(p_new):
            count += 1
            continue
        elif distance(p_new, goal) <= thresh_hold:
            tree.append([goal[0], goal[1], row])
            flag = 1
            break
        else:
            p_new_near, n_row = nearest(p_new, tree)
            if distance(p_new, p_new_near) < thresh_hold:
                count += 1
                continue
            else:
                tree.append([p_new[0], p_new[1], row])

    if flag == 1:
        print("found path")
        path = []
        i = -1
        ni = tree[-1][2]
        while ni != -1:
            path.append([tree[i][0], tree[i][1]])
            i = ni
            ni = tree[i][2]
        path.append([init[0], init[1]])

        c = cost(len(tree)-1, tree)

        return path[::-1], tree, c
    else:
        print('reached max attempts')
        return tree


if __name__ == '__main__':
    m_map = get_map('map1.bmp')  # 调用图像预处理函数

    '''
    图像读出来的坐标轴和一般的坐标轴x、y是反的，所以要先转置
    但是plt.imshow(m_map)画出来的图像会把反的坐标轴再反过来
    所以最后画图时还要把图像的矩阵转置回去
    '''
    m_map = m_map.swapaxes(0, 1)

    # 初始参数
    init = [10, 10]
    goal = [490, 490]
    step = 15
    thresh_hold = 15

    # 获得初始路径
    if not feasible_point(init) or not feasible_point(goal):
        print('init or goal is not feasible')
    else:
        t0 = time.time()
        path, tree, cost = rrt(step, thresh_hold, init, goal)
        print('time: ', time.time()-t0)
        print('origin_cost: ', cost)
        x = [i[0] for i in path]
        y = [i[1] for i in path]
        p_tree, tx, ty = [], [], []
        for i in tree:
            tx.append(i[0])
            ty.append(i[1])
        plt.plot(x, y, color="blue")
        plt.scatter(tx, ty, color='lightsteelblue')

        # 剪枝优化
        new_path, new_cost = pruning(path, step)
        print('new_cost: ', new_cost)
        n_x = [i[0] for i in new_path]
        n_y = [i[1] for i in new_path]
        plt.plot(n_x, n_y, color="green")

        # 剪枝后插值
        i_path = interp(new_path, step)
        print(new_path)
        print(i_path)
        i_x = [i[0] for i in i_path]
        i_y = [i[1] for i in i_path]

        # 用插值后的点集做控制点生成B样条曲线
        c = curv.B_spline(i_x, i_y)
        c_x,  c_y = c.get_curv()
        # plt.plot(c_x, c_y, color="red")

        m_map = m_map.swapaxes(0, 1)
        plt.imshow(m_map)

        plt.show()
