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
    r_x = int(random.random() * (m_map.shape[0] - 1))
    r_y = int(random.random() * (m_map.shape[1] - 1))
    return [r_x, r_y]


'''
函数：碰撞检测
输入：点p
返回：可达（True）；不可达（False）
'''
def feasible_point(p):
    # print(p[0],p[1])
    if p[0] < m_map.shape[0] and p[1] < m_map.shape[1] and m_map[int(p[0])][int(p[1])][0] == 255:
        return True
    else:
        return False


'''
函数：计算距离，这里采用曼哈顿距离
输入：点p，q
返回：曼哈顿距离
'''
def distance(p, q):
    d = abs(p[0] - q[0]) + abs(p[1] - q[1])
    return d


'''
函数：寻找树tree上与点p的最近点
输入：点p，树tree
返回：树上离点p最近的点q，和q的行数row
'''
def nearest(p_rand, tree):
    row, min_d = -1, float('INF')  # p_rand在tree0上最近点
    for i in range(len(tree)):
        p_n = [tree[i][0], tree[i][1]]
        td = distance(p_n, p_rand)
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
    while i < len(path) - 2:
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
函数：rrt_connect主体
'''
def rrt_connect(step, thresh_hold, init, goal):
    max_attempts = 100000
    tree0 = list()
    tree0.append(init + [-1])
    tree1 = list()
    tree1 .append(goal + [-1])
    flag = 0

    count = 0
    while count < max_attempts:
        if random.random() < 0:  # 随机点
            p_rand = goal
        else:
            p_rand = random_point()

        p_near0, row0 = nearest(p_rand, tree0)  # p_rand在tree0上最近点

        p_new0 = extend(p_near0, p_rand, step)  # p_near0朝p_rand生长

        if not feasible_point(p_new0):
            count += 1
            if len(tree1) < len(tree0):
                tree0, tree1 = tree1, tree0
            continue
        else:
            tree0.append([p_new0[0], p_new0[1], row0])

            p_near1, row1 = nearest(p_new0, tree1)  # p_new0在tree1上最近点

            p_new1 = extend(p_near1, p_new0, step)  # p_near1朝p_new0生长

            if not feasible_point(p_new1):
                count += 1
                if len(tree1) < len(tree0):
                    tree0, tree1 = tree1, tree0
                continue
            else:
                tree1.append([p_new1[0], p_new1[1], row1])

                p_new_new1 = extend(p_new1, p_new0, step)  # p_new1朝p_new0生长

                if not feasible_point(p_new_new1):
                    count += 1
                    if len(tree1) < len(tree0):
                        tree0, tree1 = tree1, tree0
                    continue
                else:
                    tree1.append([p_new_new1[0], p_new_new1[1], len(tree1)-1])
                    p_new1 = p_new_new1

                    while distance(p_new1, p_new0) > thresh_hold:
                        p_new_new1 = extend(p_new1, p_new0, step)  # p_new1朝p_new0生长
                        if not feasible_point(p_new_new1):
                            count += 1
                            break
                        else:
                            tree1.append([p_new_new1[0], p_new_new1[1], len(tree1)-1])
                            p_new1 = p_new_new1

                    if distance(p_new1, p_new0) <= thresh_hold:
                        tree1.append([p_new0[0], p_new0[1], len(tree1)-1])
                        flag = 1
                        break

            if len(tree1) < len(tree0):
                tree0, tree1 = tree1, tree0

    if flag == 1:
        print("found path")

        path0 = []
        i = -1
        ni = tree0[-1][2]
        while ni != -1:
            path0.append([tree0[i][0], tree0[i][1]])
            i = ni
            ni = tree0[i][2]
        path0.append([tree0[0][0], tree0[0][1]])

        path1 = []
        i = -1
        ni = tree1[-1][2]
        while ni != -1:
            path1.append([tree1[i][0], tree1[i][1]])
            i = ni
            ni = tree1[i][2]
        path1.append([tree1[0][0], tree1[0][1]])

        return path0, path1, tree0, tree1

    else:
        print('reached max attempts')


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
        path0, path1, tree0, tree1 = rrt_connect(step, thresh_hold, init, goal)
        print('time: ', time.time()-t0)

        x0 = [i[0] for i in path0]
        y0 = [i[1] for i in path0]
        plt.plot(x0, y0, color="darkred")

        x1 = [i[0] for i in path1]
        y1 = [i[1] for i in path1]
        plt.plot(x1, y1, color="darkblue")

        t0x, t0y = [], []
        for i in tree0:
            t0x.append(i[0])
            t0y.append(i[1])
        t1x, t1y = [], []
        for i in tree1:
            t1x.append(i[0])
            t1y.append(i[1])
        plt.scatter(t0x, t0y, color='lightcoral')
        plt.scatter(t1x, t1y, color='lightsteelblue')

        # 合并path0，path1
        if path0[-1] == init:
            path = path0[::-1][:-1] + path1
        else:
            path = path1[::-1][:-1] + path0
        cost = 0
        for i in range(len(path) - 1):
            cost += distance(path[i], path[i + 1])
        print('cost: ', cost)

        # 剪枝优化
        new_path, new_cost = pruning(path, step)
        print('new_cost: ', new_cost)
        n_x = [i[0] for i in new_path]
        n_y = [i[1] for i in new_path]
        plt.plot(n_x, n_y, color="green")

        # 剪枝后插值
        i_path = interp(new_path, step)
        i_x = [i[0] for i in i_path]
        i_y = [i[1] for i in i_path]

        # 用插值后的点集做控制点生成B样条曲线
        c = curv.B_spline(i_x, i_y)
        c_x, c_y = c.get_curv()
        # plt.plot(c_x, c_y, color="purple")

        m_map = m_map.swapaxes(0, 1)
        plt.imshow(m_map)

        plt.show()
