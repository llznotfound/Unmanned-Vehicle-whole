import osmnx as ox
import networkx as nx
import csv
import math
import sys, getopt


def ox_shortest_path(midmap, origin, destination):
    """osmnx计算最短路径"""
    """midmap:地图中点经纬度wgs64  origin：起始点经纬度wgs64  destination:终点经纬度wgs64"""
    # city = ox.gdf_from_place('闵行区,Shanghai,China')
    # ox.plot_shape(ox.project_gdf(city))

    # ox.plot_graph(ox.graph_from_place("闵行区,上海市,中国"))

    G = ox.graph_from_point(midmap, distance=700, network_type='drive_service', \
                            simplify=False)  # 取消简化 即显示非相交的节点 为了获取弯道的多个节点
    # ox.plot_graph(G)

    # 接著我們給定原點跟目的地的坐標，然後計算其node的編號
    origin_node = ox.get_nearest_node(G, origin)
    destination_node = ox.get_nearest_node(G, destination)

    # 计算最短路径
    route = nx.shortest_path(G, origin_node, destination_node)
    # ox.plot_graph_route(G, route)
    distance = nx.shortest_path_length(G, origin_node, destination_node)
    # print(route)
    # print(len(route))
    # x = G.nodes[route[0]]  # 确定节点属性 'y': 31.0249842, 'x': 121.4355697, 'osmid': 1439718495, 'highway': 'crossing'}
    # path = [origin]
    path = []
    for i in range(len(route)):
        x = G.nodes[route[i]]['x']
        y = G.nodes[route[i]]['y']
        path.append((y, x))
    # path.append(destination)
    # print(path)
    # print(len(path))
    return path


def B2A_distance(latA, latB, lngA, lngB):
    """计算B点到A点的距离"""
    cc = math.sin(90 - latA) * math.sin(90 - latB) * math.cos(lngB - lngA) + math.cos(90 - latA) * math.cos(
        90 - latB)  # 求出cos（c）
    distance = 6371000 * math.acos(cc) * (math.pi / 180)  # 地球半径m * 弧度（acos（cos（c））* pi/180）

    return distance


def w2csv(path_lat, path_lng):
    """将计算好的路径点写入csv"""
    fileHeader = ["id", "lat", "lng"]

    csvFile = open("./planned_path.csv", "w", newline='')  # newline='' 取消空行
    # writer = csv.writer(csvFile)

    # #写入的内容都是以列表的形式传入函数
    # writer.writerow(fileHeader)
    # for i in range(len(broad.M1)):
    #     x = [i, broad.M1[i], broad.N2[i]]
    #     writer.writerow(x)
    # 写入的内容都是以字典的形式传入函数
    dict_writer = csv.DictWriter(csvFile, fileHeader)
    dict_writer.writeheader()
    for i in range(len(path_lat)):
        x = ({"id": i, "lat": path_lat[i], "lng": path_lng[i]})
        dict_writer.writerow(x)
    csvFile.close()


def sp(midmap, origin, destination):
    """处理osmnx收尾节点和起始点终点的优化问题"""
    path = ox_shortest_path(midmap, origin, destination)
    olat, olng = origin
    deslat, deslng = destination
    plat1, plng1 = path[0]
    plat2, plng2 = path[1]
    plat3, plng3 = path[-1]
    plat4, plng4 = path[-2]
    # 起点部分
    # 计算起点坐标到第二个节点（osmnx返回的最短路径节点数组不包含起始点终点坐标）的距离和第一个节点到第二个节点作比较
    start_d1 = B2A_distance(plat2, olat, plng2, olng)
    start_d2 = B2A_distance(plat2, plat1, plng2, plng1)
    # print(start_d1)
    # print(start_d2)
    # 若长则直接在头插入起点
    # 若短将第一个节点替换成起点
    if start_d1 > start_d2:
        path.insert(0, origin)
    else:
        path[0] = origin

    # 终点
    # 计算倒数第二个节点到终点的距离和倒数第二个节点到最后一个点的距离
    des_d1 = B2A_distance(deslat, plat4, deslng, plng4)
    des_d2 = B2A_distance(plat3, plat4, plng3, plng4)
    # print(des_d1)
    # print(des_d2)
    # 若长则直接在最后添加终点
    # 若短则将最后一个节点替换成终点
    if des_d1 > des_d2:
        path.append(destination)
    else:
        path[-1] = destination

    return path


def arg_change(arg):
#    a, b = arg.split(',')
    a = arg[0]
    b = arg[1]
    return (float(a), float(b))


def main(argv):

    #mlat = mlng = olat = olng = dlat = dlng = 1.11
    # 命令行执行.py文件 的预处理
    try:
        opts, args = getopt.getopt(argv, "hm:n:o:p:d:e:", ["midmaplat=", "midmaplng=", "originlat=", "originlng=", "destinationlat=", "destinationlng="])
    except getopt.GetoptError:
        print('osmnx_test.py -m <midmaplat> -n <midmaplng> -o <originlat> -p <originlng> -d <destinationlat> -e <destinationlng>')
        sys.exit(2)
    for opt, arg in opts:
        # print(opts)
        # print(arg)
        # print(opt)
        if opt == '-h':
            print('osmnx_test.py -m <midmaplat> -n <midmaplng> -o <originlat> -p <originlng> -d <destinationlat> -e <destinationlng>')
            sys.exit()
        elif opt in ('-m', '--midmaplat'):
            mlat = float(arg)
        elif opt in ('-n', '--midmaplng'):
            mlng = float(arg)
        elif opt in ('-o', '--originlat'):
            olat = float(arg)
        elif opt in ('-p', '--originlng'):
            olng = float(arg)
        elif opt in ('-d', '--destinationlat'):
            dlat = float(arg)
        elif opt in ('-e', '--destinationlng'):
            dlng = float(arg)
    
    # origin = (31.0259956642637,       121.43515693151937)
    # destination = (31.03037729233816, 121.44042251662931)
    # midmap = (31.028714411736857, 121.43916318394845)
    # path = ox_shortest_path(midmap, origin, destination)
    midmap = (mlat, mlng)
    origin = (olat, olng)
    destination = (dlat, dlng)
    
    #print(type(mlat))
    #print(mlat)
    #print(midmap)
    #print(type(midmap))
    spath = sp(midmap, origin, destination)

    w2csv([i for i, j in spath], [j for i, j in spath])

    print(len(spath))
    print(spath)


if __name__ == '__main__':
    main(sys.argv[1:])
