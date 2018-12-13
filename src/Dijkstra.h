#ifndef Dijkstra_H 
#define Dijkstra_H 

//#include "stdafx.h"
//#include<cstdio>
//#include<algorithm>
//#include<cmath>
//#include<vector>
//#include<queue>
//#include<iostream>
//#include<functional>
//using namespace std;
//const int maxn = 1e3;
//const int INF = 1e9;
////int G[maxn][maxn];//邻接表表示
//int dis[maxn];//最短路
//int vis[maxn] = { false };//标记顶点v是否已被访问
//int vertex, edge, start;//顶点，边，起点
//int pre[maxn];//pre[i]=u代表结点i的前驱为u
//int num[maxn] = { 0 };//记录最短路径条数
//struct node {
//	int v;
//	int routelen;
//};
//vector<node> adjacent[maxn];//邻接矩阵表示
//void dijkstra(int start) {//Dijkstra算法
//	fill(dis, dis + maxn, INF);//最短路径初始化为无穷
//	fill(pre, pre + maxn, -1);//初始化前驱
//	dis[start] = 0;
//	pre[start] = start;
//	num[start] = 1;
//	for (int i = 0; i < vertex; i++) {//循环n次
//		int u = -1, min = INF;
//		for (int j = 0; j < vertex; j++) {//寻找还未被访问顶点的最短路
//			if (dis[j] < min && vis[j] == false) {
//				u = j;
//				min = dis[j];
//			}
//		}
//		if (u == -1) return;//如果没找到，则return
//		vis[u] = true;//如果找到，则置u被访问
//		for (int j = 0; j < adjacent[u].size(); j++) {//从u出发能够到达的所有顶点
//			int v = adjacent[u][j].v;
//			if (vis[v] == false) {
//				if (dis[u] + adjacent[u][j].routelen < dis[v]) {//v未被访问&&以u为中介点能够使得dis[v]更短
//					dis[v] = dis[u] + adjacent[u][j].routelen;//更新路径
//					pre[v] = u;//更新前驱
//					num[v] = num[u];//更新路径条数
//				}
//				else if (dis[u] + adjacent[u][j].routelen == dis[v]) {//此处一定要用else if语句,不能用if,否则得到的num不正确
//					num[v] += num[u];//更新路径条数
//				}
//			}
//		}
//	}
//}
//void dfs(int a[], int k) {//递归输出最短路径
//	if (a[k] == k) {
//		printf("%d", k);
//	}
//	else if (a[k] == -1) {
//		printf("无到达此点的路径");
//	}
//	else {
//		dfs(a, a[k]);//递归输出前驱结点
//		printf("->%d", k);//输出本结点
//	}
//}
//int main() {
//	//fill(G[0], G[0] + maxn * maxn, INF);
//	scanf("%d%d%d", &vertex, &edge, &start);
//	int s, e, value;
//	for (int i = 0; i < edge; i++) {
//		int start, end, weight;
//		scanf("%d%d%d", &start, &end, &weight);//分别输入起点，终点，边权
//		node V;
//		V.v = end;//v是边的目标顶点
//		V.routelen = weight;//边权
//		adjacent[start].push_back(V);
//	}
//	dijkstra(start);
//	for (int i = 0; i < vertex; i++) {//输出以start为源点，到达各大顶点的最短距离
//		printf("%d ", dis[i]);
//	}
//	putchar('\n');
//	int k;
//	scanf("%d", &k);//输入目标顶点k
//	dfs(pre, k);//输出源点到目标顶点k的最短路径
//	printf("\n%d", num[k]);//输出源点到目标顶点的最短路径条数
//	return 0;
//}


#endif