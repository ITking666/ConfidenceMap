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
////int G[maxn][maxn];//�ڽӱ��ʾ
//int dis[maxn];//���·
//int vis[maxn] = { false };//��Ƕ���v�Ƿ��ѱ�����
//int vertex, edge, start;//���㣬�ߣ����
//int pre[maxn];//pre[i]=u������i��ǰ��Ϊu
//int num[maxn] = { 0 };//��¼���·������
//struct node {
//	int v;
//	int routelen;
//};
//vector<node> adjacent[maxn];//�ڽӾ����ʾ
//void dijkstra(int start) {//Dijkstra�㷨
//	fill(dis, dis + maxn, INF);//���·����ʼ��Ϊ����
//	fill(pre, pre + maxn, -1);//��ʼ��ǰ��
//	dis[start] = 0;
//	pre[start] = start;
//	num[start] = 1;
//	for (int i = 0; i < vertex; i++) {//ѭ��n��
//		int u = -1, min = INF;
//		for (int j = 0; j < vertex; j++) {//Ѱ�һ�δ�����ʶ�������·
//			if (dis[j] < min && vis[j] == false) {
//				u = j;
//				min = dis[j];
//			}
//		}
//		if (u == -1) return;//���û�ҵ�����return
//		vis[u] = true;//����ҵ�������u������
//		for (int j = 0; j < adjacent[u].size(); j++) {//��u�����ܹ���������ж���
//			int v = adjacent[u][j].v;
//			if (vis[v] == false) {
//				if (dis[u] + adjacent[u][j].routelen < dis[v]) {//vδ������&&��uΪ�н���ܹ�ʹ��dis[v]����
//					dis[v] = dis[u] + adjacent[u][j].routelen;//����·��
//					pre[v] = u;//����ǰ��
//					num[v] = num[u];//����·������
//				}
//				else if (dis[u] + adjacent[u][j].routelen == dis[v]) {//�˴�һ��Ҫ��else if���,������if,����õ���num����ȷ
//					num[v] += num[u];//����·������
//				}
//			}
//		}
//	}
//}
//void dfs(int a[], int k) {//�ݹ�������·��
//	if (a[k] == k) {
//		printf("%d", k);
//	}
//	else if (a[k] == -1) {
//		printf("�޵���˵��·��");
//	}
//	else {
//		dfs(a, a[k]);//�ݹ����ǰ�����
//		printf("->%d", k);//��������
//	}
//}
//int main() {
//	//fill(G[0], G[0] + maxn * maxn, INF);
//	scanf("%d%d%d", &vertex, &edge, &start);
//	int s, e, value;
//	for (int i = 0; i < edge; i++) {
//		int start, end, weight;
//		scanf("%d%d%d", &start, &end, &weight);//�ֱ�������㣬�յ㣬��Ȩ
//		node V;
//		V.v = end;//v�Ǳߵ�Ŀ�궥��
//		V.routelen = weight;//��Ȩ
//		adjacent[start].push_back(V);
//	}
//	dijkstra(start);
//	for (int i = 0; i < vertex; i++) {//�����startΪԴ�㣬������󶥵����̾���
//		printf("%d ", dis[i]);
//	}
//	putchar('\n');
//	int k;
//	scanf("%d", &k);//����Ŀ�궥��k
//	dfs(pre, k);//���Դ�㵽Ŀ�궥��k�����·��
//	printf("\n%d", num[k]);//���Դ�㵽Ŀ�궥������·������
//	return 0;
//}


#endif