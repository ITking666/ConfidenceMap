#ifndef ASTAR_H
#define ASTAR_H

#include <vector>  
#include <list>  

///************************************************************************///
// a class to implement the A* (called A star) algorithm
// A* - A star algorithm, the raw code is from
// https://github.com/lnp1993/Astar.git
// modified by Huang Pengdi, 2018.01.03

//Version 1.0 
// - raw version by the original author
//Version 2.0 2019.01.03
// - modified the class to adapt our system

///************************************************************************///
const int kCost1 = 10; //直移一格消耗  
const int kCost2 = 10; //斜移一格消耗  

struct GridNode
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列  
	int F, G, H; //F=G+H  
	GridNode *parent; //parent的坐标，这里没有用指针，从而简化代码  
	GridNode(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化  
	{
	}
};


class Astar
{
public:
	void InitAstar(std::vector<std::vector<int>> &_maze);
	std::list<GridNode *> GetPath(GridNode &startPoint, GridNode &endPoint, bool isIgnoreCorner);

private:
	GridNode *findPath(GridNode &startPoint, GridNode &endPoint, bool isIgnoreCorner);
	std::vector<GridNode *> getSurroundPoints(const GridNode *point, bool isIgnoreCorner) const;
	bool isCanreach(const GridNode *point, const GridNode *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断  
	GridNode *isInList(const std::list<GridNode *> &list, const GridNode *point) const; //判断开启/关闭列表中是否包含某点  
	GridNode *getLeastFpoint(); //从开启列表中返回F值最小的节点  
	//计算FGH值  
	int calcG(GridNode *temp_start, GridNode *point);
	int calcH(GridNode *point, GridNode *end);
	int calcF(GridNode *point);
private:
	std::vector<std::vector<int>> maze;
	std::list<GridNode *> openList;  //开启列表  
	std::list<GridNode *> closeList; //关闭列表  
};


#endif // !ASTAR_H