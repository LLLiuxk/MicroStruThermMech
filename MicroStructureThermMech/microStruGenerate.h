#pragma once
#ifndef MICROSTRUGENERATE_H
#define MICROSTRUGENERATE_H

#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std;

enum ConnectionType {
    MODE_NONE = 0,
    MODE_LINEAR,
    MODE_BSPLINE,
    MODE_HERMITE,
    MODE_RANDOM,
    // ... 其他模式
};

namespace msGen {

    //带切向的点：
	struct PointTang
	{
		Eigen::Vector2d point;
		Eigen::Vector2d tangent;
		double ratio;

		PointTang() {}
		
		PointTang(Eigen::Vector2d p, Eigen::Vector2d t, double r = 1.0f)
			: point(p)
			, tangent(t)
			, ratio(r)
		{
			tangent *= ratio;
		}
		
		void setTangRatio(double r)
		{
			ratio = r;
			tangent *= ratio;
		}

		void setDegree(double tangle, int type = 0) //type =  0 :degree   type =1: radian
		{
			if (type == 0)
			{
				//if (tangle < 0 && tangle >360)
					//std::cout << "Input type error: It should be a DEGREE!" << std::endl;
				double tranS = 1.0f / 180.0f * std::acos(-1);
				tangent = Eigen::Vector2d(std::cosf(tangle * tranS), std::sinf(tangle * tranS)) * ratio;
			}
			else
			{
				//if (tangle < 0 && tangle >2 * std::cos(-1))
					//std::cout << "Input type error: It should be a RADIAN!" << std::endl;
				tangent = Eigen::Vector2d(std::cosf(tangle), std::sinf(tangle)) * ratio;
			}
		}
		double getDegree(int type = 0) //type =  0 :degree   type =1: radian
		{
			double angle = std::atan2f(tangent.y(), tangent.x());
			if (angle < 0)
				angle += std::acos(-1) * 2;
			if(type == 0)
				return angle * 180.0f / std::acos(-1);
			else return angle;
		}
	};


 //   // 结构体：边的信息
 //   struct Edge {
 //       std::vector<Eigen::Vector2d> points;  // 当前边上的点序列
 //       ConnectionType connectionType;        // 连接类型
 //   };


 //   std::vector<std::vector<int>> generateMicrostructure(
 //       int leftPointCount,
 //       int topPointCount,
 //       const std::vector<double>& leftPositions,
 //       const std::vector<double>& topPositions,
 //       double connectionWidthLeft,
 //       double connectionWidthTop,
 //       int connectionModeLeft,
 //       int connectionModeTop
 //   );

} // namespace msGen





//// 拓扑连接信息的结构体（可扩展）
//struct EdgeConnection {
//    int edge1;         // 第一个边的索引
//    int edge2;         // 第二个边的索引
//    std::string note;  // 可选的说明，比如"shared corner"等
//};
//
//class QuarterCell {
//public:
//    QuarterCell();
//
//    // 添加一条边
//    void addEdge(const std::vector<Eigen::Vector2d>& pts, ConnectionType type);
//
//    // 添加边之间的拓扑连接
//    void addConnection(int edge1, int edge2, const std::string& note = "");
//
//    // 获取边的数量
//    int getEdgeCount() const;
//
//    // 显示信息（调试用）
//    void printInfo() const;
//
//private:
//    std::vector<Edge> edges;                      // 所有边
//    std::vector<EdgeConnection> edgeConnections;  // 拓扑连接信息
//};
//


#endif // MICROSTRUGENERATE_H