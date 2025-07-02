#pragma once
#ifndef MICROSTRUGENERATE_H
#define MICROSTRUGENERATE_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <random>
#include "tool.h"
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


 //   边的信息只保存一个vector，里面是边上点的序列
 
    struct Connection {
        int pointId1;
        int pointId2;
        int centerId;
        ConnectionType type; // 连线的类型
        double width;     // 线宽

        Connection(int p1, int p2, int p3, ConnectionType t, double w)
            : pointId1(p1), pointId2(p2), centerId(p3), type(t), width(w) {}
    };

    class QuarterCell {   //左下四分之一单元
    public:

        enum Edge { LEFT =0, TOP, RIGHT, BOTTOM  };
    private:
        std::vector<std::vector<double>> edgePointsRatios; // 每条边上的点的位置比例
        std::vector<Eigen::Vector2d> fout_corners = { {0.0,0.0}, {0.0, 200.0}, {200.0,200.0}, {200.0, 0.0} }; // 固定初始化值
        std::vector<int> edgePointsNums;
        std::vector<std::vector<PointTang>> edgePoints; // 每条边上的点，分别是左，上，右，下
        std::vector<PointTang > AllPoints; // 边界上的所有点
        std::vector<Connection> connections;               // 点之间的连接
        vector<vector<Eigen::Vector2d>> connectLines;
        vector<vector<Eigen::Vector2d>> connectPolys;
        int pointCounter = 0;                              // 点编号计数器

    public: 

        QuarterCell() {};
        
        QuarterCell(std::vector<PointTang> allPoints_, std::vector<Connection> connections_) :AllPoints(std::move(allPoints_)), connections(std::move(connections_)) {};
        
        QuarterCell(std::vector<std::vector<PointTang>> edgePoints_, std::vector<Connection> connections_);
        QuarterCell(std::vector<std::vector<double>> edgePointsRatios_, std::vector<std::vector<Eigen::Vector2d>> Tangents, std::vector<Connection> connections_);

        // 获取某条边的点
        const std::vector<PointTang>& getEdgePoints(Edge edge) const {
            return edgePoints.at(edge);
        }

        // 添加连接（点ID之间）
        void addConnection(int id1, int id2, int cenid, const ConnectionType type, double width) {
            connections.emplace_back(id1, id2, cenid, type, width);
        }

        // 获取所有连接
        const std::vector<Connection>& getConnections() const {
            return connections;
        }

        // 获取所有点
        const std::vector<PointTang>& getAllPoints() const {
            return AllPoints;
        }

        //根据信息构建cell的完整像素图信息
        void createCell();

        void drawCell();
    };

} // namespace msGen


#endif // MICROSTRUGENERATE_H