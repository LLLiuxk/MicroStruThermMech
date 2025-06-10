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


 //   边的信息只保存一个vector，里面是边上点的序列
 

    class QuarterCell {
    public:

        enum Edge { TOP = 0, RIGHT, BOTTOM, LEFT };
        struct Connection {
            int pointId1;
            int pointId2;
            ConnectionType type; // 连线的类型
            double width;     // 线宽

            Connection(int p1, int p2, ConnectionType t, double w)
                : pointId1(p1), pointId2(p2), type(t), width(w) {}
        };

    private:
        std::vector<int> edgePointsNums;
        std::vector<std::vector<PointTang>> edgePoints; // 每条边上的点
        std::vector<PointTang > AllPoints; // 边界上的所有点
        std::vector<Connection> connections;               // 点之间的连接
        int pointCounter = 0;                              // 点编号计数器

    public:

        QuarterCell() {};

        QuarterCell(std::vector<std::vector<PointTang>> edgePoints, std::vector<Connection> connections) {
            edgePoints = edgePoints;
            connections = connections;
            for (int i = 0; i < 4; i++)
            {
                int pCounter = 0;
                for (int j = 0; j < edgePoints[i].size(); j++)
                {
                    pCounter++;
                    AllPoints.push_back(edgePoints[i][j]);
                }
                edgePointsNums.push_back(pCounter);
                pointCounter += pCounter;
            }
               

        
        }



        // 获取某条边的点
        const std::vector<PointTang>& getEdgePoints(Edge edge) const {
            return edgePoints.at(edge);
        }

        // 添加连接（点ID之间）
        void addConnection(int id1, int id2, const std::string& type, double width) {
            connections.emplace_back(id1, id2, type, width);
        }

        // 获取所有连接
        const std::vector<Connection>& getConnections() const {
            return connections;
        }

        // 获取所有点
        std::vector<PointTang> getAllPoints() const {
            std::vector<PointTang> all;
            for (const auto& [edge, pts] : edgePoints) {
                all.insert(all.end(), pts.begin(), pts.end());
            }
            return all;
        }

        // 显示信息（调试用）
        void printSummary() const {
            std::cout << "Quarter Cell Summary:\n";
            for (const auto& [edge, pts] : edgePoints) {
                std::cout << "Edge " << edge << ": " << pts.size() << " points\n";
            }
            std::cout << "Total Connections: " << connections.size() << "\n";
            for (const auto& conn : connections) {
                std::cout << "Connect " << conn.pointId1 << " -> " << conn.pointId2
                    << ", Type: " << conn.type << ", Width: " << conn.width << "\n";
            }
        }
    };
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


#endif // MICROSTRUGENERATE_H