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
    // ... ����ģʽ
};

namespace msGen {

    //������ĵ㣺
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


 //   �ߵ���Ϣֻ����һ��vector�������Ǳ��ϵ������
 

    class QuarterCell {
    public:
        enum Edge { TOP = 0, RIGHT, BOTTOM, LEFT };
        struct Connection {
            int pointId1;
            int pointId2;
            std::string type; // �ߵ����ͣ����� "solid", "dashed", "curve"
            double width;     // �߿�

            Connection(int p1, int p2, const std::string& t, double w)
                : pointId1(p1), pointId2(p2), type(t), width(w) {}
        };

    private:
        std::map<Edge, std::vector<PointTang>> edgePoints; // ÿ�����ϵĵ�
        std::vector<Connection> connections;               // ��֮�������
        int pointCounter = 0;                              // ���ż�����

    public:
        // ����ĳ�����ϵĵ㣨���Էֲ���
        void generateEdgePoints(Edge edge, int numPoints) {
            edgePoints[edge].clear();
            double step = 1.0 / (numPoints - 1);

            for (int i = 0; i < numPoints; ++i) {
                double x = 0.0, y = 0.0;
                switch (edge) {
                case TOP:    x = i * step; y = 1.0; break;
                case RIGHT:  x = 1.0; y = 1.0 - i * step; break;
                case BOTTOM: x = 1.0 - i * step; y = 0.0; break;
                case LEFT:   x = 0.0; y = i * step; break;
                }
                edgePoints[edge].emplace_back(x, y, pointCounter++);
            }
        }

        // ��ȡĳ���ߵĵ�
        const std::vector<PointTang>& getEdgePoints(Edge edge) const {
            return edgePoints.at(edge);
        }

        // ������ӣ���ID֮�䣩
        void addConnection(int id1, int id2, const std::string& type, double width) {
            connections.emplace_back(id1, id2, type, width);
        }

        // ��ȡ��������
        const std::vector<Connection>& getConnections() const {
            return connections;
        }

        // ��ȡ���е�
        std::vector<PointTang> getAllPoints() const {
            std::vector<PointTang> all;
            for (const auto& [edge, pts] : edgePoints) {
                all.insert(all.end(), pts.begin(), pts.end());
            }
            return all;
        }

        // ��ʾ��Ϣ�������ã�
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
 //       std::vector<Eigen::Vector2d> points;  // ��ǰ���ϵĵ�����
 //       ConnectionType connectionType;        // ��������
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