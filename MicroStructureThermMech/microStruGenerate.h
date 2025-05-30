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


 //   // �ṹ�壺�ߵ���Ϣ
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





//// ����������Ϣ�Ľṹ�壨����չ��
//struct EdgeConnection {
//    int edge1;         // ��һ���ߵ�����
//    int edge2;         // �ڶ����ߵ�����
//    std::string note;  // ��ѡ��˵��������"shared corner"��
//};
//
//class QuarterCell {
//public:
//    QuarterCell();
//
//    // ���һ����
//    void addEdge(const std::vector<Eigen::Vector2d>& pts, ConnectionType type);
//
//    // ��ӱ�֮�����������
//    void addConnection(int edge1, int edge2, const std::string& note = "");
//
//    // ��ȡ�ߵ�����
//    int getEdgeCount() const;
//
//    // ��ʾ��Ϣ�������ã�
//    void printInfo() const;
//
//private:
//    std::vector<Edge> edges;                      // ���б�
//    std::vector<EdgeConnection> edgeConnections;  // ����������Ϣ
//};
//


#endif // MICROSTRUGENERATE_H