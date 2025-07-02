#include "microStruGenerate.h"

#include <algorithm>  // std::max_element
#include <cmath>      // std::round, std::abs
#include <stdexcept>  // std::runtime_error
#include <iostream>   // 调试时候可以打印日志

namespace msGen {

    QuarterCell::QuarterCell(std::vector<std::vector<PointTang>> edgePoints_, std::vector<Connection> connections_) :edgePoints(std::move(edgePoints_)), connections(std::move(connections_))
    {
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

    QuarterCell::QuarterCell(std::vector<std::vector<double>> edgePointsRatios_, std::vector<std::vector<Eigen::Vector2d>> Tangents, std::vector<Connection> connections_) :connections(std::move(connections_))
    {
        for (int i = 0; i < 4; i++)
        {
            int pCounter = 0;
            std::vector<PointTang> edges_p;
            for (int j = 0; j < edgePointsRatios_[i].size(); j++)
            {
                double ratio_ = edgePointsRatios_[i][j];
                Eigen::Vector2d point_ = ratio_ * (fout_corners[(i + 1) % 4] - fout_corners[i]);
                Eigen::Vector2d tangent_ = Tangents[i][j];
                PointTang pt(point_, tangent_);
                pCounter++;
                AllPoints.push_back(pt);
                edges_p.push_back(pt);
            }
            edgePointsNums.push_back(pCounter);
            edgePoints.push_back(edges_p);
            pointCounter += pCounter;
        }
    }

    void QuarterCell::createCell()
    {
        //遍历connections里的所有连接信息，构造所有曲线
        int connet_num = connections.size();
        vector<vector<Eigen::Vector2d>> AllLines;
        vector<vector<Eigen::Vector2d>> AllPolys;

        for (int i = 0; i < connet_num; i++)
        {
            vector<Eigen::Vector2d> linePoints;
            Connection line = connections[i];
            PointTang endp1 = AllPoints[line.pointId1];
            PointTang endp2 = AllPoints[line.pointId2];
            Vector2d cen_p = fout_corners[line.centerId];
            ConnectionType linetype = line.type;
            double width = line.width;
            if (linetype == MODE_RANDOM)
            {
                std::vector<ConnectionType> validTypes = { MODE_LINEAR, MODE_BSPLINE, MODE_HERMITE };
                static std::random_device rd;      // 随机设备种子
                static std::mt19937 engine(rd());  // Mersenne Twister引擎
                std::uniform_int_distribution<size_t> dist(0, validTypes.size() - 1);
                size_t randomIndex = dist(engine);
                linetype = validTypes[randomIndex];
            }
            switch (linetype) {
            case MODE_LINEAR: {
                linePoints = { endp1.point, endp2.point };
                break;
            }
            case MODE_BSPLINE: {
                double segnum = 20;
                cout << endp1.point << endp2.point << endp1.tangent << endp2.tangent << endl;
                linePoints = HermiteCurve(endp1.point, endp2.point, endp1.tangent, endp2.tangent, segnum).curvePoints;
                break;
            }
            case MODE_HERMITE: {
                double segnum = 20;
                cout << endp1.point << endp2.point << endp1.tangent << endp2.tangent << endl;
                linePoints = HermiteCurve(endp1.point, endp2.point, endp1.tangent, endp2.tangent, segnum).curvePoints;
            }
            default:
                // 返回空路径或原始控制点
                linePoints = vector<Eigen::Vector2d>();
            }
            AllLines.push_back(linePoints);
            AllPolys.push_back(expandLineRadial(linePoints, cen_p, width));
            connectLines = AllLines;
            connectPolys = AllPolys;
            //AllPoints
            //std::vector<Connection> connections;               // 点之间的连接
        }
    }


    void QuarterCell::drawCell()
    {
        cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        draw_contour(image, eigen2cv(fout_corners), Point2f(50, 50), Scalar(0, 0, 0));
        for (int i = 0; i < connectPolys.size(); i++)
            draw_poly(image, eigen2cv(connectPolys[i]), Point2f(50, 50), Scalar(0, 0, 0));

        cv::imshow("Cell connectPolys", image);
    }
    //class Microstructure
    //{
    //public:
    //  
    //    const std::vector<std::vector<int>>& getMicrostructure() const;

    //    /**
    //     * @brief 获取弹性张量（示例中以一维向量存储，可自定义为矩阵或其他格式）。
    //     * @return 返回 m_elasticTensor，例如 [C11, C22, C33, C12]。
    //     */
    //    const std::vector<double>& getElasticTensor() const;

    //    /**
    //     * @brief 获取热导率张量（示例中以一维向量存储，假设 2×2）。
    //     * @return 返回 m_thermalConductivityTensor，例如 [k11, k22, k12, k21]。
    //     */
    //    const std::vector<double>& getThermalConductivityTensor() const;
    //    std::vector<double> m_elasticTensor;

    //    // 热导率张量（示例：2×2，共 4 个分量 [k11, k22, k12, k21]）
    //    std::vector<double> m_thermalConductivityTensor;

    //private:
    //    /**
    //     * @brief 计算弹性张量 (示例，可替换为实质的力学均质化算法)
    //     */
    //    void computeElasticTensor();

    //    /**
    //     * @brief 计算热导率张量 (示例，可替换为实质的热传导均质化算法)
    //     */
    //    void computeThermalConductivityTensor();

    //};

}