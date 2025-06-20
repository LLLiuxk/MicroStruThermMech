#include <iostream>
#include <vector>
#include <string>


#include "tool.h"
#include "microStruGenerate.h"
#include "propertyCalculate.h"
#include "Visualization.h"

// 归一化向量（单位向量）
Vector2d normVector(const Vector2d& v) {
    double length = std::sqrt(v.x() * v.x() + v.y() * v.y());
    if (length < 1e-10) return { 0, 0 }; // 避免除以零
    return { v.x() / length, v.y() / length };
}

// 计算点的偏移点
Vector2d offsetPoint(const Vector2d& p, const Vector2d& dir, double distance) {
    Vector2d off_P(p.x() + dir.x() * distance, p.y() + dir.y() * distance);
    return off_P;
}

std::pair<std::vector<Vector2d>, std::vector<Vector2d>> expandLineToWidth(
    const std::vector<Vector2d>& line, double width) {

    if (line.size() < 2) {
        // 线段至少需要2个点
        return { {}, {} };
    }

    std::vector<Vector2d> upperBoundary;
    std::vector<Vector2d> lowerBoundary;
    const double halfWidth = width / 2.0;

    // 处理第一个点
    Vector2d firp = line[1] - line[0];
    Vector2d firstDir(firp.y(), -firp.x());
    Vector2d firstUnitDir = normVector(firstDir);
    upperBoundary.push_back(offsetPoint(line[0], firstUnitDir, halfWidth));
    lowerBoundary.push_back(offsetPoint(line[0], firstUnitDir, -halfWidth));

    // 处理中间点
    for (size_t i = 1; i < line.size() - 1; i++) {
        Vector2d prevDir = line[i] - line[i - 1];
        Vector2d nextDir = line[i + 1] - line[i];

        // 计算两个相邻线段的平均方向
        Vector2d avgDir(prevDir.x() + nextDir.x(), prevDir.y() + nextDir.y());

        // 计算垂直方向并归一化
        Vector2d perp(avgDir.y(), -avgDir.x());
        Vector2d unitPerp = normVector(perp);

        // 计算偏移点
        upperBoundary.push_back(offsetPoint(line[i], unitPerp, halfWidth));
        lowerBoundary.push_back(offsetPoint(line[i], unitPerp, -halfWidth));
    }

    // 处理最后一个点
    Vector2d lastp = line[line.size() - 1] - line[line.size() - 2];
    Vector2d lastDir(lastp.y(), -lastp.x());
    Vector2d lastUnitDir = normVector(lastDir);
    upperBoundary.push_back(offsetPoint(line.back(), lastUnitDir, halfWidth));
    lowerBoundary.push_back(offsetPoint(line.back(), lastUnitDir, -halfWidth));

    return { upperBoundary, lowerBoundary };
}

int main()
{
    // 设置精度和科学计数法显示
    std::cout << std::fixed << std::setprecision(6);  // 固定小数，保留 4 位
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    // 示例：x轴上的直线 (y=0)
    std::vector<Vector2d> line = { {0, 0}, {20,0}, { 50, 0 }, {100, 0} };
    double width = 20.0; // 宽度

   /* std::pair<std::vector<Vector2d>, std::vector<Vector2d>> two_v*/auto two_v = expandLineToWidth(line, width);

    // 打印结果
   std::vector<Vector2d>& upper = two_v.first;
   std::vector<Vector2d>& lower = two_v.second;

    std::cout << "Upper Boundary:\n";
    for (const auto& p : upper) {
        std::cout << "(" << p.x() << ", " << p.y() << ")\n";
    }

    std::cout << "\nLower Boundary:\n";
    for (const auto& p : lower) {
        std::cout << "(" << p.x() << ", " << p.y() << ")\n";
    }

    cv::Mat image(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));

    draw_lines(image, eigen2cv(upper), Point2f(50,50),  Scalar(50, 128, 200),  0, 1);
    draw_lines(image, eigen2cv(line), Point2f(50, 50), Scalar(0, 0, 0), 0, 1);
    draw_lines(image, eigen2cv(lower), Point2f(50, 50), Scalar(128, 0, 1000), 0, 1);

    cv::imshow("Expanded Line with Width", image);
    cv::waitKey(0);
    //std::string imagePath = "D:/VSprojects/MicroStructureThermMech/input/therm0.png";
    //Eigen::MatrixXi binaryMat = image2matrix(imagePath);

    //// 验证输出（前5x5像素）
    ////std::cout << "Binary Matrix (first 5x5 elements):\n";
    ////std::cout << binaryMat.block(0, 0, 50, 50) << std::endl;

    //double E = 1.0;
    //double mu = 0.45;

    //std::vector<double> Lambda = { mu * E / ((1 + mu) * (1 - 2 * mu)) , 1e-20 };
    //std::vector<double> Mu = { E / (2 * (1 + mu)) , 1e-20 };
    //double phi = 90;

    //std::vector<double> Lambda2 = { 0, 0 };
    //std::vector<double> Mu2 = { 10, 1 };
    //
    ///*MatrixXd CH = homogenize(binaryMat.rows(), binaryMat.cols(), Lambda, Mu, phi, binaryMat);
    //cout << CH << endl;*/
    //MatrixXd KH = homogenize_therm(binaryMat.rows(), binaryMat.cols(), Lambda2, Mu2, phi, binaryMat);
    //cout <<"KH:"<<endl<< KH << endl;


    // Define points and tangents
    //vector<Vector2d> cp{ {0.0, 0.0}, {1.0, 1.0} };
    //vector<Vector2d> ct{ {1.0, 0.0}, {0.0, 1.0} };
    //// Sampling
    //vector<Vector2d> curvePoints;
    //const int numSamples = 100;
    //HermiteCurve curve(cp,ct, numSamples);
    //curvePoints = curve.curvePoints;
    //
    //// Visualization parameters
    //const int imgSize = 500;
    //const double scale = 200.0; // scale world to image
    //const int offset = 50;

    //// Create blank image
    //cv::Mat img(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

    //// Draw curve
    //for (size_t i = 1; i < curvePoints.size(); ++i) {
    //    cv::line(
    //        img,
    //        worldToImage(curvePoints[i - 1], imgSize, scale, offset),
    //        worldToImage(curvePoints[i], imgSize, scale, offset),
    //        cv::Scalar(0, 0, 255), 2
    //    );
    //}

    //// Draw points and tangents
    //cv::Point p0 = worldToImage(cp[0], imgSize, scale, offset);
    //cv::Point p1 = worldToImage(cp[1], imgSize, scale, offset);
    //cv::circle(img, p0, 5, cv::Scalar(0, 255, 0), -1);
    //cv::circle(img, p1, 5, cv::Scalar(0, 255, 0), -1);

    //// Draw tangent vectors
    //cv::arrowedLine(img, p0, worldToImage(cp[0] + 0.3 * ct[0], imgSize, scale, offset), cv::Scalar(255, 0, 0), 2);
    //cv::arrowedLine(img, p1, worldToImage(cp[1] + 0.3 * ct[1], imgSize, scale, offset), cv::Scalar(255, 0, 0), 2);

    //// Show 
    //cv::imshow("Hermite Curve", img);
   
    //cv::waitKey(0);

    //test

    return 0;
}

