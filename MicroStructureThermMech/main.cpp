#include <iostream>
#include <vector>
#include <string>


#include "tool.h"
#include "microStruGenerate.h"
#include "propertyCalculate.h"
#include "Visualization.h"

cv::Point2i worldToImage(const Vector2d& pt, int imgSize, double scale, int offset) {
    int x = static_cast<int>(pt.x() * scale) + offset;
    int y = imgSize - (static_cast<int>(pt.y() * scale) + offset); // Flip Y
    return cv::Point(x, y);
}

int main()
{
    // 设置精度和科学计数法显示
    std::cout << std::fixed << std::setprecision(6);  // 固定小数，保留 4 位
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

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
    vector<Vector2d> cp{ {0.0, 0.0}, {1.0, 1.0} };
    vector<Vector2d> ct{ {1.0, 0.0}, {0.0, 1.0} };
    // Sampling
    vector<Vector2d> curvePoints;
    const int numSamples = 100;
    HermiteCurve curve(cp,ct, numSamples);
    curvePoints = curve.curvePoints;
    
    // Visualization parameters
    const int imgSize = 500;
    const double scale = 200.0; // scale world to image
    const int offset = 50;

    // Create blank image
    cv::Mat img(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw curve
    for (size_t i = 1; i < curvePoints.size(); ++i) {
        cv::line(
            img,
            worldToImage(curvePoints[i - 1], imgSize, scale, offset),
            worldToImage(curvePoints[i], imgSize, scale, offset),
            cv::Scalar(0, 0, 255), 2
        );
    }

    // Draw points and tangents
    cv::Point p0 = worldToImage(cp[0], imgSize, scale, offset);
    cv::Point p1 = worldToImage(cp[1], imgSize, scale, offset);
    cv::circle(img, p0, 5, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, p1, 5, cv::Scalar(0, 255, 0), -1);

    // Draw tangent vectors
    cv::arrowedLine(img, p0, worldToImage(cp[0] + 0.3 * ct[0], imgSize, scale, offset), cv::Scalar(255, 0, 0), 2);
    cv::arrowedLine(img, p1, worldToImage(cp[1] + 0.3 * ct[1], imgSize, scale, offset), cv::Scalar(255, 0, 0), 2);

    // Show
    cv::imshow("Hermite Curve", img);
   
    cv::waitKey(0);
    return 0;
}

