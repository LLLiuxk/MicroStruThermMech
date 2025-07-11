#include <iostream>
#include <vector>
#include <string>


#include "tool.h"
#include "microStruGenerate.h"
#include "propertyCalculate.h"
#include "Visualization.h"

using namespace msGen;

int main()
{
    // 设置精度和科学计数法显示
    std::cout << std::fixed << std::setprecision(6);  // 固定小数，保留 4 位
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<PointTang> allPoints_ = {
    PointTang(Vector2d(0,100), Vector2d(100,0)),
    PointTang(Vector2d(100,200), Vector2d(0,100)),
    PointTang(Vector2d(100,200), Vector2d(0,-100)),
    PointTang(Vector2d(200,100), Vector2d(100,0))
    };
    std::vector<Connection> connections_ = {
    Connection(0,1,1,MODE_HERMITE,20),
    Connection(2,3,2,MODE_BEZIER,20)
    };

    HermiteCurve heline1(Vector2d(0, 100), Vector2d(100, 200), Vector2d(100, 0), Vector2d(0, 100), 20);
    vector<Vector2d> ctpoints = { Vector2d(0, 100), Vector2d(100, 100), Vector2d(100, 200) };
    BezierCurve bline1(Vector2d(0, 100), Vector2d(100, 200), Vector2d(100, 0), Vector2d(0, 100), 20);
    //for (auto p : curvepoints)
    //    cout << p << endl;

    /*for(auto p: heline1.curvePoints)
        cout << p << endl;*/
    draw_lines(image, eigen2cv(heline1.curvePoints), Point2f(100,100), Scalar(100,0,0),1);

    draw_lines(image, eigen2cv(bline1.curvePoints), Point2f(100, 100), Scalar(0, 100, 0), 1);
    /*QuarterCell cell1(allPoints_, connections_);
    cell1.createCell();
    cell1.drawCell();*/




    // 示例：x轴上的直线 (y=0)
   // std::vector<Vector2d> line = { {0, 0}, {20,0}, { 50, 0 }, {100, 0} };
   // double width = 50.0; // 宽度

   // 
   // Point2f center(400, 300);
   // vector<Point2f> CIR;
   // for (int i = 0; i < 10; ++i) {
   //     double angle = static_cast<double>(i) * CV_PI / 10;
   //     float x = center.x + 20 * cos(angle);
   //     float y = center.y + 20 * sin(angle);
   //     CIR.push_back(Point2f(x, y));
   // }

   ///* std::pair<std::vector<Vector2d>, std::vector<Vector2d>> two_v*///auto two_v = expandLineToWidth(cv2eigen(CIR), width);
   // Vector2d cen(center.x, center.y);
   // auto two_v = expandLineRadial(cv2eigen(CIR), Vector2d(300,100), width);

   // // 打印结果
   ////std::vector<Vector2d>& upper = two_v.first;
   ////std::vector<Vector2d>& lower = two_v.second;

   // std::cout << "Upper Boundary:\n";
   // for (const auto& p : two_v) {
   //     std::cout << "(" << p.x() << ", " << p.y() << ")\n";
   // }

   // /*std::cout << "\nLower Boundary:\n";
   // for (const auto& p : lower) {
   //     std::cout << "(" << p.x() << ", " << p.y() << ")\n";
   // }*/

   // draw_poly(image, eigen2cv(two_v), Point2f(50,50),  Scalar(0, 128, 200));
   // draw_lines(image, CIR, Point2f(50, 50), Scalar(0, 0, 0), 0, 1);
    //draw_lines(image, eigen2cv(lower), Point2f(50, 50), Scalar(128, 0, 1000), 0, 1);

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

