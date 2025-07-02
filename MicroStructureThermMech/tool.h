#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/src/KroneckerProduct/KroneckerTensorProduct.h>
#include <iostream>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L /* pi */
#endif                                               // ! M_PI

// calculate homo property
MatrixXd homogenize(int lx, int ly, 
    std::vector<double> lambda, 
    std::vector<double> mu, 
    double phi, 
    MatrixXi x);

MatrixXd homogenize_therm(int lx, int ly,
    std::vector<double> lambda,
    std::vector<double> mu,
    double phi,
    MatrixXi x);

void elementMatVec(double a,
    double b,
    double phi,
    MatrixXd& keLambda,
    MatrixXd& keMu,
    MatrixXd& feLambda,
    MatrixXd& feMu);

void elementMatVec_therm(double a,
    double b,
    double phi,
    MatrixXd& keLambda,
    MatrixXd& keMu,
    MatrixXd& feLambda,
    MatrixXd& feMu);

MatrixXi image2matrix(string filename);

void showSparseMatrix(SparseMatrix<double> X);

//------------------------------math tools-------------------------------------------
class BSampleFunction
{
public:
    BSampleFunction() {};
    std::vector<Eigen::Vector2d> ThreeOrderBSplineInterpolatePt(std::vector<Eigen::Vector2d>& pt, int InsertNum);

private:
    double F03(double t);
    double F13(double t);
    double F23(double t);
    double F33(double t);
};

class HermiteCurve
{
public:
    HermiteCurve() {};

    HermiteCurve(const std::vector<Eigen::Vector2d>& control_points, const std::vector<Eigen::Vector2d>& tangents, double segnum);

    HermiteCurve(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d t0, Eigen::Vector2d t1, double segnum);

    HermiteCurve(Eigen::Vector2d p0, Eigen::Vector2d p1, double angle1, double angle2, int segnum);

    Eigen::Vector2d getPoint(double t);

    std::vector<Eigen::Vector2d> getPoints(int segnum);

    Eigen::Vector2d getDerivation(double t);

    double distance(const Eigen::Vector2d& point);
    
public:
     std::vector<Eigen::Vector2d> ctrP; // control points
     std::vector<Eigen::Vector2d> ctrT; // tangents
     std::vector<Eigen::Vector2d> curvePoints; // curvePoints

};

// 归一化向量（单位向量）
Vector2d normVector(const Vector2d& v);
// 计算点的偏移点
Vector2d offsetPoint(const Vector2d& p, const Vector2d& dir, double distance);

std::vector<Vector2d>expandLineToWidth(const std::vector<Vector2d>& line, double width);

std::vector<Vector2d> expandLineRadial(const std::vector<Vector2d>& line, Vector2d ray_center, double width);
std::vector<Vector2d> combineVectors_reverse(std::vector<Vector2d>& line1, std::vector<Vector2d>& line2);

//--------------------------------- visual tool----------------------------------------------
cv::Point2i worldToImage(const Vector2d& pt, int imgSize, double scale, int offset);

void draw_poly(Mat& drawing_, vector<Point2f> contour_s, Point2f shift, Scalar color = Scalar(0, 0, 0));
void draw_contour(Mat& drawing_, vector<Point2f> contour_s, Point2f shift, Scalar color = Scalar(0,0,0), int type = 0, int thickness = 1);
void draw_lines(Mat& drawing_, vector<Point2f> contour_s, Point2f shift, Scalar color = Scalar(0, 0, 0), int type = 0, int thickness = 1);


//data transfer
vector<Point2f> eigen2cv(vector<Vector2d> points);

vector<Vector2d> cv2eigen(vector<Point2f> points);