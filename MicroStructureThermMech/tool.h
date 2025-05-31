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


//--------------------------------- visual tool----------------------------------------------
cv::Point2i worldToImage(const Vector2d& pt, int imgSize, double scale, int offset);