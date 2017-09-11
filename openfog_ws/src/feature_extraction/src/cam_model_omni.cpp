//
// Created by xuqw on 17-8-12.
//

#include "cam_model_omni.h"
#include "misc.h"

namespace OmniSLAM{
    using namespace cv;
    using namespace std;

    void CamModelGeneral::ImgToWorld(cv::Point3_<double> &X,
                                     const cv::Point_<double> &m) {
        const double u_t = m.x -u0_;
        const double v_t = m.y -v0_;
        // inverse affine matrix image to sensor plane conversion
        X.x = (1*u_t - d_*v_t) / this->inv_affine_;
        X.y = (-e_ * u_t + c_ * v_t) / this->inv_affine_;
        const double X2 = pow(X.x, 2);
        const double Y2 = pow(X.y, 2);
        X.z = -horner((double*)p_.data, p_deg_, sqrt(X2 + Y2));
        // normalize vectors spherically
        const double  norm = sqrt(X2 + Y2 + pow(X.z, 2));
        X.x /= norm;
        X.y /= norm;
        X.z /= norm;
    }

    void CamModelGeneral::ImgToWorld(
            double &x,
            double &y,
            double &z,
            const double &u,
            const double &v) {
        const double u_t = x -u0_;
        const double v_t = y -v0_;
        // inverse affine matrix image to sensor plane conversion
        x = (1*u_t - d_*v_t) / this->inv_affine_;
        y = (-e_ * u_t + c_ * v_t) / this->inv_affine_;
        const double X2 = pow(x, 2);
        const double Y2 = pow(y, 2);
        z = -horner((double*)p_.data, p_deg_, sqrt(X2 + Y2));
        // normalize vectors spherically
        const double  norm = sqrt(X2 + Y2 + pow(z, 2));
        x /= norm;
        y /= norm;
        z /= norm;
    }

    void CamModelGeneral::ImgToWorld(cv::Vec3d &X, const cv::Vec2d &m) {
        //double invAff = c - d*e;
        const double u_t = m(0) - u0_;
        const double v_t = m(1) - u0_;
        // inverse affine matrix image to sensor plane conversion
        X(0) = (u_t - d_ * v_t) / this->inv_affine_;
        X(1) = (-e_ * u_t + c_ * v_t) / this->inv_affine_;
        const double X2 = pow(X(0), 2);
        const double Y2 = pow(X(1), 2);
        X(2) = -horner((double*)p_.data, p_deg_, sqrt(X2 + Y2));

        // normalize vectors spherically
        double norm = sqrt(X2 + Y2 + pow(X(2), 2));
        X(0) /= norm;
        X(1) /= norm;
        X(2) /= norm;
    }

    void CamModelGeneral::WorldToImg(const cv::Point3_<double> &X, cv::Point_<double> &m) {
        double norm = sqrt(X.x*X.x + X.y*X.y);

        if (norm == 0.0)
            norm = 1e-14;

        const double theta = atan(-X.z / norm);
        const double rho = horner((double*)inv_p_.data, inv_p_deg_, theta);

        const double uu = X.x / norm * rho;
        const double vv = X.y / norm * rho;

        m.x = uu*c_ + vv*d_ + u0_;
        m.y = uu*e_ + vv + v0_;
    }

    void CamModelGeneral::WorldToImg(const cv::Vec3d &X, cv::Vec2d &m) {
        double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

        if (norm == 0.0)
            norm = 1e-14;

        const double theta = atan(-X(2) / norm);
        const double rho = horner((double*)inv_p_.data, inv_p_deg_, theta);

        const double uu = X(0) / norm * rho;
        const double vv = X(1) / norm * rho;

        m(0) = uu*c_ + vv*d_ + u0_;
        m(1) = uu*e_ + vv + v0_;
    }

    void CamModelGeneral::WorldToImg(const cv::Vec3d &X, cv::Vec2f &m) {
        double norm = cv::sqrt(X(0)*X(0) + X(1)*X(1));

        if (norm == 0.0)
            norm = 1e-14;

        const double theta = atan(-X(2) / norm);

        const double rho = horner((double*)inv_p_.data, inv_p_deg_, theta);

        const double uu = X(0) / norm * rho;
        const double vv = X(1) / norm * rho;

        m(0) = uu*c_ + vv*d_ + u0_;
        m(1) = uu*e_ + vv + v0_;
    }

    void CamModelGeneral::WorldToImg(
            const double &x,
            const double &y,
            const double &z,
            double &u,
            double &v) const {
        double norm = sqrt(x*x + y*y);
        if (norm == 0.0)
            norm = 1e-14;

        const double theta = atan(-z / norm);
        const double rho = horner((double*)inv_p_.data, inv_p_deg_, theta);

        const double uu = x / norm * rho;
        const double vv = y / norm * rho;

        u = uu*c_ + vv*d_ + u0_;
        v = uu*e_ + vv + v0_;
    }

    bool CamModelGeneral::IsPointInMirrorMask(const double &u, const double &v, int pyr) {
        const int ur = cvRound(u);
        const int vr = cvRound(v);
        // check image bounds
        if (ur >= mirror_masks_[pyr].cols || ur <= 0 ||
            vr >= mirror_masks_[pyr].rows || vr <= 0)
            return false;
        // check mirror
        if (mirror_masks_[pyr].ptr<uchar>(vr)[ur] > 0)
            return true;
        else return false;
    }

    void CreateMirrorMask(CamModelGeneral camera,
                          int pyrLevel,
                          vector<Mat>& mirror_masks)
    {
        int w = (int)camera.GetWidth();
        int h = (int)camera.GetHeight();
        float u0 = (float)camera.Get_v0();
        float v0 = (float)camera.Get_u0();
        Mat sizeDef = Mat::zeros(h, w, CV_8UC1);
        vector<Mat> sizeDefvec;
        buildPyramid(sizeDef, sizeDefvec, pyrLevel);
        // Mirror mask for pyramid
        float offset[4] = { 22.0f, 10.0f, 5.0f, 1.0f };
        //float offset[4] = { 50.0f, 50.0f, 50.0f, 50.0f};
        for (int mIdx = 0; mIdx < pyrLevel; mIdx++)
        {
            if (mIdx != 0)
            {
                w = sizeDefvec[mIdx].cols;
                h = sizeDefvec[mIdx].rows;

                u0 = ceil(u0 / 2.0f);
                v0 = ceil(v0 / 2.0f);
            }
            Mat tempMask = Mat::zeros(h, w, CV_8UC1);
            for (int i = 0; i < h; ++i)
            {
                for (int j = 0; j < w; ++j)
                {
                    float ans = sqrt((float)pow(i - u0, 2) + (float)pow(j - v0, 2));
                    if (ans < (u0 + offset[mIdx]))
                        tempMask.at<uchar>(i, j) = 255;
                    else
                        tempMask.at<uchar>(i, j) = 0;
                }

            }
            mirror_masks.push_back(tempMask);
        }
    }
}

