/*
 *  Function: OCamModel
 *
 *  Copy from MultiColSLAM
 *
 *  Author: Xu Qingwen
 *  Date: 2017.08.06
 */

#ifndef CAM_MODEL_GENERAL_H
#define CAM_MODEL_GENERAL_H

#include <opencv2/opencv.hpp>
#include <limits>
#include <Eigen/Dense>

namespace OmniSLAM{
    class CamModelGeneral{
    public:
        CamModelGeneral() :
                c_(1),
                d_(0),
                e_(0),
                u0_(0),
                v0_(0),
                p_((cv::Mat_<double>(1, 1) << 1)),
                inv_p_((cv::Mat_<double>(1, 1) << 1)),
                p_deg_(1),
                inv_p_deg_(1),
                img_width_(0),
                img_height_(0),
                p1_(1)
        {}

        CamModelGeneral(double cdeu0v0[],
                        cv::Mat_<double> p,
                        cv::Mat_<double> inv_p) :
                c_(cdeu0v0[0]),
                d_(cdeu0v0[1]),
                e_(cdeu0v0[2]),
                u0_(cdeu0v0[3]),
                v0_(cdeu0v0[4]),
                p_(p),
                inv_p_(inv_p){
                // initialize degree of polynomials
                p_deg_ = p.rows > 1 ? p.rows : p.cols;
                inv_p_deg_ = p.rows > 1 ? inv_p.rows : inv_p.cols;
                cde1_ = (cv::Mat_<double>(2, 2) << c_, d_, e_, 1.0);
                p1_ = p_.at<double>(0);
                inv_affine_ = c_ - d_ * e_;
        }

        CamModelGeneral(double cdeu0v0[],
                        cv::Mat_<double> p,
                        cv::Mat_<double > inv_p,
                        double width,
                        double height) :
                c_(cdeu0v0[0]),
                d_(cdeu0v0[1]),
                e_(cdeu0v0[2]),
                u0_(cdeu0v0[3]),
                v0_(cdeu0v0[4]),
                p_(p),
                inv_p_(inv_p),
                img_width_(width),
                img_height_(height){
                // initialize degree of polynomials
                p_deg_ = p.rows > 1 ? p.rows : p.cols;
                inv_p_deg_ = p.rows > 1 ? inv_p.rows : inv_p.cols;
                cde1_ = (cv::Mat_<double>(2, 2) << c_, d_, e_, 1.0);
                p1_ = p_.at<double>(0);
                inv_affine_ = c_ - d_ * e_;
        }

        ~CamModelGeneral(){}

        void WorldToImg(const double& x,
                        const double& y,
                        const double& z,    // 3D scene point
                        double& u,
                        double& v) const;
        void WorldToImg(const cv::Point3_<double>& X,
                        cv::Point_<double>& m);
        void WorldToImg(const cv::Vec3d& X, cv::Vec2d& m);
        void WorldToImg(const cv::Vec3d& X, cv::Vec2f& m);
        void ImgToWorld(double& x,
                        double& y,
                        double& z,    // 3D scene point
                        const double& u,
                        const double& v);
        void ImgToWorld(cv::Point3_<double>& X,
                        const cv::Point_<double>& m);
        void ImgToWorld(cv::Vec3d& X, const cv::Vec2d& m);
        void undistortPointsOcam(
			const double& ptx, const double& pty,
			const double& undistScaleFactor,
			double& out_ptx, double& out_pty) {
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			this->ImgToWorld(x, y, z, ptx, pty);
			out_ptx = -x / z * undistScaleFactor;
			out_pty = -y / z * undistScaleFactor;
		}

        void distortPointsOcam(
                const double& ptx, const double& pty,
                double& dist_ptx, double& dist_pty) {
                WorldToImg(ptx, pty, -p1_, dist_ptx, dist_pty);
        }

        // get functions
        double Get_c() { return c_; }
        double Get_d() { return d_; }
        double Get_e() { return e_; }

        double Get_u0() { return u0_; }
        double Get_v0() { return v0_; }

        int GetInvDeg() { return inv_p_deg_; }
        int GetPolDeg() { return p_deg_; }

        cv::Mat_<double> Get_invP() { return inv_p_; }
        cv::Mat_<double> Get_P() { return p_; }

        double GetWidth() { return img_width_; }
        double GetHeight() { return img_height_; }

        cv::Mat GetMirrorMask(int pyrL) { return mirror_masks_[pyrL]; }
        void SetMirrorMasks(std::vector<cv::Mat> mirror_masks) {
                mirror_masks_ = mirror_masks; }

        bool IsPointInMirrorMask(const double& u, const double& v, int pyr);


        inline double operator [](int i) const
        {
                assert(i < (12 + 5));
                if (i > 4)
                        return inv_p_.at<double>(i - 5, 0);
                if (i == 0)
                        return c_;
                if (i == 1)
                        return d_;
                if (i == 2)
                        return e_;
                if (i == 3)
                        return u0_;
                if (i == 4)
                        return v0_;
        }

        // ATTENTION, used fixed size here!!
        // this will not work in general!!
        inline Eigen::Matrix<double, 12 + 5, 1> ToVector() const
        {
                Eigen::Matrix<double, 12 + 5, 1> tmp =
                        Eigen::Matrix<double, 12 + 5, 1>::Zero();
                tmp(0, 0) = c_;
                tmp(1, 0) = d_;
                tmp(2, 0) = e_;
                tmp(3, 0) = u0_;
                tmp(4, 0) = v0_;
                for (int i = 0; i < inv_p_.rows; ++i)
                        tmp(5 + i, 0) = inv_p_.at<double>(i, 0);

                return tmp;
        }

        inline void FromVector(const Eigen::Matrix<double, 12 + 5, 1> tmp)
        {
                c_ = tmp(0, 0);
                d_ = tmp(1, 0);
                e_ = tmp(2, 0);
                u0_ = tmp(3, 0);
                v0_ = tmp(4, 0);
                for (int i = 0; i < inv_p_.rows; ++i)
                        inv_p_.at<double>(i, 0) = tmp(5 + i, 0);
        }

        inline CamModelGeneral operator+ (
                const Eigen::Matrix<double, 12 + 5, 1>& values2add) const
        {
                CamModelGeneral result(*this);
                Eigen::Matrix<double, 12 + 5, 1> valuesOld =
                        result.ToVector();
                result.FromVector(valuesOld + values2add);
                return result;
        }

    protected:
        //affine
        double c_;
        double d_;
        double e_;
        double inv_affine_;
        cv::Mat_<double> cde1_;
        //principal
        double u0_;
        double v0_;
        //polynomial
        double p1_;
        cv::Mat_<double> p_;
        int p_deg_;
        //inversee polynomial
        cv::Mat_<double> inv_p_;
        int inv_p_deg_;
        //image width and height
        double img_width_;
        double img_height_;
        //mirror mask on pyramid levels
        std::vector<cv::Mat> mirror_masks_;
    };

}

#endif
