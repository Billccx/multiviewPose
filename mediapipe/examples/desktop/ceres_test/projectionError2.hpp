#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <typeinfo>

using namespace Eigen;

class ReprojectionError2 {
public:
    ReprojectionError2(const Eigen::Matrix<double, 3, 3>& pose_matrixR, 
                    const Eigen::Matrix<double, 3, 1>& pose_matrixt, 
                    const std::vector<double>& coeffs, 
                    const Eigen::Vector2d& feature, 
                    const double visibility)
        : pose_matrixR_(pose_matrixR), 
          pose_matrixt_(pose_matrixt),
          coeffs_(coeffs), 
          feature_(feature), 
          visibility_(visibility) {
    }

    template <typename T>
    bool operator()(const T* input_point, T* reprojection_error) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1> > point(input_point);
        //std::cout<<pose_matrixR_<<std::endl;
        //std::cout<<pose_matrixt_<<std::endl;
        Eigen::Matrix<T,3,1> camera_coordinate=pose_matrixR_ * point + pose_matrixt_;
        //const Eigen::Matrix<T, 2, 1> norm_camera_coordinate =  (pose_matrixR_ * point + pose_matrixt_).hnormalized();

        T x=camera_coordinate[0]/camera_coordinate[2];
        T y=camera_coordinate[1]/camera_coordinate[2];
        T r2=x*x+y*y;
        T f=1.0+coeffs_[0]*r2+coeffs_[1]*r2*r2+coeffs_[4]*r2*r2*r2;
        T xf=x*f;
        T yf=y*f;
        T dx=xf+2.0*coeffs_[2]*x*y+coeffs_[3]*(r2+2.0*x*x);
        T dy=yf+2.0*coeffs_[3]*x*y+coeffs_[2]*(r2+2.0*y*y);
        T rpx = dx * coeffs_[5] + coeffs_[7];
        T rpy = dy * coeffs_[6] + coeffs_[8];

        reprojection_error[0] = (feature_[0] - rpx) * visibility_; 
        reprojection_error[1] = (feature_[1] - rpy) * visibility_;

        return true;
    }

    static ceres::CostFunction * Create(const Eigen::Matrix<double, 3, 3>& pose_matrixR_, 
                                        const Eigen::Matrix<double, 3, 1>& pose_matrixt_,
                                        const std::vector<double>& coeffs_, 
                                        const Eigen::Vector2d& feature_, 
                                        const double visibility_) {
        return (
            new ceres::AutoDiffCostFunction<ReprojectionError2, 2, 3> (
                new ReprojectionError2(pose_matrixR_,pose_matrixt_,coeffs_,feature_,visibility_)
            )
        );
    }

private:
    const Eigen::Matrix<double, 3, 3>& pose_matrixR_;
    const Eigen::Matrix<double, 3, 1>& pose_matrixt_;
    const Eigen::Vector2d& feature_;
    const std::vector<double> coeffs_;
    const double visibility_;
    // coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]. + [fx,fy,ppx,ppy]
};


/*
if (intrin->model == RS2_DISTORTION_BROWN_CONRADY){
    float r2 = x * x + y * y;
    float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;

    float xf = x * f;
    float yf = y * f;

    float dx = xf + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
    float dy = yf + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);

    x = dx;
    y = dy;
}
*/