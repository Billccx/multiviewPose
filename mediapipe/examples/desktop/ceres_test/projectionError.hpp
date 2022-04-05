#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <typeinfo>

using namespace Eigen;

class ReprojectionError {
public:
    ReprojectionError(const Eigen::Matrix<double, 3, 4>& pose_matrix, const std::vector<double>& coeffs, const Eigen::Vector2d& feature, const double visibility)
        : pose_matrix_(pose_matrix), coeffs_(coeffs), feature_(feature), visibility_(visibility) {
    }

    template <typename T>
    bool operator()(const T* input_point, T* reprojection_error) const {
        Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);
        //const Eigen::Matrix<T, 2, 1> norm_camera_coordinate =  (pose_matrix_.cast<T>() * point).hnormalized();
        const Eigen::Matrix<T, 2, 1> norm_camera_coordinate =  (pose_matrix_ * point).hnormalized();
        // std::cout<<"point:"<<point<<std::endl;
        // std::cout<<"pose_mat:"<<pose_matrix_<<std::endl;
        // std::cout<<"normal:"<<norm_camera_coordinate<<std::endl;
        T x=norm_camera_coordinate[0];
        T y=norm_camera_coordinate[1];
        T r2=x*x+y*y;
        T f=1.0+coeffs_[0]*r2+coeffs_[1]*r2*r2+coeffs_[4]*r2*r2*r2;
        T xf=x*f;
        T yf=y*f;
        T dx=xf+2.0*coeffs_[2]*x*y+coeffs_[3]*(r2+2.0*x*x);
        T dy=yf+2.0*coeffs_[3]*x*y+coeffs_[2]*(r2+2.0*y*y);
        T rpx = dx * coeffs_[5] + coeffs_[7];
        T rpy = dy * coeffs_[6] + coeffs_[8];

        reprojection_error[0] = (feature_[0] - rpx) * T(visibility_); 
        reprojection_error[1] = (feature_[1] - rpy) * T(visibility_);

        return true;
    }

    static ceres::CostFunction * Create(const Eigen::Matrix<double, 3, 4>& pose_matrix_, const std::vector<double>& coeffs_, const Eigen::Vector2d& feature_, const double visibility_) {
        return (
            new ceres::AutoDiffCostFunction<ReprojectionError, 2,4> (
                new ReprojectionError(pose_matrix_,coeffs_,feature_,visibility_)
            )
        );
    }

private:
    const Eigen::Matrix<double, 3, 4>& pose_matrix_;
    const Eigen::Vector2d& feature_;
    const std::vector<double>& coeffs_;
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