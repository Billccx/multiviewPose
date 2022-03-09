#ifndef SnavelyReprojection_H
#define SnavelyReprojection_H

#include <iostream>
#include "ceres/ceres.h"
#include "rotation.h"
#include <typeinfo>

class SnavelyReprojectionError {
public:
    SnavelyReprojectionError(double observation_x, double observation_y) 
    : observed_x(observation_x),observed_y(observation_y) {
    }

    template<typename T>
    bool operator()(const T *const camera,const T *const point,T *residuals) const {
        // camera[0,1,2] are the angle-axis rotation
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(observed_x);
        residuals[1] = predictions[1] - T(observed_y);
        return true;
    }

    // camera : 9 dims array
    // [0-2] : angle-axis rotation
    // [3-5] : translateion
    // [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
    // point : 3D location.
    // predictions : 2D predictions with center of the image plane. 归一化平面上的二维坐标
    template<typename T>
    static inline bool CamProjectionWithDistortion(const T *camera, const T *point, T *predictions) {

        std::cout<<std::endl;
        std::cout<<"type of camera: "<<typeid(camera).name()<<std::endl;
        std::cout<<"type of point: "<<typeid(point).name()<<std::endl;
        std::cout<<"type of predictions: "<<typeid(predictions).name()<<std::endl;



        // Rodrigues' formula
        T p[3];

        //points*R
        AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation   points+t
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center fo distortion
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion
        const T &l1 = camera[7];
        const T &l2 = camera[8];

        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + r2 * (l1 + l2 * r2);

        const T &focal = camera[6];
        predictions[0] = focal * distortion * xp;
        predictions[1] = focal * distortion * yp;

        return true;
    }

    //2,9,3分别代表观测维度(残差维度),优化变量位姿的维度,优化变量空间点的维度
    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)
            )
        );
    }

private:
    double observed_x;
    double observed_y;
};

#endif // SnavelyReprojection.h

