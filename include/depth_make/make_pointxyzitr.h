#define PCL_NO_PRECOMPILE 
#ifndef MAKE_POINTXYZITR_H
#define MAKE_POINTXYZITR_H
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
 
namespace pcl
{
struct PointXYZITR
{
    PCL_ADD_POINT4D;          
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;       
}
 
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZITR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (uint16_t, ring, ring)
                                  )
 
#endif