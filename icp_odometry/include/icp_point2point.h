#include "icp_general.h"

class icp_point2point: public icp_general<pcl::PointXYZ, pcl::PointXYZ> {
public:
    // constructors
    icp_point2point();
    ~icp_point2point(){};

private:
    // private functions
    void calculate_rotation() override;
    double calculate_error() override;
};
