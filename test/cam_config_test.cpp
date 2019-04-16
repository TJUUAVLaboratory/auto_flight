#include <flight/Config.h>
using namespace myslam;
int main()
{
     Config::setParameterFile("src/auto_flight/default.yaml");
     float fx = Config::get<float>("camera.fx");
    std::cout<< fx << std::endl;
}