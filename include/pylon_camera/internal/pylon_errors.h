#ifndef PYLON_ERRORS_INTERNAL_H_
#define PYLON_ERRORS_INTERNAL_H_

#include <stdexcept>

namespace pylon_camera
{

class CameraRemovedError : std::runtime_error
{
public:
    const char* what() const throw ()
    {
        return "Camera removed";
    }
};

}

#endif
