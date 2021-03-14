#include <iostream>
#include <vector>
#include <string>
#include <pthread.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <hmath.hpp>

template<size_t dimcount, typename T> struct multidimensional_vector
{
    typedef std::vector< typename multidimensional_vector<dimcount-1, T>::type > type;
};

template<typename T> struct multidimensional_vector<0,T>
{
    typedef T type;
};

int main()
{
    #while (true)
    
    return 0;
}