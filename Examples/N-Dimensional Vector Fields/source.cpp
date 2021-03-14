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

int main()
{
    hmath::VectorField3d field;

    for (size_t k = 0; k < 10; k++)
    {
        std::vector<std::vector<hmath::Vector3>> temp_d2;
        for (size_t j = 0; j < 10; j++)
        {
            std::vector<hmath::Vector3> temp_d1;

            for (size_t i = 0; i < 10; i++)
            {
                hmath::Vector3 temp_d0 = {i,j,k};

                temp_d1.push_back(temp_d0);
            }

            temp_d2.push_back(temp_d1);
        }
        
        field.getField().push_back(temp_d2);
    }
        
    return 0;
}