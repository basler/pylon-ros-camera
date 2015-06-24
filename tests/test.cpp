
#include "geometry_utils/geometry_utils.h"
#include <stdio.h>

bool testFitPlaneToPointSet()
{
   vector<Point3f> ps;
   ps.push_back(Point3f(0,0,0));
   ps.push_back(Point3f(0,0,0));
   ps.push_back(Point3f(0,0,0));
   ps.push_back(Point3f(0,0,0));

   Mat plane_vec_hom;

   GeometryUtils::fitPlaneToPointSet(ps, plane_vec_hom);

   ps.clear();
   ps.push_back(Point3f(0,0,0));

   ps.push_back(Point3f(1,0,0));
   ps.push_back(Point3f(0,1,0));
   ps.push_back(Point3f(1,1,0));

   GeometryUtils::fitPlaneToPointSet(ps, plane_vec_hom);

   if (plane_vec_hom.rows != 4 || plane_vec_hom.cols != 1)
   {
      printf("size test failed: rows are %d and cols are %d\n", plane_vec_hom.rows, plane_vec_hom.cols);
      return false;
   }

   if (fabs(plane_vec_hom.at<float>(2, 0)) < 0.99)
   {
       printf("direction test failed: expected 1.0 or -1.0 got [%f %f  %f %f]\n", plane_vec_hom.at<float>(0, 0),plane_vec_hom.at<float>(1, 0),plane_vec_hom.at<float>(2, 0),plane_vec_hom.at<float>(3, 0));
       return false;
   }

   if (fabs(plane_vec_hom.at<float>(3, 0)) > 0.01)
   {
       printf("distance test failed\n");
       return false;
   }

   return true;
}


int main(int arg_n, char **arg_v)
{
    assert(arg_n > 1);
    if (std::string("smoke_test") == std::string(arg_v[1]))
    {
       if(!testFitPlaneToPointSet())
          return -1;
    }
    else
    {
       return -1;
    }
    return 0;
}
