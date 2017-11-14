//
// Created by hyacinth on 2017/4/29.
//

#ifndef SDF2SDF_MYPOINT_H
#define SDF2SDF_MYPOINT_H

class myPoint
{
public:
    myPoint()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    myPoint(double _x, double _y, double _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    double x,y,z;
};
#endif //SDF2SDF_MYPOINT_H