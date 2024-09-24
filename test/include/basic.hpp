#ifndef BASIC_HPP
#define BASIC_HPP

#include <vector>
#include <iostream>

namespace basic
{
// int Counter = 0;
const int RubikCubeNum = 15;
const int BilliardsNum = 6;

enum class ObjectType
{
    RUBIKCUBE,
    BILLIARDS,
};

enum class ObjectColor
{
    YELLOW,
    GREEN,
    BROWN,
    BULE,
    PINK,
    BLACK,
    NONE,
};

struct Object
{
    double size_;
    ObjectType type_;
    ObjectColor color_;

    Object(ObjectType type, ObjectColor color) : type_(type)
    {
        if(type_ == ObjectType::RUBIKCUBE){
            size_ = 0.560;
            color_ = ObjectColor::NONE;
        } else if(type_ == ObjectType::BILLIARDS) {
            size_ = 0.525;
            color_ = color;
        } else {
            return;
        }
    }
};

}

#endif // BASIC_HPP