#ifndef CLASSIFY_BASIC_HPP
#define CLASSIFY_BASIC_HPP

#include <vector>
#include <iostream>

namespace ac_classify
{
const int RubikCubeNum = 15;
const int BilliardsNum = 6;

enum class ObjectType
{
    RUBIKCUBE,  // 魔方
    BILLIARDS,  // 台球
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
            size_ = 0.560;  // 魔方边长
            color_ = ObjectColor::NONE;
        } else if(type_ == ObjectType::BILLIARDS) {
            size_ = 0.525;  // 台球直径
            color_ = color;
        } else {
            return;
        }
    }
};

}

#endif // CLASSIFY_BASIC_HPP