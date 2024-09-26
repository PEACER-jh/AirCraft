#ifndef CLASSIFY_BASIC_HPP
#define CLASSIFY_BASIC_HPP

#include <vector>
#include <iostream>

namespace ac_classify
{
#define RubikCubeSize 0.560;   // m
#define BilliardsSize 0.525;   // m
#define RubikCubeNum 15;
#define BilliardsNum 6;

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
            size_ = RubikCubeSize;
            color_ = ObjectColor::NONE;
        } else if(type_ == ObjectType::BILLIARDS) {
            size_ = BilliardsSize;
            color_ = color;
        } else {
            return;
        }
    }
};

}

#endif // CLASSIFY_BASIC_HPP