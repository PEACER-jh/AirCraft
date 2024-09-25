#include <unordered_map>
#include "cam_interface.hpp"

namespace ac_camera
{
CamInterface::CamInterface() 
{
    this->is_open_ = false;
    this->is_init_ = false;
    this->is_run_ = false;
    this->error_message_ = "";

    SensorOpen();
    SensorInit();

    if(isOpen() && isInit())
        this->is_run_ = true;
}

CamInterface::~CamInterface() 
{
    if(SensorShut()){
        this->is_open_ = false;
        this->is_init_ = false;
        this->is_run_ = false;
    }
}

bool CamInterface::SensorOpen()
{
    this->capture_.open(0);
    if(this->capture_.isOpened()){
        this->is_open_ = true;
        return true;
    }
    return false;
}

bool CamInterface::SensorInit() 
{
    this->is_init_ = true;
}

bool CamInterface::SensorRun(cv::Mat & image)
{
    if(!this->is_open_) 
        return false;

    capture_ >> image;
    return true;
}

bool CamInterface::SensorShut() 
{
    this->capture_.release();
    return true;
}

bool CamInterface::isOpen() { return this->is_open_; }
bool CamInterface::isInit() { return this->is_init_; }
bool CamInterface::isRun() { return this->is_run_; }








}