/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <asguard/Transformation.hpp>

using namespace laser_filter;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name), boxFilter(NULL), noiseFilter(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), boxFilter(NULL), noiseFilter(NULL)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if(!laser_filter::TaskBase::configureHook())
	return false;
    
    if(boxFilter)
	delete boxFilter;
    
    boxFilter = new BoxFilter();
    
    std::vector<laser_filter::Box> boxes = _filterBoxes.get();
    
    for(std::vector<laser_filter::Box>::const_iterator it = boxes.begin(); it != boxes.end(); it++)
    {
	boxFilter->addBoundingBox(*it);
    }
    
    delete noiseFilter;
    noiseFilter = new NoiseFilter();

    noiseFilter->setMaxIncline(_maxIncline.get());
    noiseFilter->setMinIncline(_minIncline.get());
    noiseFilter->setNumMaskedNeighbours(_maskedNeighbours.get());

    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
	return false;
    
    asguard::Transformation tf;

    base::samples::RigidBodyState lowerDyn2Head;
    lowerDyn2Head.sourceFrame = "lower_dynamixel";
    lowerDyn2Head.targetFrame = "head";
    lowerDyn2Head.setTransform(tf.lowerDynamixel2Head);
    _transformer.pushStaticTransformation(lowerDyn2Head);

    base::samples::RigidBodyState tiltHead2UpperDyn;
    tiltHead2UpperDyn.sourceFrame = "tilt_head";
    tiltHead2UpperDyn.targetFrame = "upper_dynamixel";
    tiltHead2UpperDyn.setTransform(tf.tiltHead2UpperDynamixel);
    _transformer.pushStaticTransformation(tiltHead2UpperDyn);
    
    base::samples::RigidBodyState laser2TiltHead;
    laser2TiltHead.sourceFrame = "laser";
    laser2TiltHead.targetFrame = "tilt_head";
    laser2TiltHead.setTransform(tf.laser2TiltHead);
    _transformer.pushStaticTransformation(laser2TiltHead);
    
    base::samples::RigidBodyState head2Body;
    head2Body.sourceFrame = "head";
    head2Body.targetFrame = "body";
    head2Body.setTransform(tf.head2Body);
    _transformer.pushStaticTransformation(head2Body);  
    
    return laser_filter::TaskBase::startHook();

}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    Eigen::Affine3d laser2Body;
    
    //check if laser2Body is available
    if(!_laser2body.get(ts, laser2Body, true))
    {
	//no transformation to filter frame available
	//we drop the whole scan as we are unable to
	//filter it
	return;
    }

    
    base::samples::LaserScan noisefilteredScan;
    
    //first the noise filter
    noiseFilter->filter(noisefilteredScan, scan_samples_sample);
    
    base::samples::LaserScan filteredScan;
    //the the BB filter
    boxFilter->setFilterFrame(laser2Body);
    boxFilter->filter(filteredScan, noisefilteredScan);

    //write out the filtered scan
    _filtered_scans.write(filteredScan);
    
    base::samples::RigidBodyState filterFrame;
    filterFrame.setTransform(laser2Body);
    filterFrame.cov_position = Eigen::Matrix3d::Identity();
    _debug_laser_frame.write(filterFrame);
}


