/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <asguard/Transformation.hpp>

using namespace laser_filter;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
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
    
    //Boundingboxes that should cover the wheels
    //note this follows the new X is forward convetion
    AlignedBox<double, 3> leftWheel(Vector3d(-0.25, 0.2, -0.2), Vector3d(0.25, 0.4, 0.5));
    AlignedBox<double, 3> rightWheel(Vector3d(-0.25, -0.4, -0.2), Vector3d(0.25, -0.20, 0.5));
    
    std::vector<AlignedBox<double, 3> > maskedAreas;
    maskedAreas.push_back(leftWheel);
    maskedAreas.push_back(rightWheel);
    
    
    std::vector<Vector3d> currentLaserPoints;
    
    base::samples::LaserScan filteredScan;
    filterLaserScan(filteredScan, scan_samples_sample, laser2Body, maskedAreas);

    //write out the filtered scan
    _filtered_scans.write(filteredScan);
}

void Task::filterLaserScan(base::samples::LaserScan& filterdScan, const base::samples::LaserScan& ls, const Eigen::Affine3d& filterFrame, const std::vector<AlignedBox<double, 3> >& maskedAreas)
{
    std::vector<Eigen::Vector3d> pointCloud;
    std::vector<bool> maskedPoints;

    int lastRange = ls.ranges.at(0);
    const int nrPoints = ls.ranges.size();

    maskedPoints.resize(nrPoints);

    //copy attributes to filtered scan
    filterdScan = ls;

    for(unsigned int i = 0; i < maskedPoints.size(); i++) {
	maskedPoints[i] = false;
    }
    
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {
	//this is a filter for false readings that do occur if one scannes over edgeds of objects
	bool isMasked = abs(lastRange - int(ls.ranges[i])) > 30 && ls.ranges[i] < 1500;

	lastRange = ls.ranges[i];

	//test if laser reading is valid
	if(!ls.isRangeValid(i))
	    continue;
	
	//convert reading to cartesian coordinates
	Vector3d curPoint;
	
	//get a vector with the right length
	curPoint = Vector3d(ls.ranges[i] / 1000.0, 0.0, 0.0);

	//rotate
	curPoint = Quaterniond(AngleAxisd(ls.start_angle + i * ls.angular_resolution, Vector3d::UnitZ())) * curPoint;    
	
	//transform into filter frame
	curPoint = filterFrame * curPoint;

	//check for intersection with masked areas
	for(std::vector<AlignedBox<double, 3> >::const_iterator it = maskedAreas.begin(); it != maskedAreas.end();it++)
	{
	    if(it->contains(curPoint))
	    {
		isMasked = true;
		break;
	    }
	}

	//second filter for ghost readings
	if(isMasked) {
	    //mask previous 5 and following 5 points
	    for(int j = -5; j < 5; j++)
	    {
		if((int(i) +j < 0) 
		   || ((int(i)+j) > nrPoints))
		{
		    continue;
		}
		maskedPoints[i + j] = true;
	    }
	}
    }

    //mark all masked points as invalid in scan 
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {
	if(maskedPoints[i]) {
	    //as we don't have a better error this is an other range error for now
	    filterdScan.ranges[i] = base::samples::OTHER_RANGE_ERRORS;
	}	
    }    
}


