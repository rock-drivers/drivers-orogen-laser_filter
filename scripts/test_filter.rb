#! /usr/bin/env ruby
require 'vizkit'
require 'roby/vizkit'

folder = ARGV[0]

puts("Folder #{folder}")


Orocos.initialize

# setup the environment so that ruby can find the test deployment
ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"


replay = Orocos::Log::Replay.open(folder)
Vizkit.use_tasks replay.tasks

view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')

laserFrameViz = view3d.createPlugin('RigidBodyStateVisualization')
laserFrameViz.resetModel(0.5)

laserViz = view3d.createPlugin('LaserScanVisualization')

lBoxViz = view3d.createPlugin('laser_filter', 'BoxVisualization')
rBoxViz = view3d.createPlugin('laser_filter', 'BoxVisualization')

y_forward_to_x_forward = Eigen::Quaternion.from_angle_axis(Math::PI/2.0, Eigen::Vector3.new(0, 0, 1))

lastLaserFrame = nil


Orocos.run "laser_filter_test" do 
    
    laser_filter = Orocos::TaskContext.get('laser_filter')
    
    laser_filter.maskedNeighbours = 3;
    laser_filter.minIncline = 0.2
    laser_filter.maxIncline = Math::PI - 0.2
    
    leftBox = Types::LaserFilter::Box.new
    leftBox.downLeft = Eigen::Vector3.new(0.2, -0.25, -0.2)
    leftBox.upRight = Eigen::Vector3.new(0.4, 0.25, 0.5)
 
    rightBox = Types::LaserFilter::Box.new
    rightBox.downLeft = Eigen::Vector3.new(-0.4, -0.25, -0.2)
    rightBox.upRight = Eigen::Vector3.new(-0.20, 0.25, 0.5)
    
    
    boxes = laser_filter.filterBoxes;
    boxes << leftBox
    boxes << rightBox
    
    laser_filter.filterBoxes = boxes

    lBoxViz.updateData(leftBox)
    rBoxViz.updateData(rightBox)
    
    replay.hokuyo.scans.connect_to laser_filter.scan_samples, :type => :buffer, :size => 100

    Vizkit.connect_port_to 'laser_filter', 'filtered_scans', :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
	if(laserFrameViz)
	    laserViz.updateData(lastLaserFrame)
	    laserViz.updateData(sample)
	end
	sample
    end    

    Vizkit.connect_port_to 'laser_filter', 'debug_laser_frame', :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
	lastLaserFrame = sample;
	lastLaserFrame.orientation = lastLaserFrame.orientation * y_forward_to_x_forward
	sample
    end    
    
    replay.dynamixel.lowerDynamixel2UpperDynamixel.connect_to laser_filter.dynamic_transformations, :type => :buffer, :size => 100
    
    laser_filter.configure
    laser_filter.start
    
    view3d.show

    Vizkit.control replay
    Vizkit.exec
end
