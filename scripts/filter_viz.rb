#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: filter_viz taskName"
    exit
end


#load log file 
taskName = ARGV[0]
Orocos::CORBA::max_message_size = 100000000


Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_asguardv3.rb'))


laser_filter = Bundles::get(taskName)
    
boxes = laser_filter.filterBoxes
    
pp boxes
    
#laserFrameViz = view3d.createPlugin('RigidBodyStateVisualization')
#laserFrameViz.resetModel(0.5)

#laserViz = view3d.createPlugin('LaserScanVisualization')

boxes.each do |i|
    boxViz = Vizkit.default_loader.BoxVisualization
    boxViz.updateData(i)
end

#lBoxViz = Vizkit.default_loader.BoxVisualization
#rBoxViz = Vizkit.default_loader.BoxVisualization
#Vizkit.display laser_filter.filterBoxes
#lBoxViz.updateData(boxes[0])
#rBoxViz.updateData(boxes[1])

laserViz =  Vizkit.display laser_filter.filtered_scans , :widget => Vizkit.default_loader.LaserScanVisualization

Vizkit.connect_port_to 'laser_filter_front', 'debug_laser_frame', :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
    #sample.orientation = Eigen::Quaternion::Identity()
    laserViz.updateData(sample)
    sample
end    
    
    
Vizkit.exec() 


