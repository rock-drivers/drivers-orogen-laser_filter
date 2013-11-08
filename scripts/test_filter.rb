#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000

log.transformer_broadcaster.track(false) 
log.transformer_broadcaster.rename('foo')
log.name_service.deregister 'transformer_broadcaster'
#log.name_service.deregister 'local_planner'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'laser_filter::Task' => 'laser_filter', :valgrind => false, :output => nil do |p|

    laser_filter = Bundles::get 'laser_filter'
    
    laser_filter.apply_conf(['default', 'laser_back'], true)

    hokuyo_back = Bundles::get 'hokuyo_back'

    hokuyo_back.scans.connect_to(laser_filter.scan_samples, :type => :buffer, :size => 100)

    Bundles.transformer.setup( laser_filter )

    laser_filter.configure
    laser_filter.start

    Vizkit.control log

    boxes = laser_filter.filterBoxes
    
    pp boxes
    
    #laserFrameViz = view3d.createPlugin('RigidBodyStateVisualization')
    #laserFrameViz.resetModel(0.5)

    #laserViz = view3d.createPlugin('LaserScanVisualization')

    lBoxViz = Vizkit.default_loader.BoxVisualization
    rBoxViz = Vizkit.default_loader.BoxVisualization
    #Vizkit.display laser_filter.filterBoxes
    lBoxViz.updateData(boxes[0])
    rBoxViz.updateData(boxes[1])

    laserViz =  Vizkit.display laser_filter.filtered_scans , :widget => Vizkit.default_loader.LaserScanVisualization

    Vizkit.connect_port_to 'laser_filter', 'debug_laser_frame', :auto_reconnect => true, :pull => false, :update_frequency => 33 do |sample, name|
        laserViz.updateData(sample)
        sample
    end    

    
    
#     Vizkit.display heading_calc.heading_debug_deg

    Vizkit.exec() 
end


