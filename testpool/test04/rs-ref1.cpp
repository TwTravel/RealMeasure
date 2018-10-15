// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include "c24bitmap.h"

#include "c256bitmap.h"
#include <librealsense2/rs.hpp>
#include <iostream>

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int global_count = 0;
int global_clock = clock();

C24BitMap CPic, gColorImg;

rs2_intrinsics  _color_intrin;
rs2_device* rscamera;
rs2_context _rs_ctx();

 

 

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
     window app(640, 480, "RealSense Pointcloud Example");
    // Construct an object to manage view state
     glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
     register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    
    rs2::config cfg;
	
    cfg.enable_stream(RS2_STREAM_COLOR, color_w, color_h, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480,  RS2_FORMAT_Z16, 30);
	
   // pipe.start(cfg);
	pipe.start( );
	//==========================================================

    while (app) // Application still alive?
	//while(1)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

		global_count ++;
        float timefps = float(global_count) *1000000.0/ float( clock() - global_clock);
        printf("processing fps is %i, %.3f\n", global_count, timefps);
	
	  
	    //########################################################################################
		int i,j, count;
         
		char filename[100];
	    sprintf(filename, "subpic/camera%04i.bmp", global_count);
		//alignPic.Save(filename);
		//printf("%i,%i\n", width, height);
		//gColorImg.Save(filename);
		
		//########################################################################################
        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points);
		// if(global_count> 200)
	    //   break;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
