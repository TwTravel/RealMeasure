// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include "c24bitmap.h"

#include "c256bitmap.h"
#include <librealsense2/rs.hpp>
#include <iostream>

#define MIN_(a, b) ((a) < (b)) ? (a) : (b)))
#define MAX_(a, b) ((a) > (b)) ? (a) : (b)))

#include  "rd_info.h"
#include  "send_msg.h"

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int global_count = 0;
int global_clock = clock();

C24BitMap CPic, gColorImg;

rs2_intrinsics  _color_intrin;
rs2_device* rscamera;
rs2_context _rs_ctx();


void sendposition(Pt3d&cube_center)
{
	char message_buffer[200];
    memset(message_buffer, 0, 200);
    sprintf(message_buffer,"%.4lf,%.4lf,%.4lf",
    cube_center.X , cube_center.Y , cube_center.Z );

    SendSystemMsg(message_buffer, strlen(message_buffer));
}

void adjust_image(C24BitMap & CPic1, C24BitMap & CPic2)
{
  //CPic1.Load(argv[1]);
  CPic2.FormatF(CPic1.Width, CPic1.Height);
  
  int i,j;
  for( i = 0 ; i < CPic1.Width; i++)
     for( j = 0; j < CPic1.Height; j++)
	 {
		 C24PixVal Pix1 = get_pix_color(CPic1, i, j);
		 C24PixVal Pix2 = get_pix_color(CPic2, i, CPic1.Height -1 - j);
		 *Pix2.r = *Pix1.b;
		 *Pix2.g = *Pix1.g;
		 *Pix2.b = *Pix1.r;
	 }
}

//int color_w(960);
//int color_h(540);

int color_w(640);
int color_h(480);

double gCAMERA_CENTER_X_POS, 
       gCAMERA_CENTER_Y_POS, 
	   gCAMERA_CENTER_X_SCALE, 
	   gCAMERA_CENTER_Y_SCALE;

/*
(gdb) p intr.fx
$2 = 618.547668
(gdb) p intr.fy
$3 = 618.547791
(gdb) p intr.ppx
$4 = 310.073059
(gdb) p intr.ppy
$5 = 250.001282
*/

void Map3dPt2XY(double X, double Y, double Z,
                int &x2d, int &y2d )
{
   x2d = double( X/Z * gCAMERA_CENTER_X_SCALE ) + gCAMERA_CENTER_X_POS;
   y2d = double( Y/Z * gCAMERA_CENTER_Y_SCALE ) + gCAMERA_CENTER_Y_POS;
} 

 
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)

{

   const int w = texture.get_width(), h = texture.get_height();
   int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
   int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);
   int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
   const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
   return std::tuple<uint8_t, uint8_t, uint8_t>(

  texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}


void RGB2HSV(double r, double g, double b, double &h, double &s,double &v)
{
   double themin,themax,delta;
   //;
   themin = std::min( r, std::min( g, b));
   themax = std::max( r, std::max( g, b));
   delta = themax - themin;
    v = themax;
    s = 0;
   if (themax > 0)
       s = delta / themax;
    h = 0;
   if (delta > 0) {
      if (themax ==  r && themax !=  g)
          h += ( g -  b) / delta;
      if (themax ==  g && themax !=  b)
          h += (2 + ( b -  r) / delta);
      if (themax ==  b && themax !=  r)
          h += (4 + ( r -  g) / delta);
       h *= 60;
   }
   //printf("%lf, %lf\n",v,s);
   //return(h);
}



void GetGrayImage(C24BitMap&CPic,C256BitMap &GPic)
{
	int i, j;
	if(GPic.Width!= CPic.Width)
		GPic.FormatF(CPic.Width, CPic.Height);
	
	for (i = 0; i< CPic.Width; i++)
	 for (j = 0; j < CPic.Height; j++)
	 {
		 C24PixVal  Pix = get_pix_color(CPic, i, j);
		 double val, val_min, val_max, R, G, B;
		 R = *Pix.r; G = *Pix.g; B = *Pix.b;
		 double  h, s, v;
		 RGB2HSV(R, G, B, h, s, v);
        //&& ( s > 128)
		s *= 255.0;
		//if( v > 40 && s > 128 )
             // if( v > 40 && s > 100 && ( h <10 ||(h>110&&h<160) ))
        if(( v < 150 && s > 200 && (h>190&&h<220) ))
			val = 255;
		else
			val = 0;

	     if(val > 128) 
			 val = 0;
		 else
			 val = 255;/**/
		 
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
	 //GPic.Save("test01.bmp");
}





void draw_3d_points(C24BitMap & alignPic, const rs2::video_frame& color , 
                     rs2::points points, double cx, double cy, double fx,double fy)
{
	//int i;
	auto vertices   = points.get_vertices();
	auto texcoords = points.get_texture_coordinates();
	alignPic.FormatF(color_w , color_h);
	
	gCAMERA_CENTER_X_POS = cx; 
    gCAMERA_CENTER_Y_POS = cy; 
	gCAMERA_CENTER_X_SCALE = fx; 
	gCAMERA_CENTER_Y_SCALE = fy;
	
	uint8_t* p_color_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(color.get_data()));
	int w = color_w;
	int h = color_h;
	
	
	double center_count = 0;
	double center_xx    = 0;
	double center_yy    = 0;
	int  i,j;
	
	Pt3d center_pos;
	center_pos.X = center_pos.Y = center_pos.Z =0;
    for (i = 0; i < points.size(); ++i)
	{
		double x,y,z;
		x = vertices[i].x;
        y = vertices[i].y;
        z = vertices[i].z;
		int x2d,y2d;
		Map3dPt2XY(x, y, z, x2d, y2d);
		x2d = BOUND(x2d, 0, color_w - 1);
		y2d = BOUND(y2d, 0, color_h - 1);
		C24PixVal Pix = get_pix_color(alignPic, x2d, y2d);
		
		x2d = std::min(std::max(int(texcoords[i].u*w + .5f), 0), w - 1);
		y2d = std::min(std::max(int(texcoords[i].v*h + .5f), 0), h - 1);
		
		char* ptr =  (char*)&p_color_frame[x2d * 3 + y2d * 640 * 3];
		
		/*std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);
		*Pix.r = std::get<0>(current_color);
		*Pix.g = std::get<1>(current_color);
		*Pix.b = std::get<2>(current_color);*/
		
		*Pix.r = ptr[0];//std::get<0>(current_color);
		*Pix.g = ptr[1];//std::get<1>(current_color);
		*Pix.b = ptr[2];//std::get<2>(current_color);
		
		double r, g, b, h, s, v;
		r = *Pix.r; 
		g = *Pix.g;
		b = *Pix.b;
		
		RGB2HSV( r, g, b, h, s, v);
		s *= 255.0;
		
		//if(((*Pix.b) > 110) && (*Pix.g <80) && (*Pix.r <80))
		if(( (v < 100 && v >20 ) && s > 100 && ( h > 140 && h < 150 ) ))
		//if( ( h > 190 && h < 220) )	
		{
		    ptr[0] = 255;
			ptr[1] = 0;
			ptr[2] = 0;
			
			 center_count ++;
	         center_xx += x2d;
	         center_yy += y2d;
			 
			 center_pos.X += x;
			 center_pos.Y += y;
			 center_pos.Z += z;
		}
		else
		{
			ptr[0] = 0;
			ptr[1] = 255;
			ptr[2] = 0;
		}/**/
		
		*Pix.r = ptr[0];//std::get<0>(current_color);
		*Pix.g = ptr[1];//std::get<1>(current_color);
		*Pix.b = ptr[2];//std::get<2>(current_color);
	}
	
	if(center_count>40)
	{
	center_xx /= center_count;
	center_yy /= center_count;

	center_pos.X /= center_count;
	center_pos.Y /= center_count;
	center_pos.Z /= center_count;
	
	center_pos  = Tranform2RobotAxis( center_pos );
	
     Loopi(10)
        Loopj(10)
		{
			int xx,yy;
			
			xx = center_xx - 5 +i;
			yy = center_yy - 5 +j; 

			char* ptr =  (char*)&p_color_frame[xx * 3 + yy * 640 * 3];
			
			ptr[0] = 255;
			ptr[1] = 255;
			ptr[2] = 0;
		}	
    }		
}

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
	
    cfg.enable_stream(RS2_STREAM_COLOR, color_w, color_h, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480,  RS2_FORMAT_Z16, 60);
	
    //pipe.start(cfg);
	pipe.start( cfg );
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
        CPic.FormatF(color_w, color_h);
        gColorImg.FormatF(color_w, color_h);
        count = CPic.LineWidth * CPic.Height;
        
		uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(color.get_data()));
		
		//auto frames = f.as<rs2::frameset>();
		auto color_frame = frames.first_or_default(RS2_STREAM_COLOR);
		rs2_intrinsics intr = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); 
		C24BitMap  alignPic;
		//rs2::points points_;
		
       
		draw_3d_points( alignPic, color, points, intr.ppx, intr.ppy, intr.fx,intr.fy);
		 
		 
		int width, height;
		width  = color_w;
		height = color_h;
		
		for(i=0; i < count; i++)
		   CPic.Buffer[i] = p_other_frame[i];
	    
		//adjust_image(CPic, gColorImg);
		  /**/
		char filename[100];
	    sprintf(filename, "subpic1/camera%04i.bmp", global_count);
		//alignPic.Save(filename);
		//gColorImg.Save(filename);
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
