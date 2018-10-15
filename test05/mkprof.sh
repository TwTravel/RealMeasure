g++ -g -pg  -fprofile-arcs -ftest-coverage   -std=gnu++11 -I./depend -I./ rs-pointcloud.cpp  ./depend/imgui.cpp ./depend/imgui_draw.cpp ./depend/imgui_impl_glfw.cpp -o  align  -lrealsense2 -lutil -lboost_iostreams -lboost_system -lboost_filesystem -lglut -lGLU -lGL -lglfw -lrt -lpthread

