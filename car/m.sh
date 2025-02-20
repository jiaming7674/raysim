g++ main.cpp car.cpp icollidable.cpp -lpthread -I/ucrt64/include -L/ucrt64/lib -lraylib -lopengl32 -lwinmm -lgdi32 -Wl,--subsystem,windows -o run

./run