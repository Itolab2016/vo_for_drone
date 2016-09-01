drone_Rt:drone_Rt.cpp Makefile
	g++ -o drone_Rt drone_Rt.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

