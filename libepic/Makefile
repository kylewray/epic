COMMAND = nvcc
FLAGS = -std=c++11 -shared -O3 -use_fast_math -Xcompiler -fPIC -Iinclude

all: epic

epic: harmonic_gpu.o harmonic_cpu.o
	mkdir -p lib
	$(COMMAND) $(FLAGS) obj/*.o -o libepic.so
	mv libepic.so lib

harmonic_gpu.o: src/harmonic/*.cu
	mkdir -p obj
	$(COMMAND) $(FLAGS) -c src/harmonic/*.cu
	mv *.o obj

harmonic_cpu.o: src/harmonic/*.cpp
	mkdir -p obj
	$(COMMAND) $(FLAGS) -c src/harmonic/*.cpp
	mv *.o obj

clean:
	rm -rf lib
	rm -rf obj

install:
	mkdir -p ros/catkin_ws/src/epic/libepic/lib
	cp lib/libepic.so ros/catkin_ws/src/epic/libepic/lib/libepic.so
	cp -r include ros/catkin_ws/src/epic/libepic/include

