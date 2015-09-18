COMMAND = nvcc
FLAGS = -std=c++11 -shared -O3 -use_fast_math -Xcompiler -fPIC -Iinclude

all: harmonic

harmonic: harmonic_gpu.o harmonic_cpu.o
	mkdir -p lib
	$(COMMAND) $(FLAGS) obj/*.o -o harmonic.so
	mv harmonic.so lib

harmonic_gpu.o: src/*.cu
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude -c src/*.cu
	mv *.o obj

harmonic_cpu.o: src/*.cpp
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude -c src/*.cpp
	mv *.o obj

clean:
	rm -rf lib
	rm -rf obj


