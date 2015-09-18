COMMAND = nvcc
FLAGS = -std=c++11 -shared -O3 -use_fast_math -Xcompiler -fPIC -Iinclude

all: harmonic

nova: harmonic_gpu.o harmonic_cpu.o
	mkdir -p lib
	$(COMMAND) $(FLAGS) obj/*.o -o harmonic.so
	mv harmonic.so lib

harmonic_gpu.o: src/harmonic_gpu.cu
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude -c src/harmonic_gpu.cu
	mv *.o obj

harmonic_cpu.o: src/harmonic_cpu.cpp
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude -c src/harmonic_cpu.cpp
	mv *.o obj

clean:
	rm -rf lib
	rm -rf obj


