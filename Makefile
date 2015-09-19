COMMAND = nvcc
FLAGS = -std=c++11 -shared -O3 -use_fast_math -Xcompiler -fPIC -Iinclude

all: inertia

inertia: harmonic_gpu.o harmonic_cpu.o
	mkdir -p lib
	$(COMMAND) $(FLAGS) obj/*.o -o inertia.so
	mv inertia.so lib

harmonic_gpu.o: src/harmonic/*.cu
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude/harmonic -c src/harmonic/*.cu
	mv *.o obj

harmonic_cpu.o: src/harmonic/*.cpp
	mkdir -p obj
	$(COMMAND) $(FLAGS) -Iinclude/harmonic -c src/harmonic/*.cpp
	mv *.o obj

clean:
	rm -rf lib
	rm -rf obj


