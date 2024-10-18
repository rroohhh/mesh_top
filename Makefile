CPLUS_INCLUDE_PATH = ~/.guix-home/profile/include/c++/v1:~/.guix-home/profile/include/
CXXFLAGS=-I/data/projects/yosys/backends/cxxrtl/runtime/ -std=c++23 -Wall -Wextra -Wno-array-bounds -Wno-shift-count-overflow -Wno-sign-compare	-stdlib=libc++ -ggdb -fopenmp
LDFLAGS=-lc++ -lz -fopenmp
CXX=clang++
objs_opt = router_top.opt ../cxxrtl_fst/cxxrtl_fst.opt
objs = sim.o
all_objs = $(objs) $(objs_opt)

.PHONY: run
run: sim
	./sim

sim: $(all_objs)
	CPLUS_INCLUDE_PATH=$(CPLUS_INCLUDE_PATH) $(CXX) $(LDFLAGS) $^ -o $@

%.opt: %.cpp
	CPLUS_INCLUDE_PATH=$(CPLUS_INCLUDE_PATH) $(CXX) $(CXXFLAGS) -Ofast -march=native -mtune=native -c -o $@ $<
#	CPLUS_INCLUDE_PATH=$(CPLUS_INCLUDE_PATH) $(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.cpp
	CPLUS_INCLUDE_PATH=$(CPLUS_INCLUDE_PATH) $(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm  -rf sim $(all_objs)
