#CPLUS_INCLUDE_PATH = /nix/store/zrs0wdqgcwr8jimk73c9gz29y5amxlsd-libcxx-19.1.0-dev/include/c++/v1 # ~/.guix-home/profile/include/c++/v1:~/.guix-home/profile/include/
# CPLUS_INCLUDE_PATH = ~/.guix-home/profile/include/c++/v1:~/.guix-home/profile/include/
#CPATH = /nix/store/zrs0wdqgcwr8jimk73c9gz29y5amxlsd-libcxx-19.1.0-dev/include/c++/v1

CXXFLAGS=-I/data/projects/yosys/backends/cxxrtl/runtime/ -Itracy/public -Igen -std=c++23 -pthread # -ggdb  -O3 -march=native -mtune=native
# FLAGS += -ggdb -fsanitize=address
FLAGS += -O3 -march=native -mtune=native --param=max-vartrack-size=1500000
# CXXFLAGS += -nostdinc++ -nodefaultlibs -isystem ~/.guix-home/profile/include/c++/v1
# FLAGS =	-mllvm -inline-threshold=10000 -mllvm -rotation-max-header-size=100000000
#FLAGS =	# -DTRACY_ENABLE -DTRACY_CALLSTACK=20 -O3 -march=native -mtune=native
CXXFLAGS += $(FLAGS)
LDFLAGS=-lc++ -lz $(FLAGS)
CXX=g++
# CXX=clang++
objs_opt = gen/router_top.opt ../cxxrtl_fst/cxxrtl_fst.opt tracy/public/TracyClient.opt
objs = sim.o
all_objs = $(objs) $(objs_opt)
heads = gen/router_top.h ../cxxrtl_fst/cxxrtl_fst.h mesh.cpp

.PHONY: run
run: sim
	./sim

sim: $(all_objs)
	$(CXX) $(LDFLAGS) $^ -o $@

%.opt: %.cpp $(heads)
	$(CXX) $(CXXFLAGS) -O3 -march=native -mtune=native -c -o $@ $<
#	CPLUS_INCLUDE_PATH=$(CPLUS_INCLUDE_PATH) $(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: %.cpp $(heads)
	$(CXX) $(CXXFLAGS) -Wall -Wextra -Wno-array-bounds -Wno-shift-count-overflow -Wno-sign-compare	 -c -o $@ $<

clean:
	rm  -rf sim $(all_objs)
