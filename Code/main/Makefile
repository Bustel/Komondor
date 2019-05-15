COST = ../COST/cxx
MAIN = komondor_main  
CC = g++
FLAGS = -Wall -Werror -g
INCLUDE = `pkg-config --cflags glib-2.0`
LIB = `pkg-config --libs glib-2.0`

DEPS = ../COST/cost.h
DEPS += ../list_of_macros.h
DEPS += agent.h
DEPS += central_controller.h
DEPS += compcxx_komondor_main.h
DEPS += node.h
DEPS += traffic_generator.h
DEPS += $(shell find ../structures -type f)
DEPS += $(shell find ../methods -type f)
DEPS += $(shell find ../learning_modules -type f)


%.cxx: %.cc $(DEPS) 
	$(COST) $<

$(MAIN): $(MAIN:=.cxx)
	$(CC) $(FLAGS) $(INCLUDE) -g -o $(MAIN) $(MAIN:=.cxx) $(LIB)

all: $(MAIN)


.PHONY: clean
clean:
	rm -rf $(MAIN) $(MAIN:=.cxx)
