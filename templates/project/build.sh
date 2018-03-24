#!/bin/sh

cd .. && cd .. 
cd build/app/ && cmake .. && make -j4 && cd .. && cd .. && ./bin/lin-x86_64/application-GLFW
cd templates/project
