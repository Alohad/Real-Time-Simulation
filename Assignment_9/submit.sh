#!/bin/bash

echo "Trying to compile assignment9.cpp ..."
clang++ -std=c++11 assignment9.cpp simulation.cpp -lpthread -lgtest -lX11 -o run_simulation > /dev/null 2>&1
error=$?
if [ $error = 0 ]; then
	tar -czf assignment9.tar.gz assignment9.cpp assignment9.h
	echo "Success! You may now submit assignment9.tar.gz to Moodle."
else
	echo "ERROR: Compilation failed, you are not ready to submit."
fi
