# Project Build Instructions

This project uses a `Makefile` to compile and link multiple C++ source files, with support for different platforms (macOS and Linux). Below are the instructions on how to use the Makefile to build and run the project.


main.cpp contains the source code for Part 3

part2.cpp contains the source code for Part 2
You can build the project by running the following command:

1. Build the Project

```
make
```
This will:

Compile the source files (main.cpp, part2.cpp, etc.).
Link them into an executable named main.

2. Link the specific files 

```
make part2

make main
```

3. Run the files

Once linked you can run them individually as 

```
./main

./part2
```

