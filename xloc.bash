#!/bin/bash

# count the lines of code in the src directory, excluding some directories
cloc src --exclude-dir=json_struct,rosboard,vision_opencv,ArcternBase,dspatch --include-lang=C++,"C/C++ Header",Python