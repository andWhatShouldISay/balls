#!/bin/bash
g++ -o balls main.cpp image_helper.o SOIL.o stb_image_aug.o image_DXT.o -lglut -lGL

