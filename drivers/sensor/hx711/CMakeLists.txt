cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hx711)

target_sources(app PRIVATE src/hx711.c)
target_include_directories(app PRIVATE src)