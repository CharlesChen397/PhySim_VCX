#!/usr/bin/env bash
# CMake 4.0+ 下，xmake-repo 中部分依赖仍声明较低 cmake_minimum_required，需提高策略下限。
# 详见 README「xmake FAQ」与 https://cmake.org/cmake/help/latest/envvar/CMAKE_POLICY_VERSION_MINIMUM.html
export CMAKE_POLICY_VERSION_MINIMUM="${CMAKE_POLICY_VERSION_MINIMUM:-3.5}"
exec xmake "$@"
