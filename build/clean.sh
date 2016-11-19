#!/bin/bash

#
# Copyright (c) Shawn Singh, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman, Glen Berseth
# See license.txt for complete license.
#

BUILD_MODULE=$1

if [ "`which gmake 2> /dev/null`" == "" ]; then
    MAKE=make
else
    MAKE=gmake
fi


# remove Visual Studio 2008 files that may exist, only used if the
# user specified "cleanall win32"
if [ "$1" == "win32" ]; then
    echo "Cleaning Visual Studio 2008 files"
    rm -rf win32/Release
    rm -rf win32/Debug
    rm -f win32/steersuite.suo
    rm -f win32/steersuite.ncb

    rm -rf ../steertool/build/win32/Release
    rm -rf ../steertool/build/win32/ReleaseAVX
    rm -rf ../steertool/build/win32/Debug
    rm -rf ../steertool/build/win32/steertool.vcproj.*.user

    rm -rf ../external/glfw/win32/Release
    rm -rf ../external/build/win32/ReleaseAVX
    rm -rf ../external/glfw/win32/Debug
    rm -f ../external/glfw/win32/glfw.vcproj.*.user

    rm -rf ../steerbench/build/win32/Release
    rm -rf ../steerbench/build/win32/ReleaseAVX
    rm -rf ../steerbench/build/win32/Debug
    rm -f ../steerbench/build/win32/steerbench.vcproj.*.user

    rm -rf ../pprAI/build/win32/Release
    rm -rf ../pprAI/build/win32/ReleaseAVX
    rm -rf ../pprAI/build/win32/Debug
    rm -f ../pprAI/build/win32/pprAI.vcproj.*.user

    rm -rf ../simpleAI/build/win32/Release
    rm -rf ../simpleAI/build/win32/ReleaseAVX
    rm -rf ../simpleAI/build/win32/Debug
    rm -f ../simpleAI/build/win32/simpleAI.vcproj.*.user

    rm -rf ../steerlib/build/win32/Release
    rm -rf ../steerlib/build/win32/ReleaseAVX
    rm -rf ../steerlib/build/win32/Debug
    rm -f ../steerlib/build/win32/steerlib.vcproj.*.user

    rm -rf ../steersim/build/win32/Release
    rm -rf ../steersim/build/win32/ReleaseAVX
    rm -rf ../steersim/build/win32/Debug
    rm -f ../steersim/build/win32/steersim.vcproj.*.user

    rm -rf ../socialForcesAI/build/win32/Release
    rm -rf ../socialForcesAI/build/win32/ReleaseAVX
    rm -rf ../socialForcesAI/build/win32/Debug
    rm -f ../socialForcesAI/build/win32/sfAI.vcproj.*.user

    rm -rf ../collisionAI/build/win32/Release
    rm -rf ../collisionAI/build/win32/ReleaseAVX
    rm -rf ../collisionAI/build/win32/Debug
    rm -f ../collisionAI/build/win32/collisionAI.vcproj.*.user

    rm -rf ../searchAI/build/win32/Release
    rm -rf ../searchAI/build/win32/ReleaseAVX
    rm -rf ../searchAI/build/win32/Debug
    rm -f ../searchAI/build/win32/searchAI.vcproj.*.user
    
    rm -rf ../curveAI/build/win32/Release
    rm -rf ../curveAI/build/win32/ReleaseAVX
    rm -rf ../curveAI/build/win32/Debug
    rm -f ../curveAI/build/win32/curveAI.vcproj.*.user
    
    rm -rf ../util/build/win32/Release
    rm -rf ../util/build/win32/ReleaseAVX
    rm -rf ../util/build/win32/Debug
    rm -f ../util/build/win32/util.vcproj.*.user
    
    
    exit 0
fi

# This brute-force careful approach makes sure we don't delete
# potentially precious files that the user forgot inside directory.
echo "Cleaning bin/"
if [ -d bin/ ]; then
		if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steerbench" ]]; then
    	rm -f bin/steerbench
    fi
    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steersim" ]]; then
    	rm -f bin/steersim
    fi
    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steertool" ]]; then
    	rm -f bin/steertool
    fi
    rmdir bin/
fi

# This brute-force careful approach makes sure we don't delete
# potentially precious files that the user forgot inside directory.
echo "Cleaning lib/"
if [ -d lib/ ]; then
		
	if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steertool" ]]; then
    	rm -f lib/libsteer.so
    fi

	if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "util" ]]; then
    	rm -f lib/libutil.so
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steersim" ]]; then
    	rm -f lib/libsteersim.so
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "glfw" ]]; then
    	rm -f lib/libglfw.so
    	rm -f lib/libglfw.dylib
	fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "pprAI" ]]; then
    	rm -f lib/libpprAI.so
    	rm -f lib/pprAI.o
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "sfAI" ]]; then
    	rm -f lib/libsfAI.so
		rm -f lib/sfAI.o
    fi
    rmdir lib/
fi


# This brute-force careful approach makes sure we don't delete
# potentially precious files that the user forgot inside directory.
echo "Cleaning modules/"
if [ -d modules/ ]; then
		if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "simpleAI" ]]; then
    	rm -f modules/simpleAI.o
    fi
    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "pprAI" ]]; then
    	rm -f modules/pprAI.o
    fi
    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "sfAI" ]]; then
    	rm -f modules/sfAI.o
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "collisionAI" ]]; then
    	rm -f modules/collisionAI.o
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "searchAI" ]]; then
        rm -f modules/searchAI.o
    fi

    if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "curveAI" ]]; then
    	rm -f modules/curveAI.o
    fi
    rmdir modules/
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "glfw" ]]; then
	echo "Cleaning GLFW"
	pushd ../external/glfw > /dev/null
	$MAKE x11-clean > /dev/null
	$MAKE macosx-clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steerlib" ]]; then
	echo "Cleaning SteerLib"
	pushd ../steerlib/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "util" ]]; then
	echo "Cleaning Util"
	pushd ../util/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steerbench" ]]; then
	echo "Cleaning SteerBench"
	pushd ../steerbench/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steersim" ]]; then
	echo "Cleaning SteerSim"
	pushd ../steersim/build > /dev/null
	rm -f autogenerated/moc_*.cpp
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "steertool" ]]; then
	echo "Cleaning SteerTool"
	pushd ../steertool/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "simpleAI" ]]; then
	echo "Cleaning SimpleAI"
	pushd ../simpleAI/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "pprAI" ]]; then
	echo "Cleaning PPR AI"
	pushd ../pprAI/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "sfAI" ]]; then
	echo "Cleaning Social Forces"
	pushd ../socialForcesAI/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "collisionAI" ]]; then
	echo "Cleaning Collision AI"
	pushd ../collisionAI/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "searchAI" ]]; then
    echo "Cleaning Search AI"
    pushd ../searchAI/build > /dev/null
    $MAKE clean > /dev/null
    popd > /dev/null
fi

if [[ $BUILD_MODULE == "all" || $BUILD_MODULE == "curveAI" ]]; then
	echo "Cleaning Curve AI"
	pushd ../curveAI/build > /dev/null
	$MAKE clean > /dev/null
	popd > /dev/null
fi
