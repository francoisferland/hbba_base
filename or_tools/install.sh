#!/bin/sh

INCPATH=$1/include/or-tools
LIBPATH=$1/lib/or-tools

mkdir -p $LIBPATH
cp $PWD/lib/*.so $LIBPATH

mkdir -p $INCPATH/constraint_solver
cp $PWD/src/constraint_solver/*.h $INCPATH/constraint_solver
mkdir -p $INCPATH/base
cp $PWD/src/base/*.h $INCPATH/base
mkdir -p $INCPATH/util
cp $PWD/src/util/*.h $INCPATH/util

