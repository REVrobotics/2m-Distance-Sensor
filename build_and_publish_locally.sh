#! /bin/bash
set -e

LIB=DistanceSensor
BUILD_YEAR=2026
VENDOR_GENERATED_DIR=./vendordeps/*
BUILD_REPO_DIR=./build/repos
RELEASE_DIR=$BUILD_REPO_DIR/releases/com/revrobotics/frc
MOVE_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/maven/com/revrobotics/frc
VENDOR_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/vendordeps

echo "*** Building ***"
./gradlew build -PreleaseMode

echo "*** Generating maven directories ***"
./gradlew publish -PreleaseMode

echo "*** Ensuring destination maven directories exist ***"
mkdir -p $MOVE_DIR

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/$LIB-cpp $MOVE_DIR
cp -r $RELEASE_DIR/$LIB-driver $MOVE_DIR
cp -r $RELEASE_DIR/$LIB-java $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r $VENDOR_GENERATED_DIR $VENDOR_DIR
