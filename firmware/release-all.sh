#!/bin/sh

echo
echo This script will release a new version of the 850C, 860C and SW102 firmwares
echo 
echo Please enter the new version number, e.g. 0.5.1

read VERSION

RELEASE_FOLDER=${PWD}/releases/$VERSION

if [ -d "$RELEASE_FOLDER" ]; then
	echo Removing existing release folder: $RELEASE_FOLDER
	echo
	rm -R $RELEASE_FOLDER
fi

mkdir -p $RELEASE_FOLDER

cd SW102
make -f Makefile clean_project
make -f Makefile generate_dfu_package VERSION=$VERSION
cp _release/sw102-otaupdate-xxx.zip $RELEASE_FOLDER/sw102-otaupdate-$VERSION.zip
cd ..

cd 860C_850C/src/
# standard version 850C
make -f Makefile clean
rm -R ../common/src
make -f Makefile VERSION=$VERSION
cp main.bin $RELEASE_FOLDER/850C_v$VERSION.bin

# version 850C bootloader
make -f Makefile clean
rm -R ../common/src
make -f Makefile VERSION=$VERSION DISPLAY_VERSION="850C_BOOTLOADER"
cp main.bin $RELEASE_FOLDER/850C_v$VERSION-bootloader.bin

# version 850C_2021 bootloader
make -f Makefile clean
rm -R ../common/src
make -f Makefile VERSION=$VERSION DISPLAY_VERSION="850C_2021_BOOTLOADER"
cp main.bin $RELEASE_FOLDER/850C_2021-v$VERSION-bootloader.bin

# version 860C bootloader
make -f Makefile clean
rm -R ../common/src
make -f Makefile VERSION=$VERSION DISPLAY_VERSION="860C_BOOTLOADER"
cp main.bin $RELEASE_FOLDER/860C_v$VERSION-bootloader.bin

# version 860C_V1.2 bootloader
make -f Makefile clean
rm -R ../common/src
make -f Makefile VERSION=$VERSION DISPLAY_VERSION="860C_V12_BOOTLOADER"
cp main.bin $RELEASE_FOLDER/860C_V12-v$VERSION-bootloader.bin
cd ../..

echo 
echo Done! If the build went correctly, find the files on: $RELEASE_FOLDER


