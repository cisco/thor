# Thor Video Codec 

Implementation of [https://tools.ietf.org/html/draft-fuldseth-netvc-thor](https://tools.ietf.org/html/draft-fuldseth-netvc-thor)

## Build

Windows: Use Visual Studio with build/Thor.sln.

Mac/Linux:

    make -j8

Binaries will appear in the build/ directory.

## Usage

encoder:        Thorenc -cf config.txt

decoder:        Thordec str.bit out.yuv

