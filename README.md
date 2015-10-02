# Thor Video Codec 

Implementation of [https://tools.ietf.org/html/draft-fuldseth-netvc-thor](https://tools.ietf.org/html/draft-fuldseth-netvc-thor)

## Build

Windows: Use Visual Studio with build/Thor.sln.

Mac/Linux:

    make -j8

Binaries will appear in the build/ directory.

## Usage


encoder:        Thorenc -if inputfile -of outputfile [-options..]


(Inputfile format has to be .y4m or .yuv)


decoder:        Thordec inputfile outputfile



##Options:

-cf	Config file in which explictly define the parameters of encoder
