# Thor Video Codec 

Implementation of [https://tools.ietf.org/html/draft-fuldseth-netvc-thor](https://tools.ietf.org/html/draft-fuldseth-netvc-thor)

## Build

Windows: Use Visual Studio with build/Thor.sln.

Mac/Linux:

    make -j8

Binaries will appear in the build/ directory.

## Usage

encoder:        Thorenc -cf config.txt -if in.yuv -of str.bit -rf out.yuv -qp N -width [width] -height [height] -f [framerate] -stat out.stat -qp [quant] -n [num frames]

A y4m file can be provided for input, and it will override width, height and framerate values given on the command-line.

decoder:        Thordec str.bit out.dec.yuv

