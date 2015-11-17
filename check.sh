#!/bin/bash 

CONFIG=$1
FRAMES=$2
VALGRIND=$3
FILES=$4

VALGRIND_PREFIX="valgrind --leak-check=full --error-exitcode=123";
if [[ $VALGRIND == 0 ]]; then
    VALGRIND_PREFIX=""
fi

if [ -z $FRAMES ]; then
    FRAMES=3
fi

W=( 1920 1280 640 640 )
H=( 1080 720 480 360 )

if [ -z $FILES ]; then #generate some random files
    TEST_FILES=()
    for idx in $(seq 0 $((${#W[@]}-1))); do
        w=${W[$idx]}
        h=${H[$idx]}

        YUV_FILE_SIZE=$((${w}*${h}*${FRAMES}*3/2))
        FILE=rnd_test_tmp_${w}x${h}_30.yuv
        head --bytes ${YUV_FILE_SIZE} </dev/urandom > ${FILE} #generate random file

        TEST_FILES+=(${FILE})
    done
    SRCH="rnd_test_tmp*.yuv"
    PTH="."
else
    if [[ $FILES == *".yuv" ]]; then
        SRCH=$(basename "${FILES}")
        PTH=$(dirname "${FILES}")
    else
        SRCH="*.yuv"
        PTH=$FILES
    fi
fi

find ${PTH} -iname "${SRCH}" | while read f
do
    wh=$(echo ${f} | rev | cut -d'_' -f2 | rev)
    rc=$(echo ${f} | rev | cut -d'_' -f1 | rev)
    rc=$(echo ${rc}| cut -d'.' -f1)
    w=$(echo ${wh} | cut -d'x' -f1)
    h=$(echo ${wh} | cut -d'x' -f2)

    echo $w $h $rc $f $FRAMES
    ${VALGRIND_PREFIX} ./build/Thorenc \
                    -cf ${CONFIG} -width ${w} -height ${h} -if ${f} \
                    -of str_tmp.bit -rf rec_tmp.yuv -n ${FRAMES}

    if [ $? != 0 ]; then
        echo "Encoder error detected"
        exit
    fi

    ${VALGRIND_PREFIX} ./build/Thordec str_tmp.bit out_tmp.yuv

    if [ $? != 0 ]; then
        echo "Decoder error detected"
        exit
    fi

    cmpout=$(cmp out_tmp.yuv rec_tmp.yuv) #mismatch checking

    if [ $? != 0 ]; then
        FRAME_SIZE=$((${w}*${h}*3/2))
        ATBYTE=$(echo ${cmpout} | grep -o -E '[0-9]+' | head -1)
        mth=$(expr $ATBYTE / $FRAME_SIZE + 1)
        echo "Encoder/Decoder mismatch detected. Frame nr.:" ${mth}
        exit
    fi

done

#cleanup
rm str_tmp.bit;rm rec_tmp.yuv;rm out_tmp.yuv
if [ -z $FILES ]; then
    rm ${TEST_FILES[@]}
fi
