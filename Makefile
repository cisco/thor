ENCODER_PROGRAM = build/Thorenc
DECODER_PROGRAM = build/Thordec

CFLAGS += -std=c99 -g -O3 -Wall -pedantic -I common
LDFLAGS = -lm

ifeq ($(ARCH),neon)
        CFLAGS += -mfpu=neon
endif

ifeq ($(ARCH),ssse3)
        CFLAGS += -mssse3
endif

ifeq ($(ARCH),sse4)
        CFLAGS += -msse4
endif


COMMON_SOURCES = \
	common/common_block.c \
	common/common_frame.c \
	common/transform.c \
	common/intra_prediction.c \
	common/inter_prediction.c \
	common/common_kernels.c \
	common/snr.c \
	common/simd.c \
        common/temporal_interp.c \
        common/wt_matrix.c


ENCODER_SOURCES = \
	enc/encode_block.c \
	enc/encode_frame.c \
	enc/mainenc.c \
	enc/putbits.c \
	enc/putvlc.c \
	enc/strings.c \
	enc/write_bits.c \
	enc/enc_kernels.c \
	enc/rc.c \
	$(COMMON_SOURCES)

DECODER_SOURCES = \
	dec/decode_block.c \
	dec/getbits.c \
	dec/getvlc.c \
	dec/maindec.c \
	dec/read_bits.c \
	dec/decode_frame.c \
	$(COMMON_SOURCES)

ENCODER_OBJECTS = $(ENCODER_SOURCES:.c=.o)
DECODER_OBJECTS = $(DECODER_SOURCES:.c=.o)
OBJS = $(ENCODER_OBJECTS) $(DECODER_OBJECTS)
DEPS = $(OBJS:.o=.d)


.PHONY = clean

all: $(ENCODER_PROGRAM) $(DECODER_PROGRAM)

$(ENCODER_PROGRAM): $(ENCODER_OBJECTS)
	$(CC) -o $@ $(ENCODER_OBJECTS) $(LDFLAGS)

$(DECODER_PROGRAM): $(DECODER_OBJECTS)
	$(CC) -o $@ $(DECODER_OBJECTS) $(LDFLAGS)


# Build object files. In addition, track header dependencies.
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
	@$(CC) -MM $(CFLAGS) $*.c > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

clean:
	rm -f $(ENCODER_OBJECTS) $(DECODER_OBJECTS) $(DEPS)

cleanall: clean
	rm -f $(ENCODER_PROGRAM) $(DECODER_PROGRAM)

check: all
	# Usage : 
	# 	make check test-config=.. test-frames-count=.. test-valgrind=.. test-files=..
	#
	# Test performs (valgrind) encode -> (valgrind) decode -> 
	# 	compare encoder reconstructed yuv - decoder output yuv
	#
	# test-config - encoder config file
	# test-frames-count - how many frames to use from the input files
	# test-valgrind - pass through valgrind or not
	# test-files (optional) - test input files. If not specified, will generate a
	# 	few random yuv files for a few resolutions. If specify directory, will use all yuv files
	# 	in that directory. If specify explicitly yuv file, will use that file. 
	# 	Filename format must be name_WIDTHxHEIGHT_RATE.yuv
	#
	# Examples:
	#	 make check test-config=config_HDB_low_complexity.txt test-frames-count=10 test-valgrind=0 test-files=rnd_test_tmp_640x480_30.yuv
	#	 make check test-config=config_HDB_low_complexity.txt test-frames-count=5 test-valgrind=0 test-files=.
	#	 make check test-config=config_HDB_low_complexity.txt test-frames-count=2 test-valgrind=1 
	#
	./check.sh $(test-config) $(test-frames-count) $(test-valgrind) $(test-files)

-include $(DEPS)

