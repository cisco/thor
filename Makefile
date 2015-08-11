ENCODER_PROGRAM = build/Thorenc
DECODER_PROGRAM = build/Thordec

CFLAGS += -std=c99 -g -O6 -Wall -pedantic -I src/common
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
	src/common/common_block.c \
	src/common/common_frame.c \
	src/common/transform.c \
	src/common/intra_prediction.c \
	src/common/inter_prediction.c \
	src/common/common_kernels.c \
	src/common/snr.c \
	src/common/simd.c

ENCODER_SOURCES = \
	src/enc/encode_block.c \
	src/enc/encode_frame.c \
	src/enc/mainenc.c \
	src/enc/putbits.c \
	src/enc/putvlc.c \
	src/enc/strings.c \
	src/enc/write_bits.c \
	src/enc/enc_kernels.c \
	$(COMMON_SOURCES)

DECODER_SOURCES = \
	src/dec/decode_block.c \
	src/dec/getbits.c \
	src/dec/getvlc.c \
	src/dec/maindec.c \
	src/dec/read_bits.c \
	src/dec/decode_frame.c \
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

-include $(DEPS)

