ACLOCAL_AMFLAGS = -Im4

CFLAGS += -std=c99 ${SIMD_FLAGS} -g -O6 -Wall -pedantic -I src/common
LDFLAGS = ${LIBM}

bin_PROGRAMS = thorenc thordec

common_SOURCES = \
	src/common/common_block.c \
	src/common/common_frame.c \
	src/common/transform.c \
	src/common/intra_prediction.c \
	src/common/inter_prediction.c \
	src/common/common_kernels.c \
	src/common/snr.c \
	src/common/simd.c

EXTRA_DIST = \
	src/enc/putvlc.h \
	src/enc/strings.h \
	src/enc/encode_block.h \
	src/enc/encode_frame.h \
	src/enc/mainenc.h \
	src/enc/enc_kernels.h \
	src/enc/write_bits.h \
	src/enc/putbits.h \
	src/dec/maindec.h \
	src/dec/read_bits.h \
	src/dec/getvlc.h \
	src/dec/decode_frame.h \
	src/dec/decode_block.h \
	src/dec/getbits.h \
	src/common/snr.h \
	src/common/common_frame.h \
	src/common/intra_prediction.h \
	src/common/transform.h \
	src/common/types.h \
	src/common/simd/v128_intrinsics.h \
	src/common/simd/v64_intrinsics.h \
	src/common/simd/v128_intrinsics_c.h \
	src/common/simd/v128_intrinsics_arm.h \
	src/common/simd/v128_intrinsics_x86.h \
	src/common/simd/v64_intrinsics_arm.h \
	src/common/simd/v64_intrinsics_x86.h \
	src/common/simd/v64_intrinsics_c.h \
	src/common/common_kernels.h \
	src/common/inter_prediction.h \
	src/common/simd.h \
	src/common/common_block.h \
	src/common/global.h


thorenc_SOURCES = \
	src/enc/encode_block.c \
	src/enc/encode_frame.c \
	src/enc/mainenc.c \
	src/enc/putbits.c \
	src/enc/putvlc.c \
	src/enc/strings.c \
	src/enc/write_bits.c \
	src/enc/enc_kernels.c \
	$(common_SOURCES)

thordec_SOURCES = \
	src/dec/decode_block.c \
	src/dec/getbits.c \
	src/dec/getvlc.c \
	src/dec/maindec.c \
	src/dec/read_bits.c \
	src/dec/decode_frame.c \
	$(common_SOURCES)

dist_doc_DATA = README.md LICENSE \
		config_high_efficiency.txt \
		config_low_complexity.txt \
		config_RA_high_efficiency.txt \
		config_RA_low_complexity.txt
