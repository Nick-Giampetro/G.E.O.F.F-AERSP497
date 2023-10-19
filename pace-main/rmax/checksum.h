#ifndef rmax_checksum_h
#define rmax_checksum_h


#if defined(__cplusplus)
extern "C"
{
#endif

unsigned int datalinkCheckSumCompute( unsigned char *buf, int byteCount );
void datalinkCheckSumEncode( unsigned char *buf, int byteCount );
unsigned char datalinkCheckSumComputeOld( unsigned char *buf, int byteCount );
void datalinkCheckSumEncodeOld( unsigned char *buf, int byteCount );

#if defined(__cplusplus)
}
#endif



#endif

