#include "checksum.h"
#include "rmax/onboard_ref.h"
#include "rmax/onboard.h"

/**
 * @brief datalinkCheckSumCompute() calculates and returns the checksum of a 
 * character buffer
 *
 * Calculates the checksum of a character buffer using a 32-bit Fletcher
 * checksum. Handles an odd number of bytes by calculating the checksum as if
 * there were an additional zero-byte appended to the end of the data.
 *
 * @param buf pointer to a character buffer array
 * @param byteCount the size in bytes of the character buffer
 */
unsigned int datalinkCheckSumCompute( unsigned char *buf, int byteCount ) {

    unsigned int sum1 = 0xffff;
    unsigned int sum2 = 0xffff;
    unsigned int tlen = 0;
    unsigned int shortCount = byteCount / sizeof(short);
    unsigned int oddLength  = byteCount % 2;

    /* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

    while( shortCount ) {
		/* 360 is the largest number of sums that can be performed without overflow */
        tlen = shortCount > 360 ? 360 : shortCount;
        shortCount -= tlen;
        do {
            sum1 +=  *buf++;
            sum1 += (*buf++ << 8);
            sum2 += sum1;
        } while (--tlen);

        /* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
        if( (oddLength==1) && (shortCount<1) ) {
            sum1 += *buf++;
            sum2 += sum1;
        }

        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
    }

    /* Second reduction step to reduce sums to 16 bits */
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);

    return( sum2 << 16 | sum1 );
}


/**
 * @brief datalinkCheckSumEncode sets the header checksum and payload checksum of a
 * character buffer to be sent as a datalink message
 *
 * @param buf pointer to a character buffer
 * @param byteCount size of the character buffer in bytes
 */
void datalinkCheckSumEncode( unsigned char *buf, int byteCount ) {

    struct datalinkHeader_ref *h = (struct datalinkHeader_ref *)buf;

	h->sync1 = DATALINK_SYNC0;
	h->sync2 = DATALINK_SYNC1;
	h->sync3 = DATALINK_SYNC2;

	h->messageSize = byteCount;

    h->hcsum = datalinkCheckSumCompute(  buf, sizeof( struct datalinkHeader_ref ) - sizeof( int )*2 );
    h->csum  = datalinkCheckSumCompute( &(buf[sizeof( struct datalinkHeader_ref )]),
                                  byteCount - sizeof( struct datalinkHeader_ref ) );

}
