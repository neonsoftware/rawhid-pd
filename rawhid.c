// RAWHID Pd External.
//
// Author : Francesco Cervigni
//
// This external is derived from the following work :
// - Teensy RAWHID C example. Which contains hid_INDOWS.hpp, hid_WINDOWS.hpp, hid_WINDOWS.hpp in the
//    form of .c files.
// - hicuHeavyBox [the threading part] by Patrick Sebastien Coulombe, adapted by Michael Egger. GNU
//   GPL 2.0 www.gnu.org
// - hid : Hans-Christoph Steiner <hans@at.or.at>. GPL
// - comport : Winfried Ritsch. LGPL
//
// TODO :
// 1) Fix write operations.
// 2) Make threaded.

#include "hid.h"
#include "m_pd.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>


#if defined(OS_CYGWIN) || defined(OS_MINGW)
#include "hid_WINDOWS.hpp"
#elif defined(OS_linux) || defined(OS_GNU) || defined(OS_kFreeBSD)
#include "hid_LINUX.hpp"
#elif defined(OS_macosx)
#include "hid_MACOSX.hpp"
#endif

//#define DEBUG

/* clang-format off */
#ifdef  DEBUG
	#define DEBUG_POST(x) post x
	#define DEBUG_DUMP(b, l, m) for (int i = 0; i < l; i++) { post("%s [%d] : %d", m, i, b[i]); }
#else
	#define DEBUG_POST(x) do { } while (0)
	#define DEBUG_DUMP(b, l, m) do { } while (0)
#endif

#define BLOCK_SIZE 64
#define RAWHID_BUF_SIZE 16384

/* declare rawhid_class as a t_class type */
static t_class *rawhid_class;

/* this struct is the 'handle' the Pd core will have to the instance. The
   rawhid_new method initializes it with a pointer to this instance */
typedef struct _rawhid {
	t_object 	x_obj;
	t_int 		x_brandId;
	t_int 		x_productId;
	t_int 		x_isOpen;
	t_int 		x_deviceId;
	t_int 		x_packetSizeBytes;
	t_int 		x_packetsBuf;
	t_outlet *	x_data_outlet;
	unsigned char 	x_buf[BLOCK_SIZE];
	t_clock *	x_clock;
	double 		x_deltime;
	unsigned char *	x_inbuf;
	unsigned char *	x_outbuf;
	size_t 		x_inbuf_len;
	size_t 		x_outbuf_len;
	size_t 		x_outbuf_wr_index; /* offset to next free location in x_outbuf */
	size_t 		x_packets_to_recv;
} t_rawhid;

static void 	rawhid_close_device(t_rawhid *x);
static void 	rawhid_tick(t_rawhid *x);
static int  	write_serial(t_rawhid *x, unsigned char serial_byte);
static int  	write_serials(t_rawhid *x, unsigned char *serial_buf, size_t buf_length);
static void 	rawhid_float(t_rawhid *x, t_float f);
static void 	rawhid_list(t_rawhid *x, t_symbol *s, int argc, t_atom *argv);
static void 	rawhid_close_device(t_rawhid *x);
static void  	rawhid_open_device(t_rawhid *x, t_symbol *brandId, t_symbol *productId);
static void   	rawhid_poll(t_rawhid *x, t_float poll);
static void   	rawhid_packets(t_rawhid *x, t_float pockets);
static void * 	rawhid_new(void);
static void   	rawhid_free(t_rawhid *x);

/* clang-format on */

static void rawhid_tick(t_rawhid *x)
{
	size_t recv_pakts = 0;
	int recv_bytes = 0;

	DEBUG_POST(("[rawhid] polling. reading up to %d packets", x->x_packets_to_recv));

	while (x->x_isOpen && recv_pakts < x->x_packets_to_recv) {
		recv_bytes = rawhid_recv(0, x->x_inbuf, BLOCK_SIZE, 0);
		// post("[rawhid] receive loop #%d", i);

		if (recv_bytes > 0) {
			int j = 0;
			recv_pakts++;
			DEBUG_POST(("[rawhid] %d° packet received: %d bytes", recv_pakts, recv_bytes));
			for (j = 0; j < recv_bytes; j++) {
				outlet_float(x->x_data_outlet, (t_float)x->x_inbuf[j]);
			}

		} else if (recv_bytes < 0) {
			post("[rawhid] error reading, device went offline");
			rawhid_close_device(x);
			break;
		} else {
			DEBUG_POST(("[rawhid] no packets to read"));
			break;
		}
	}
	DEBUG_POST(("[rawhid] %i packets received. next polling in %.1f ms", recv_pakts, x->x_deltime));
	clock_delay(x->x_clock, x->x_deltime);
}

static int write_serial(t_rawhid *x, unsigned char serial_byte)
{
	if (!x->x_isOpen) {
		post("[rawhid] No device open");
		return 0;
	} else if (x->x_outbuf_wr_index < x->x_outbuf_len) {
		DEBUG_POST(("[rawhid] Adding float to buffer"));
		x->x_outbuf[x->x_outbuf_wr_index++] = serial_byte;
		return 1;
	}
	/* handle overrun error */
	post("[rawhid] buffer is full");
	return 0;
}

static int write_serials(t_rawhid *x, unsigned char *buf, size_t buf_len)
{
	size_t bytes_to_send = buf_len;

	if (!x->x_isOpen) {
		post("[rawhid] Serial port is not open");
		return 0;
	}

	/* if buf contains blocks of BLOCK_SIZE bytes, we send them entirely */
	while (bytes_to_send >= BLOCK_SIZE) {
		if (rawhid_send(0, buf, BLOCK_SIZE, 0) != BLOCK_SIZE) {
			post("[rawhid] Error. Out buffer is full. Cannot send.");
			return -1;
		}
		bytes_to_send -= BLOCK_SIZE;
		buf += BLOCK_SIZE;
	}

	/* if less than BLOCK_SIZE bytes are still inthe buffer, then padding them and send */
	if (bytes_to_send > 0) {
		unsigned char padded_buf[BLOCK_SIZE];
		memset(padded_buf, 0, BLOCK_SIZE);
		memcpy(padded_buf, buf, bytes_to_send);
		if (rawhid_send(0, padded_buf, BLOCK_SIZE, 0) != BLOCK_SIZE) {
			post("[rawhid] Error. Out buffer is full. Could not send block.");
			return -1;
		}
	}
	return buf_len;
}

static void rawhid_float(t_rawhid *x, t_float f)
{
	unsigned char serial_byte = ((int)f) & 0xFF; /* brutal conv */

	if (write_serial(x, serial_byte) != 1) {
		post("Write error, maybe TX-OVERRUNS on serial line");
	}
}

static void rawhid_list(t_rawhid *x, t_symbol *s, int argc, t_atom *argv)
{
	unsigned char temp_array[RAWHID_BUF_SIZE]; /* arbitrary maximum list length */
	int i, count;
	int result;

	count = argc;
	if (argc > RAWHID_BUF_SIZE) {
		post("[rawhid] truncated list of %d elements to %d", argc, count);
		count = RAWHID_BUF_SIZE;
	}
	for (i = 0; i < count; i++){
		temp_array[i] = ((unsigned char)atom_getint(argv + i)) & 0xFF; /* brutal conv */
	}	
	result = write_serials(x, temp_array, count);
}

static void rawhid_open_device(t_rawhid *x, t_symbol *brandId, t_symbol *productId)
{
	int bId = (int)strtol(brandId->s_name, NULL, 16);
	int pId = (int)strtol(productId->s_name, NULL, 16);
	if ((bId > 0) && (brandId->s_name[0] == '0') && (brandId->s_name[1] == 'x') && (pId > 0) &&
	    (productId->s_name[0] == '0') && (productId->s_name[1] == 'x')) {
		if (rawhid_open(1, bId, pId, 0xFFAB, 0x0200) > 0) {
			post("[rawhid] Device %s %s open", brandId->s_name, productId->s_name);
			x->x_brandId = bId;
			x->x_productId = pId;
			x->x_isOpen = 1;
			clock_delay(x->x_clock, x->x_deltime);
		} else {
			post("[rawhid] Impossible to open device %s %s", brandId->s_name,
			     productId->s_name);
		}
	} else {
		post("[rawhid] Invalid input for open operation. (e.g. open 0x002a 0x160c)",
		     brandId->s_name, productId->s_name);
	}
}

static void rawhid_close_device(t_rawhid *x)
{
	if (x->x_isOpen) {
		rawhid_close(0);
		x->x_isOpen = 0;
		clock_unset(x->x_clock);
		post("[rawhid] Device 0x%04x 0x%04x closed", x->x_brandId, x->x_productId);
	} else {
		post("[rawhid] There are no open devices to close.");
	}
}

static void rawhid_poll(t_rawhid *x, t_float poll)
{
	post("[rawhid] Polling set to %.01fms", poll);
	x->x_deltime = poll;
}

static void rawhid_packets(t_rawhid *x, t_float packets)
{
	x->x_packets_to_recv = (size_t)packets;
	post("[rawhid] Packets to receive per poll set to %d", x->x_packets_to_recv);
}

/* the 'constructor' method which defines the t_rawhid struct for this
   instance and returns it to the caller which is the Pd core */
static void *rawhid_new(void)
{
	post("[rawhid] Starting ...");
	t_rawhid *x = (t_rawhid *)pd_new(rawhid_class);

	x->x_inbuf = getbytes(RAWHID_BUF_SIZE);
	x->x_outbuf = getbytes(RAWHID_BUF_SIZE);
	if (NULL == x->x_inbuf || NULL == x->x_outbuf) {
		pd_error(x, "[rawhid] fatal error : unable to allocate buffer");
		return 1;
	}
	x->x_inbuf_len = RAWHID_BUF_SIZE;
	x->x_outbuf_len = RAWHID_BUF_SIZE;
	x->x_outbuf_wr_index = 0;
	x->x_data_outlet = outlet_new(&x->x_obj, &s_float);
	x->x_packets_to_recv = 1; // default = 1
	/* Since 10ms is also the default poll time for most HID devices,
	 * and it seems that for most uses of [comport] (i.e. arduinos and
	 * other serial ports 115200 baud or less) that 10ms polling is
	 * going to give the data as fast as 1ms polling with a lot less
	 * CPU time wasted. */
	x->x_deltime = 1000;
	x->x_clock = clock_new(x, (t_method)rawhid_tick);
	post("[rawhid] Successfully started");
	return (void *)x;
}

static void rawhid_free(t_rawhid *x)
{
	post("[rawhid] free rawhid...");
	clock_unset(x->x_clock);
	clock_free(x->x_clock);
	rawhid_close_device(x);
	freebytes(x->x_inbuf, x->x_inbuf_len);
	freebytes(x->x_outbuf, x->x_outbuf_len);
}

/* This method is the only one the Pd core expects to be present */

#if defined(_LANGUAGE_C_PLUS_PLUS) || defined(__cplusplus)
extern "C" {
#endif
void rawhid_setup(void)
{
	/* this registers the 'rawhid' class. The 'rawhid_new' method will be executed at each
	instantiation. */
	rawhid_class = class_new(gensym("rawhid"), (t_newmethod)rawhid_new, 0, sizeof(t_rawhid),
				 CLASS_DEFAULT, 0);

	class_addfloat(rawhid_class, (t_method)rawhid_float);
	class_addlist(rawhid_class, (t_method)rawhid_list);
	class_addmethod(rawhid_class, (t_method)rawhid_open_device, gensym("open"), A_DEFSYM,
			A_DEFSYM, 0);
	class_addmethod(rawhid_class, (t_method)rawhid_poll, gensym("poll"), A_FLOAT, 0);
	class_addmethod(rawhid_class, (t_method)rawhid_packets, gensym("packets"), A_FLOAT, 0);
	class_addmethod(rawhid_class, (t_method)rawhid_close_device, gensym("close"), 0);
}
#if defined(_LANGUAGE_C_PLUS_PLUS) || defined(__cplusplus)
}
#endif
