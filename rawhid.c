// RAWHID Pd External. 
//  
// Author : Francesco Cervigni
//
// This external is derived from the following work :
// - Teensy RAWHID C example. Which contains hid_INDOWS.hpp, hid_WINDOWS.hpp, hid_WINDOWS.hpp in the form of .c files. 
// - hicuHeavyBox [the threading part] by Patrick Sebastien Coulombe, adapted by Michael Egger. GNU GPL 2.0 www.gnu.org
// - hid : Hans-Christoph Steiner <hans@at.or.at>. GPL
// - comport : Winfried Ritsch. LGPL
//
// TODO : 
// 1) Fix write operations.  
// 2) Make threaded.

#include "m_pd.h" 

#include "string.h"
#include "stdio.h"
#include "errno.h"

#if defined(OS_CYGWIN) || defined(OS_MINGW)
	#include "hid_WINDOWS.hpp"
#elif defined(OS_linux) || defined(OS_GNU) || defined(OS_kFreeBSD)
	#include "hid_LINUX.hpp"
#elif defined(OS_macosx)
	#include "hid_MACOSX.hpp"
#endif

//#define DEBUG_FILE
//#define DEBUG_COM

/* declare rawhid_class as a t_class type */
static t_class *rawhid_class;


#define RAWHID_BUF_SIZE 16384 

/* this struct is the 'handle' the Pd core will have to the instance. The
   rawhid_new method initializes it with a pointer to this instance */
typedef struct _rawhid 
{
	t_object          x_obj;
	t_int		      x_brandId;
	t_int 	      x_productId;
	t_int		      x_isOpen;
	t_int		      x_deviceId;
	t_int		      x_packetSizeBytes;
	t_int		      x_packetsBuf;
	t_outlet *		  x_data_outlet;
	char              x_buf[64];
	t_clock *         x_clock;
	double            x_deltime;
	unsigned char *   x_inbuf;
	unsigned char *   x_outbuf;
	int               x_inbuf_len; 
	int               x_outbuf_len; 
  int               x_outbuf_wr_index; /* offset to next free location in x_outbuf */
	FILE *            x_debug_file_out;
	FILE *            x_debug_file_in;
	t_int		      x_packets_to_recv;
} t_rawhid;

static void rawhid_close_device(t_rawhid *x);
static void rawhid_tick(t_rawhid *x);
static int write_serial(t_rawhid *x, unsigned char  serial_byte);
static int write_serials(t_rawhid *x, unsigned char *serial_buf, int buf_length);
static void rawhid_float(t_rawhid *x, t_float f);
static void rawhid_list(t_rawhid *x, t_symbol *s, int argc, t_atom *argv);
static void rawhid_close_device(t_rawhid *x);
static void rawhid_open_device(t_rawhid* x, t_symbol *brandId, t_symbol *productId); 
static void rawhid_poll(t_rawhid* x, t_float poll); 
static void rawhid_packets(t_rawhid* x, t_float pockets); 
static void *rawhid_new(void);
static void rawhid_free(t_rawhid *x);

// ------------------------------------------------------------------------------


static void rawhid_tick(t_rawhid *x)
{
	int packets_received = 0 ;

	if ( x->x_isOpen) 
	{
		int tot_recv_bytes = 0;
		int recv_bytes = 0;

		while ( packets_received < x->x_packets_to_recv )
		{
       		#ifdef DEBUG_COM
				post("\n[rawhid] %d° iteration \n", packets_received);
       		#endif

			recv_bytes = 0;
      		
      		#ifdef DEBUG_FILE
    			recv_bytes = fread(x->x_inbuf, sizeof(char), 64, x->x_debug_file_in);
      		#else
				recv_bytes = rawhid_recv(0, x->x_inbuf, 64, 0);
      		#endif

			// post("[rawhid] receive loop #%d", i);

			if (recv_bytes != 0 )
			{
				if (recv_bytes > 0) 
				{
					packets_received++;
	          		#ifdef DEBUG_COM
						post("\n[rawhid] received %d bytes. %d° packet \n", recv_bytes, packets_received);
	          		#endif
					for (int j=0; j < recv_bytes; j++) 
					{
						outlet_float(x->x_data_outlet, (t_float) x->x_inbuf[j]);
					}
					
				}
				else if (recv_bytes < 0) 
				{
					post("\n[rawhid] error reading, device went offline\n");
					rawhid_close_device(x);
					break;
				}
			}
			else
			{
				#ifdef DEBUG_COM
				post("\n[rawhid] Received nothing. Exit\n");
       			#endif

				break;
			}
		}
	}

  #ifdef DEBUG_COM
	  post("[rawhid] New clock in %.1f ms. Readed %i packets", x->x_deltime, packets_received);
	#endif
  clock_delay(x->x_clock, x->x_deltime);
}


static int write_serial(t_rawhid *x, unsigned char  serial_byte)
{
    if( ! x->x_isOpen)
    {
        post ("[rawhid] No device open");
        return 0;         
    }
    else if(x->x_outbuf_wr_index < x->x_outbuf_len)
    {
    	#ifdef DEBUG_COM
        	post ("[rawhid] Adding float to buffer");
        #endif
        x->x_outbuf[x->x_outbuf_wr_index++] = serial_byte;
        return 1;
    }    
    /* handle overrun error */
    post ("[rawhid] buffer is full");
    return 0;
}

static int write_serials(t_rawhid *x, unsigned char *serial_buf, int buf_length)
{
    if( ! x->x_isOpen )
    {
        post ("[rawhid] Serial port is not open");
        return 0;         
    }

    int sent_bytes = 0;

    #ifdef DEBUG_COM
    	post ("[rawhid] Received to write : _%s_ . %d bytes.", serial_buf, buf_length);
    #endif

    if( buf_length <= 64)
    {
      if ( buf_length == 64 )
      {
        #ifdef DEBUG_FILE
          sent_bytes += fwrite(serial_buf, sizeof(char), 64, x->x_debug_file_out);
          fflush(x->x_debug_file_out);
        #else
          sent_bytes += rawhid_send(0, serial_buf, 64, 0);
        #endif
      }
      else
      {
        char padded_buf[64];
        memset(padded_buf, 0, 64);

        #ifdef DEBUG_COM
	        for (int i = 0; i < 64 ; i++)
	        {
	            post ("[rawhid] padded_buf_zeroed [%d] : %d", i, padded_buf[i]);
	        }
	        
	        for (int i = 0; i < buf_length ; i++)
	        {
	            post ("[rawhid] serial_buf [%d] : %d", i, serial_buf[i]);
	        }
	    #endif

        memcpy(padded_buf, serial_buf, buf_length);

        #ifdef DEBUG_COM
        	post ("[rawhid] Making padding. copying the first %d bytes. %s", buf_length, padded_buf);
       	#endif

       	//memset(padded_buf + buf_length, 0, 64 - buf_length); // Adding final padding bytes
        #ifdef DEBUG_FILE
          sent_bytes += fwrite(padded_buf, sizeof(char), 64, x->x_debug_file_out);
          fflush(x->x_debug_file_out);
        #else

          #ifdef DEBUG_COM
          	for (int i = 0; i < 64 ; i++)
          	{
            	post ("[rawhid] padded_buf [%d] : %d", i, padded_buf[i]);
          	}
          #endif

          sent_bytes += rawhid_send(0, padded_buf, 64, 0);
        #endif
      }
    }
    else
    {
      int outpacket_bytes = buf_length % 64 ;
      int fullpacket_bytes = serial_buf - outpacket_bytes ;

      // Sending full packets. 
      #ifdef DEBUG_FILE
        sent_bytes += fwrite(serial_buf, sizeof(char), serial_buf - fullpacket_bytes, x->x_debug_file_out);
        fflush(x->x_debug_file_out);
      #else
        sent_bytes += rawhid_send(0, serial_buf, serial_buf - fullpacket_bytes, 0);
      #endif

      // Sending remaining bytes within a new packet
      unsigned char padded_buf[64];
      strncpy(padded_buf, serial_buf+fullpacket_bytes, outpacket_bytes);
      memset(padded_buf + outpacket_bytes, 0, 64 - buf_length); // Adding final padding bytes

      #ifdef DEBUG_FILE
        sent_bytes += fwrite(padded_buf, sizeof(char), 64, x->x_debug_file_out);
        fflush(x->x_debug_file_out);
      #else
        sent_bytes += rawhid_send(0, padded_buf, 64, 0);
      #endif
    }

    //int sent = rawhid_send(0, serial_buf, buf_length, 0);
    
    if (sent_bytes < buf_length && (sent_bytes % 64) != 0 )
    {
      pd_error(x,"[rawhid] Write failed for %d bytes, error is %d",sent_bytes,errno);
    }
    else
    {
      #ifdef DEBUG_COM
      	post("[rawhid] Written %d bytes.", sent_bytes);
      #endif
    }

    // if (i != buf_length) 
    // {
    // 	post ("[rawhid] buffer is full");
    // }
    return sent_bytes;
}

static void rawhid_float(t_rawhid *x, t_float f)
{
    unsigned char serial_byte = ((int) f) & 0xFF; /* brutal conv */

    if (write_serial(x,serial_byte) != 1)
    {
        post("Write error, maybe TX-OVERRUNS on serial line");
    }
}

static void rawhid_list(t_rawhid *x, t_symbol *s, int argc, t_atom *argv)
{
    unsigned char   temp_array[RAWHID_BUF_SIZE];/* arbitrary maximum list length */
    int             i, count;
    int             result;

    count = argc;
    if (argc > RAWHID_BUF_SIZE)
    {
        post ("[rawhid] truncated list of %d elements to %d", argc, count);
        count = RAWHID_BUF_SIZE;
    }
    for(i = 0; i < count; i++)
        temp_array[i] = ((unsigned char)atom_getint(argv+i))&0xFF; /* brutal conv */
    result = write_serials(x, temp_array, count);
}


static void rawhid_close_device(t_rawhid *x)
{
	if ( x->x_isOpen )
	{
    #ifdef DEBUG_FILE
      fclose(x->x_debug_file_out); 
      fclose(x->x_debug_file_in); 
    #else
	  rawhid_close(0);
    #endif

      x->x_isOpen = 0;
      clock_unset(x->x_clock);
	  post("[rawhid] Device 0x%04x 0x%04x closed", x->x_brandId , x->x_productId );
	}
  	else
  	{
      post("[rawhid] No open device to close found", x->x_brandId , x->x_productId );
  	}
}

 
static void rawhid_open_device(t_rawhid* x, t_symbol *brandId, t_symbol *productId) 
{

    int bId = (int) strtol(brandId->s_name,NULL,16);
    int pId = (int) strtol(productId->s_name,NULL,16);

  #ifdef DEBUG_FILE

    #ifdef DEBUG_FILE
      x->x_debug_file_out = fopen("/tmp/rawhid_out", "wb");
      x->x_debug_file_in = fopen("/tmp/rawhid_in", "rb"); 
    #endif

    x->x_brandId = bId; 
    x->x_productId = pId;
    x->x_isOpen = 1;
    clock_delay(x->x_clock, x->x_deltime);
  #else


    if ( ( bId > 0 ) && ( brandId->s_name[0] == '0' ) && ( brandId->s_name[1] == 'x') && ( pId > 0 ) && ( productId->s_name[0] == '0' ) && ( productId->s_name[1] == 'x') ) 
    {
      if ( rawhid_open(1, bId, pId , 0xFFAB, 0x0200 ) > 0 ) 
      {
          post("[rawhid] Device %s %s open", brandId->s_name, productId->s_name);
          x->x_brandId = bId; 
          x->x_productId = pId;
          x->x_isOpen = 1;
          clock_delay(x->x_clock, x->x_deltime);
      }
      else
      {
          post("[rawhid] Impossible to open device %s %s", brandId->s_name, productId->s_name);
      }
    }
    else
    {
      post("[rawhid] Invalid input for open operation. (e.g. open 0x002a 0x160c)", brandId->s_name, productId->s_name);
    }
  #endif

}

static void rawhid_poll(t_rawhid* x, t_float poll) 
{
    post("[rawhid] Polling %f", poll);
    x->x_deltime = poll;
}

static void rawhid_packets(t_rawhid* x, t_float pockets) 
{
    x->x_packets_to_recv = (int) pockets ;
    post("[rawhid] Packets %d", x->x_packets_to_recv);
}


/* the 'constructor' method which defines the t_rawhid struct for this 
   instance and returns it to the caller which is the Pd core */
static void *rawhid_new(void)
{
    t_rawhid *x = (t_rawhid *)pd_new(rawhid_class);

    x->x_inbuf = getbytes(RAWHID_BUF_SIZE);
    if (NULL == x->x_inbuf)
    {
        pd_error(x, "[rawhid] unable to allocate input buffer");
        return 0;
    }
    x->x_inbuf_len = RAWHID_BUF_SIZE;
    x->x_outbuf = getbytes(RAWHID_BUF_SIZE);
    if (NULL == x->x_outbuf)
    {
        pd_error(x, "[rawhid] unable to allocate output buffer");
        return 0;
    }
    x->x_outbuf_len = RAWHID_BUF_SIZE;
    x->x_outbuf_wr_index = 0;

    x->x_data_outlet = outlet_new(&x->x_obj, &s_float);

    x->x_packets_to_recv = 1 ; // default = 1



    /* Since 10ms is also the default poll time for most HID devices,
     * and it seems that for most uses of [comport] (i.e. arduinos and
     * other serial ports 115200 baud or less) that 10ms polling is
     * going to give the data as fast as 1ms polling with a lot less
     * CPU time wasted. */
    x->x_deltime = 10;
    x->x_clock = clock_new(x, (t_method)rawhid_tick);

    //clock_delay(x->x_clock, x->x_deltime);

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



void rawhid_setup(void) {
  /* this registers the 'rawhid' class. The 'rawhid_new' method will be executed at each instantiation. */
  rawhid_class = class_new(gensym("rawhid"), (t_newmethod)rawhid_new, 0, sizeof(t_rawhid), CLASS_DEFAULT, 0);

  class_addfloat(rawhid_class, (t_method)rawhid_float);
  class_addlist(rawhid_class, (t_method)rawhid_list);

  class_addmethod(rawhid_class, (t_method)rawhid_open_device, gensym("open"), A_DEFSYM, A_DEFSYM, 0);
  class_addmethod(rawhid_class, (t_method)rawhid_poll, gensym("poll"), A_FLOAT, 0);
  class_addmethod(rawhid_class, (t_method)rawhid_packets, gensym("packets"), A_FLOAT, 0);
  class_addmethod(rawhid_class, (t_method)rawhid_close_device, gensym("close"), 0);
}




















































