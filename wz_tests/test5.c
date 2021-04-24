#include <czmq.h>
#include <msgpack.h>
#include <string.h>
#include <stdio.h>
#include <endian.h>
int main (void)
{
    int end_flag = 0;
    zsock_t *pull = zsock_new_pair ("");
    zsock_bind(pull,"tcp://0.0.0.0:*[3000-]");
    while(!end_flag) {
      zmsg_t * msg = zmsg_recv (pull);
      printf("frames=%d, bytes=%d\n",zmsg_size(msg),zmsg_content_size(msg));
      zframe_t * frame = zmsg_first(msg);
      //Process the frame
      uint64_t * ptr = (uint64_t *)zframe_data(frame);
      if(zframe_size(frame) % 8) {
         printf("Frame size not N*8\n");
         abort();
      }
      uint64_t * pend = ptr + zframe_size(frame)/8;
      do {
         //Check the type of the record
         void * rec = (void *) ptr;
         if (!memcmp(rec,"WZDAQ1-E",8)) {
             printf("End of set!\n");
             ptr ++;
         } else if (!memcmp(rec,"WZDAQ1-D",8)) {
             if(++ptr >= pend) {
                printf("Error: WZDAQ1-D at end of frame\n");
                abort();
             }                        
             int nwords = be64toh(* ptr++);
             if (ptr + nwords > pend) {
                printf("Error: WZDAQ1-D data beyond the end of frame\n");
                abort();
             }
             printf("Frame: %d words\n",nwords);
             ptr += nwords;
         } else if (!memcmp(rec,"WZDAQ1-Q",8)) {
             printf("End of run!\n");
             end_flag = 1;
             break;             
         } else {
           printf("Error: No data type!");
           abort();
         }        
      } while(ptr < pend);      
      zmsg_destroy(&msg);
    }
    zsock_destroy (&pull);
    return 0;
}

