#ifndef  SEND_SYSTEM_MESSAGE_HEAD
#define  SEND_SYSTEM_MESSAGE_HEAD


#include <fcntl.h>
#include <sys/stat.h>        /* For mode constants */
#include <mqueue.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#define MQNAME "/mqkhcamera"


int SendSystemMsg(char *messgestr, int str_length)
{

    mqd_t mqd;
    mq_attr  attr;
    int ret;

    

    mqd = mq_open(MQNAME, O_RDWR|O_CREAT, 0600, NULL);
    if (mqd == -1) {
        perror("mq_open()");
        exit(1);
    }
   
    if (mq_getattr(mqd, &attr) < 0) { //获取消息队列属性
        fprintf(stderr, "mq_getattr: %s\n", strerror(errno));
        return -1;
    }
 
    printf("flags: %ld, maxmsg: %ld, msgsize: %ld, curmsgs: %ld\n",
            attr.mq_flags, attr.mq_maxmsg, attr.mq_msgsize, attr.mq_curmsgs);

    if(attr.mq_curmsgs==attr.mq_maxmsg)
     {
      mq_close(mqd);
      return 0;
     }
    ret = mq_send(mqd, messgestr, str_length, 1 );//atoi(argv[2]));
	printf("msg: (%s), prior:%i", messgestr, 1);
    if (ret == -1) {
        perror("mq_send()");
        exit(1);
    }
	
    mq_close(mqd);
	
	return 1;
    //exit(0);
}




#endif
