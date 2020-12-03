#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <read_from_serial.h>

float read_from_serial()
{
int fd; /* port file descriptor */
char c;
char port[20] = "/dev/ttyO5"; /* port to connect to */
speed_t baud = B115200; /* baud rate */

fd = open(port, O_RDWR); /* connect to port */

/* set the other settings (in this case, 9600 8N1) */
struct termios settings;
tcgetattr(fd, &settings);

cfsetospeed(&settings, baud); /* baud rate */
settings.c_cflag &= ~PARENB; /* no parity */
settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
settings.c_cflag &= ~CSIZE;
settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
settings.c_lflag = ICANON; /* canonical mode */
settings.c_oflag &= ~OPOST; /* raw output */

tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
tcflush(fd, TCOFLUSH);

/* — code to use the port here — */

int bytes_read = 1;
int i=-1;
char array[1500];

char *ptr;
while (bytes_read>0) {
	i=i+1;
	char c = 0;
	bytes_read = read(fd, &c, 1);
//	printf("%c",c);
	array[i]=c;
	if (c == '\n') {
		const char s[2] = " ";
		char* token = strtok(array,s);
		int j=-1;
		while( token != NULL ) {
			token = strtok(NULL,s);
			j=j+1;
			//printf(" %s\n", token);
			if (j==4){
				data.vel[0]=strtod(token, &ptr);
			}
			else if (j==6){
				data.vel[1]=strtod(token, &ptr);
			}
			else if (j==8){
				data.vel[2]=strtod(token, &ptr);
			}
			else if (j==16){
				data.rho[0]=strtod(token, &ptr);

			}
	}
//	printf("%c",array);
//	printf("%lf\n",data.vel[0]);	
//	printf("%lf\n",data.vel[1]);	
//	printf("%lf\n",data.vel[2]);	
	
	break;
	}
}
return 0;
close(fd); /* cleanup */

}
