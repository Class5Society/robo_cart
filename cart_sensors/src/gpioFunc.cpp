#include "cart_sensors/cart_sensors.h"

/****************************************************************
 * gpio_set_edge
 ****************************************************************/
int gpio_set_edge(std::string encoderPort, std::string edge)
{
	int fd;
	std::string buf = encoderPort + "/edge";

	fd = open((char *) buf.c_str(), O_WRONLY);
	if (fd < 0) {
		return -1;
	}
 
	write(fd, (char *) edge.c_str(), edge.length() + 1); 
	close(fd);
	return 0;
}
/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(std::string encoderPort, std::string dirStr)
{
	int fd;
	std::string buf = encoderPort + "/direction";
 
	fd = open((char *) buf.c_str(), O_WRONLY);
	if (fd < 0) {
		return -1;
	}
 
	write(fd, (char *) dirStr.c_str(), dirStr.length() + 1); 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(std::string encoderPort)
{
	int fd, len;

	std::string buf = encoderPort + "/value";
 
	fd = open((char *) buf.c_str(), O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		return -1;
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
	return close(fd);
}



