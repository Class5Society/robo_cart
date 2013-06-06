#ifndef GPIO_FUNC_H
#define GPIO_FUNC_H

int gpio_set_edge(std::string encoderPort, std::string edge);
int gpio_set_dir(std::string encoderPort, std::string dirStr);
int gpio_fd_open(std::string encoderPort);
int gpio_fd_close(int fd);

#endif
