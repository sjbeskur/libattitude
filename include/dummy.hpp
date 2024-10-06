
#ifndef DUMMY_H
#define DUMMY_H

#include <iostream>

void dummy_hello(const std::string& message) {
    std::cout << "Hello " << message << std::endl;
}

int add(int i, int j)
{
    return i + j;
}

int subtract(int i, int j)
{
    return i - j;
}

#endif // DUMMY_H
