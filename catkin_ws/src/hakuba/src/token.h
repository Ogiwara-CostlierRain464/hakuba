#ifndef SRC_TOKEN_H
#define SRC_TOKEN_H

#include <string>

enum class Type: char{
    Number = 'n',
    Add = '+',
    Sub = '-',
    Mul = '*',
    Div = '/',
    LeftPar = '(',
    RightPar = ')',
    Empty = 'e'
};

struct Token{
    Type
};

#endif