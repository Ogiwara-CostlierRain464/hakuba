#ifndef SRC_CALCULATOR_H
#define SRC_CALCULATOR_H

#include "lexer.h"
#include <string>

int calculate(std::string const &str){
    Lexer lexer(str);
    lexer.tokenize();
}

#endif