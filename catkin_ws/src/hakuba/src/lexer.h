#ifndef SRC_LEXER_H
#define SRC_LEXER_H

#include <sstream>
#include <string>
#include <vector>
#include "token.h"

class Lexer {
public:
    Lexer(std::string const &str) {

    }

    void tokenize() {

    }

    std::vector<Token> getTokens(){
        return tokens;
    }

private:
    std::istringstream stream;
    std::vector<Token> tokens;

    std::string getNumber(){

    }


    bool validOperator(){

    }
}

#endif