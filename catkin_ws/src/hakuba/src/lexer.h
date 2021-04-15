#ifndef SRC_LEXER_H
#define SRC_LEXER_H

#include <sstream>
#include <string>
#include <vector>
#include "token.h"

class Lexer {
public:
    explicit Lexer(std::string const &str)
    : stream(str)
    {}

    void tokenize() {
        char c = stream.peek();
        while(stream.good()){
            if(isdigit(c) or c == '.'){
                tokens.emplace_back(Type::Number, getNumber());
            }else{
                stream.get(c);
                if(isspace(c)){
                    while(stream.good() and isspace(stream.peek())) stream.get();
                }else if(validOperator(c)){
                    tokens.push_back(Token(static_cast<Type>(c), {c}));
                }else{
                    assert(false && "Invalid character");
                }
            }
            c = stream.peek();
        }
    }

    std::vector<Token> getTokens(){
        return tokens;
    }

private:
    std::istringstream stream;
    std::vector<Token> tokens;

    std::string getNumber(){
        std::string rv;
        bool decimal = false;
        char c;
        while(stream.good()){
            c = stream.peek();
            if(c != '.' and !isdigit(c)) break;
            c = stream.get();
            if(c == '.'){
                if(decimal)
                    assert(false && "Multiple dots on number");
                else decimal = true;
            }
            rv += c;
        }
        return rv;
    }


    static bool validOperator(char c){
        std::string operators = "+-*/()";
        for(char i : operators) if(i==c) return true;
        return false;
    }
};

#endif