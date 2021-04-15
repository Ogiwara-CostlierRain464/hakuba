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
    Type type;
    std::string value;
    int precedence;
    Token(Type t, std::string v): type(t), value(v), precedence(0){
        if(t == Type::Div or t == Type::Mul) precedence = 2;
        if(t == Type::Add or t == Type::Sub) precedence = 1;
    }

    bool isOperator(){
        return (type != Type::Number
            and type != Type::RightPar
            and type != Type::LeftPar
            and type != Type::Empty
        );
    }
};

#endif