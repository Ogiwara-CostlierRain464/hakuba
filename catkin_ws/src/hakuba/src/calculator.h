#ifndef SRC_CALCULATOR_H
#define SRC_CALCULATOR_H

#include "lexer.h"
#include "parser.h"
#include "ast.h"
#include <string>

double calculate(std::string const &str){
    Lexer lexer(str);
    lexer.tokenize();
    Parser parser(lexer.getTokens());

//    for(const auto & t : lexer.getTokens()){
//        std::cout << t.value;
//    }
//    std::cout << std::endl;

    AST tree;
    tree.build(parser.getPostfix());
    tree.print();
    return tree.evaluate();
}

#endif