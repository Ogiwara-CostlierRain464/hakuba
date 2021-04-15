#ifndef SRC_CALCULATOR_H
#define SRC_CALCULATOR_H

#include "lexer.h"
#include "parser.h"
#include "ast.h"
#include <string>

int calculate(std::string const &str){
    Lexer lexer(str);
    lexer.tokenize();
    Parser parser(lexer.getTokens());
    AST tree;
    tree.build(parser.getPostfix());
    return tree.evaluate();
}

#endif