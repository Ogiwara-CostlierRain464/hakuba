
#ifndef SRC_PARSER_H
#define SRC_PARSER_H

#include <vector>
#include <stack>
#include "token.h"

class Parser{
public:
    Parser(std::vector<Token> const &v): tokens(v){}
    std::vector<Token> getPostfix(){
        if(!validSyntax()){
            assert(false && "Invalid syntax");
        }

        std::vector<Token> rv;
        std::stack<Token> ops;
        for(size_t i = 0; i < tokens.size(); ++i){
            if(tokens[i].type == Type::Empty) continue;
            if(tokens[i].type == Type::Number){
                rv.push_back(tokens[i]);
            }else if(tokens[i].type == Type::LeftPar){
                ops.push(tokens[i]);
                if(i < tokens.size() - 1 && (tokens[i+1].type == Type::Add or tokens[i+1].type == Type::Sub)){
                    rv.emplace_back(Type::Number, "0");
                }
            }else if(tokens[i].type == Type::RightPar){
                while(ops.top().type != Type::LeftPar){
                    rv.push_back(extract(ops));
                }
                ops.pop();
            }else{
                while(!ops.empty() and ops.top().precedence >= tokens[i].precedence){
                    rv.push_back(extract(ops));
                }
                ops.push(tokens[i]);
            }
        }
        while (!ops.empty()){
            rv.push_back(extract(ops));
        }
        return rv;
    }

private:
    std::vector<Token> tokens;

    bool validSyntax(){
        int openPar = 0;
        bool operation = false;
        if(!tokens.empty() and tokens[0].isOperator()){
            return false;
        }
        for(size_t i = 0; i < tokens.size(); ++i){
            if(tokens[i].type == Type::Empty) continue;
            if(tokens[i].isOperator()){
                if(operation) return false;
                else operation = true;
            }else if(tokens[i].type != Type::RightPar){
                operation = false;
            }

            if(i > 0 and tokens[i].type == Type::Number and tokens[i-1].type == Type::Number) return false;
            if(tokens[i].type == Type::LeftPar and i > 0 and tokens[i-1].type == Type::Number) return false;
            if(tokens[i].type == Type::LeftPar) openPar++;
            if(tokens[i].type == Type::RightPar) openPar--;
            if(openPar < 0) return false;
        }
        return (openPar == 0 and !operation);
    }

    Token extract(std::stack<Token> &s){
        Token rv = s.top();
        s.pop();
        return rv;
    }
};

#endif //SRC_PARSER_H
