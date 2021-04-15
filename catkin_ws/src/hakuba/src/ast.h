#ifndef SRC_AST_H
#define SRC_AST_H

#include <memory>
#include <iostream>
#include <sstream>
#include "token.h"

class AST{
public:
    AST(): token(Token(Type::Empty, "")), value(0), left(nullptr), right(nullptr){}
    AST(Token t): token(t), value(0), left(nullptr), right(nullptr){}
    void print(){ printAST(std::shared_ptr<AST>(this));  std::cout << std::endl; }
    void build(std::vector<Token> postfix){
        std::shared_ptr<AST> n = buildAST(postfix);
        this->token = n->token;
        this->right = n->right;
        this->left = n->left;
    }

    int evaluate(){
        if(token.type == Type::Number) return toNumber(token.value);
        if(token.type == Type::Empty) return 0;
        switch (token.type) {
            case Type::Add: return left->evaluate() + right->evaluate();
            case Type::Sub: return left->evaluate() - right->evaluate();
            case Type::Mul: return left->evaluate() * right->evaluate();
            case Type::Div: return left->evaluate() / right->evaluate();
            default: assert(false && "A node with children can only be an operator");
        }
        return 0;
    }

private:
    Token token;
    int value;
    std::shared_ptr<AST> left;
    std::shared_ptr<AST> right;

    bool isFull(){
        return left != nullptr and right != nullptr;
    }

    void populate(std::shared_ptr<AST> n, std::shared_ptr<AST> m){
        if(n->right == nullptr){
            n->right = m;
        }else if(n->left == nullptr){
            n->left = m;
        }else{
            exit(-1);
        }
    }

    void printAST(std::shared_ptr<AST> n){
        if(n->left != nullptr and n->right != nullptr) std::cout << "(";
        if(n->left != nullptr) printAST(n->left);
        std::cout << n->token.value;
        if(n->right != nullptr) printAST(n->right);
        if(n->left != nullptr and n->right != nullptr) std::cout << ")";
    }

    int toNumber(std::string str){
        std::stringstream ss(str);
        int rv; ss >> rv;
        if(ss.fail()){
            exit(-1);
        }
        return rv;
    }

    std::shared_ptr<AST> buildAST(std::vector<Token> &postfix){
        std::shared_ptr<AST> root(new AST(postfix.back()));
        postfix.pop_back();
        while (!postfix.empty()){
            if(postfix.back().isOperator()){
                populate(root, buildAST(postfix));
            }else{
                populate(root, std::make_shared<AST>(postfix.back()));
                postfix.pop_back();
            }
            if(root->isFull()){
                return root;
            }
        }
        return root;
    }
};

#endif //SRC_AST_H
