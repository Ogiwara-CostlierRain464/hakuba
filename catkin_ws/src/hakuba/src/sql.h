#ifndef SRC_SQL_H
#define SRC_SQL_H

#include <memory>
#include <iostream>
#include <sstream>
#include <stack>

namespace sql{
    enum class Type: char{
        Number = 'n',
        LessThan = '<',
        GreaterThan = '>',
        And = '&',
        Or = '|',
        LeftPar = '(',
        RightPar = ')',
        Empty = 'e'
    };

    struct Token{
        Type type;
        std::string value{};
        int precedence;
        Token(Type t, std::string v): type(t), value(v), precedence(0){
            if(t == Type::LessThan or t == Type::GreaterThan) precedence = 2;
            if(t == Type::And or t == Type::Or) precedence = 1;
        }

        bool isOperator() const{
            return (type != Type::Number
                    and type != Type::RightPar
                    and type != Type::LeftPar
                    and type != Type::Empty
            );
        }
    };

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
            std::string operators = "<>&|()";
            for(char i : operators) if(i==c) return true;
            return false;
        }
    };

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
                    if(i < tokens.size() - 1 && (tokens[i+1].type == Type::LessThan or tokens[i+1].type == Type::GreaterThan)){
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

    class AST{
    public:
        AST(): token(Token(Type::Empty, "")), value(0), left(nullptr), right(nullptr){}
        AST(Token t): token(t), value(0), left(nullptr), right(nullptr){}
        void print(){ printAST();  std::cout << std::endl; }
        void build(std::vector<Token> postfix){
            std::shared_ptr<AST> n = buildAST(postfix);
            this->token = n->token;
            this->right = n->right;
            this->left = n->left;
        }

        double evaluate(){
            if(token.type == Type::Number) return toNumber(token.value);
            if(token.type == Type::Empty) return 0;
            switch (token.type) {
                case Type::LessThan: return (left->evaluate() < right->evaluate() ? 1 : 0);
                case Type::GreaterThan: return (left->evaluate() > right->evaluate() ? 1 : 0);
                case Type::And: return (left->evaluate() and right->evaluate() ? 1 : 0);
                case Type::Or: return (left->evaluate() or right->evaluate() ? 1 : 0);
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

        void printAST(){
            if(left != nullptr and right != nullptr) std::cout << "(";
            if(left != nullptr) left->printAST();
            std::cout << token.value << " ";
            if(right != nullptr) right->printAST();
            if(left != nullptr and right != nullptr) std::cout << ")";
        }

        double toNumber(std::string str){
            std::stringstream ss(str);
            double rv; ss >> rv;
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


    void replaceAll(std::string& str, const std::string& from, const std::string& to) {
        if(from.empty())
            return;
        size_t start_pos = 0;
        while((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }


    bool exec(std::string str){

        replaceAll(str, "x", std::to_string(2.5));

        Lexer lexer(str);
        lexer.tokenize();
        Parser parser(lexer.getTokens());

        AST tree;
        tree.build(parser.getPostfix());
//        tree.print();
        return tree.evaluate() == 1;
    }
}

#endif //SRC_SQL_H
