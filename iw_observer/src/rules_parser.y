%{

#include <iw_observer/rules_ast.hpp>
#include <ros/ros.h>

extern int yylex();

void yyerror(const char* err) 
{
    ROS_ERROR("Rules parser error: %s", err);
}

using namespace iw_observer;

%}

%union {
    iw_observer::Rules*    rules;
    iw_observer::Rule*     rule;
    iw_observer::Commands* cmds;
    iw_observer::Command*  cmd;
    iw_observer::Args*     args;
    iw_observer::Arg*      arg;
    iw_observer::Ident*    ident;
    std::string*           string;
    int                    token;

}

%token <string> TSTRING;
%token <ident>  TIDENTIFIER;
%token <token>  TINTEGER;
%token <token>  TARROW;
%token <token>  TCOLON;
%token <token>  TCOMMA;
%token <token>  TSEMICOLON;
%token <token>  TLB;
%token <token>  TRB;
%token <token>  TIDENT_ADD;
%token <token>  TIDENT_DEL;
%token <token>  TIDENT_ID;
%token <token>  TIDENT_UTIL;
%token <token>  TIDENT_INT;
%token <token>  TIDENT_PARAMS;

%type <rules> rules;
%type <rule>  rule;
%type <cmds>  cmds;
%type <cmd>   cmd;
%type <args>  args;
%type <arg>   arg;
%type <ident> ident;

%start rules

%% 

rules : rule       { $$ = new Rules(); $$->push_back($<rule>1); }
      | rules rule { $1->push_back($<rule>2); }
      ;

rule : filter TARROW cmd_block { $$ = new Rule(); }
     ;

filter : ident TCOLON idents

cmd_block : cmd
          | TLB cmds TRB

cmds : cmd      { $$ = new Commands(); $$->push_back($<cmd>1); } 
     | cmds cmd { $1->push_back($<cmd>2); }
     ;

cmd : TIDENT_ADD ident TSEMICOLON
        { $$ = new AddCommand(*$<ident>1, Args()); }
    | TIDENT_ADD ident args TSEMICOLON
        { $$ = new AddCommand(*$<ident>1, *$<args>2); }
    | TIDENT_DEL ident TSEMICOLON
        { $$ = new DelCommand(*$<ident>1); }


idents : ident
       | idents ident

ident : TIDENTIFIER
      ;

args : arg      { $$ = new Args(); $$->push_back($<arg>1); }
     | args arg { $1->push_back($<arg>2); }
     ;

arg : TIDENT_ID     TCOLON TIDENTIFIER { $$ = new IdArg(*$3); }
    | TIDENT_UTIL   TCOLON TINTEGER    { $$ = new UtilArg($3); }
    | TIDENT_INT    TCOLON TINTEGER    { $$ = new IntArg($3); }
    | TIDENT_PARAMS TCOLON TSTRING     { $$ = new ParamsArg(*$3); }
    ;

%%
