#include <iostream>
#include "generic_database.h"

using namespace std;

int main(){
  GenericDB db("/tmp/ababa");
  auto table = db.loadTable(0);

  for(auto iter = table.begin(); iter != table.end(); ++iter){
    auto pair = *iter;
    cout << pair.first.toSec() << endl;
  }
}