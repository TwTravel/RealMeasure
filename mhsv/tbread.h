#ifndef Table_Read_H
#define Table_Read_H
#include <string>
#include <vector>
using namespace std;

struct tb_item
{
int cnt;
vector<string> strvec;
};

class TableRead
{
public:
int recordcnt;
vector<tb_item>itemvec;
vector<string> vecstr;
void readtb_fromfile(char*filename);
void rdstrlist_fromfile(char*filename);
string seperator;
};

void TableRead::rdstrlist_fromfile(char*filename)
{
FILE*file;
file=fopen(filename,"r");
char line[500];
tb_item item;
string rcdstr,linestr;
 while(!feof(file))
	{ //fscanf(file,"%s",msg);
	 fgets(line,500,file);
	 linestr=line;
	 if(linestr.length()<2)continue;   
	 linestr=linestr.substr(0,linestr.length()-1);
     vecstr.push_back(linestr); 
	}
fclose(file);
}

void TableRead::readtb_fromfile(char*filename)
{
FILE*file;
file=fopen(filename,"r");
char line[500];
tb_item item;
string rcdstr,linestr;
 while(!feof(file))
	{ //fscanf(file,"%s",msg);
	 fgets(line,500,file);
	 linestr=line;
	 if(linestr.length()<2)continue;
	 linestr=linestr.substr(0,linestr.length()-1);
     item.strvec.clear(); 
     
	 char* token = strtok((char*)linestr.c_str(), seperator.c_str()); 
     while( token != NULL ) 
    { rcdstr=token;
	  item.strvec.push_back(rcdstr); 
      token = strtok( NULL, seperator.c_str()); 
    } 

    itemvec.push_back(item); 
	}
fclose(file);
}

#endif