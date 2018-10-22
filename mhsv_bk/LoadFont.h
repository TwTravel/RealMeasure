#ifndef xylsfitting_HHH
#define xylsfitting_HHH
#include <windows.h>


HINSTANCE LoadFontCharImgDLL;

 
typedef int  (__cdecl FontCharImgExtractDLLPROC)
//(char* FontFileName,char*TxtChar,
// char* buffer,int &PicWidth,int&PicHeight );
(char* FontFileName,char*TxtChar,
int charsize, char* buffer,int &PicWidth,int&PicHeight);

FontCharImgExtractDLLPROC * FontCharImgExtract_f = NULL;
 

void init_LoadFont_Func()
{
	if(FontCharImgExtract_f==NULL)
	{
    LoadFontCharImgDLL   =  LoadLibrary("GetFontText.dll");
    FontCharImgExtract_f = (FontCharImgExtractDLLPROC*)::GetProcAddress(LoadFontCharImgDLL,"FontCharImgExtract");
	}
}



#endif