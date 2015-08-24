//////////////////////////////////////////////////////////////////////////////////////////
//	LOG.cpp
//	Functions for an error log
//	Downloaded from: www.paulsprojects.net
//	Created:	20th July 2002
//	Modified:	8th November 2002	-	Added "Output Misc"
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	
#include <stdarg.h>
#include <string.h>
#include "LOG.h"

//Initiate log
bool LOG::Init(const char * newFilename)
{
    //Save filename
    if(filename)
        delete [] filename;

    filename=new char[strlen(newFilename)];
    if(!filename)
        return false;

    strcpy(filename, newFilename);

    //Open file, overwriting any previous contents
    logFile=fopen(filename, "wb");
    if(!logFile)
        return false;

    //Close the file
    fclose(logFile);
    logFile=NULL;

    return true;
}

//Output a newline
void LOG::OutputNewLine()
{
    //Open file to append
    logFile=fopen(filename, "a+");
    if(!logFile)
        return;

    //Write the newline
    putc('\n', logFile);

    //close the file
    fclose(logFile);
    logFile=NULL;
}

//Output Success
void LOG::OutputSuccess(const char * text, ...)
{
    //Compile string to output
    va_list argList;
    va_start(argList, text);

    //Open file to append
    logFile=fopen(filename, "a+");
    if(!logFile)
        return;

    //Write the text
    fprintf(logFile, "<-> ");
    vfprintf(logFile, text, argList);
    putc('\n', logFile);

    //close the file
    fclose(logFile);
    logFile=NULL;
    va_end(argList);
}

//Output An Error
void LOG::OutputError(const char * text, ...)
{
    //Compile string to output
    va_list argList;
    va_start(argList, text);

    //Open file to append
    logFile=fopen(filename, "a+");
    if(!logFile)
        return;

    //Write the text
    fprintf(logFile, "<!> ");
    vfprintf(logFile, text, argList);
    putc('\n', logFile);

    //close the file
    fclose(logFile);
    logFile=NULL;
    va_end(argList);
}

//Output Miscellaneous
void LOG::OutputMisc(const char * text, ...)
{
    //Compile string to output
    va_list argList;
    va_start(argList, text);

    //Open file to append
    logFile=fopen(filename, "a+");
    if(!logFile)
        return;

    //Write the text
    fprintf(logFile, "<#> ");
    vfprintf(logFile, text, argList);
    putc('\n', logFile);

    //close the file
    fclose(logFile);
    logFile=NULL;
    va_end(argList);
}