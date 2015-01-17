//////////////////////////////////////////////////////////////////////////////////////////
//	LOG.h
//	Singleton class declaration for a log file
//	Downloaded from: www.paulsprojects.net
//	Created:	20th July 2002
//	Modified:	8th November 2002	-	Added "Output Misc"
//				18th November 2002	-	Made singleton to provide global access point
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef LOG_H
#define LOG_H

#include <stdio.h>

class LOG
{
protected:
	//protected constructor and copy constructor to prevent making copies
	LOG()
	{	Init("Error Log.txt");}
	LOG(const LOG &)
	{}
	LOG & operator= (const LOG &)
	{}
    LOG(const char * newFilename)
    {  Init(newFilename);}

public:
	//public function to access the instance of the log class
	static LOG * Instance()
	{
		//Instance of log class
		static LOG instance;
		return &instance;
	}

    static LOG * Instance(const char * newFilename)
    {
        static LOG instance(newFilename);
        return &instance;
    }
	
	bool Init(const char * newFilename);

	//Output a new line
	void OutputNewLine();

	//Output messages
	void OutputSuccess(const char * text, ...);
	void OutputError(const char * text, ...);
	void OutputMisc(const char * text, ...);

protected:
	char * filename;
	FILE * logFile;		//pointer to log file, only open when writing
};

#endif
