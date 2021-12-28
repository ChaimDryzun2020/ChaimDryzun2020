// Base Interface parent 

#pragma once

#include <Windows.h>
#include <unknwn.h>


// Parent of all interfaces
// Dependencies: None
class DECLSPEC_NOVTABLE IBase : public IUnknown
{
	// Operations:
public:

	// Number of instances
	virtual ULONG RefCount() const = 0;
};
