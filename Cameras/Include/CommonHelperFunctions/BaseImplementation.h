// Basic implentation of the basic interface
// Also contain simple smart pointer definitions

#pragma once

#include "BaseInterface.h"
#include "CrtDbg.h"

// Class IBaseImpl : an Implementation of IBase 
template<class CT>
class IBaseImpl : public CT
{
	// Data:
private:
	unsigned long int m_refCount;
	// Construction / Destruction:
public:
	IBaseImpl() : CT(), m_refCount(0) { }     // default Constructor
	IBaseImpl(const IBaseImpl<CT>& rCopy) : CT(rCopy), m_refCount(0) { } // copy Constructor
protected:
	virtual ~IBaseImpl() { }
	// Assignment:
public:
	const IBaseImpl<CT>& operator=(const IBaseImpl<CT>& rCopy)
	{
		CT::operator=(rCopy);
		return *this;
	}

	// Implementation:
public:
	virtual ULONG STDMETHODCALLTYPE AddRef()
	{
		return InterlockedIncrement(reinterpret_cast<LONG*>(&m_refCount));
	}

	virtual ULONG STDMETHODCALLTYPE Release()
	{
		ULONG refCount = InterlockedDecrement(reinterpret_cast<LONG*>(&m_refCount));
		if (refCount == 0)
		{
			delete this;
			return 0;
		}

		return refCount;
	}

	virtual ULONG RefCount() const
	{
		return m_refCount;
	}

	virtual HRESULT  STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject)
	{
		if (ppvObject == NULL)
			return E_POINTER;
		if (riid == IID_IUnknown)
		{
			AddRef();
			*ppvObject = this;
			return S_OK;
		}
		*ppvObject = NULL;
		return E_NOINTERFACE;
	}
};

typedef IBaseImpl<IBase> CBase;

template <class CT>
class CNoAddRefReleaseCast : public CT
{
private:
	virtual ULONG __stdcall AddRef() = 0;
	virtual ULONG __stdcall Release() = 0;
};

//  Class CSmartPtr : a smart pointer class.
template<class CT>
class CSmartPtr
{
	// Data:
private:
	CT* m_ptr;
	//Implementation:
public:
	// Construction  / Destruction
	CSmartPtr() : m_ptr(NULL) { }
	CSmartPtr(const CSmartPtr<CT>& rSP) : m_ptr(rSP.m_ptr)
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}

	template<class TDerived>
	CSmartPtr(const CSmartPtr<TDerived>& rSP) : m_ptr(rSP.GetPtr())
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}

	// move ctor - avoid unnecessary refcount management
	CSmartPtr(CSmartPtr<CT>&& rSP) : m_ptr(rSP.m_ptr)
	{
		rSP.m_ptr = nullptr;
	}

	// move ctor - avoid unnecessary refcount management
	template<class TDerived>
	CSmartPtr(CSmartPtr<TDerived>&& rSP) : m_ptr(rSP.GetPtr())
	{
		rSP.Detach();
	}

	CSmartPtr(CT* iPtr) : m_ptr(iPtr)
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}
	~CSmartPtr()
	{
		if (m_ptr != NULL)
			m_ptr->Release();
	}
	// assignment:
	const CSmartPtr<CT>& operator=(const CSmartPtr<CT>& rSP)
	{
		if (rSP.m_ptr)
			rSP.m_ptr->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = rSP.m_ptr;
		return *this;
	}

	// assignment derived:
	template <class TDerived>
	const CSmartPtr<CT>& operator=(const CSmartPtr<TDerived>& rSP)
	{
		if (rSP.GetPtr())
			rSP.GetPtr()->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = rSP.GetPtr();
		return *this;
	}

	// move operator= : avoid unnecessary ref count management
	const CSmartPtr<CT>& operator=(CSmartPtr<CT>&& rSP)
	{
		if (m_ptr != nullptr)
			m_ptr->Release();

		m_ptr = rSP.m_ptr;
		rSP.m_ptr = nullptr;

		return *this;
	}

	const CSmartPtr<CT>& operator=(CT* iPtr)
	{
		if (iPtr)
			iPtr->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = iPtr;
		return *this;
	}

	// Cast operator:
	operator CT* () const
	{
		return m_ptr;
	}

	// Operations:

	void Release()
	{
		if (m_ptr != NULL)
			m_ptr->Release();
		m_ptr = NULL;
	}

	void** GetPtrPtr()
	{
		return (void**)&m_ptr;
	}


	CT* GetPtr() const
	{
		return m_ptr;
	}

	__forceinline CNoAddRefReleaseCast<CT>* operator->() const
	{
		return (CNoAddRefReleaseCast<CT> *)m_ptr;
	}

	CT& operator*() const
	{
		return *m_ptr;
	}

	void Attach(CT* pT)
	{
		if (m_ptr != NULL)
			m_ptr->Release();
		m_ptr = pT;
	}

	CT* Detach()
	{
		CT* pProd = m_ptr;
		m_ptr = NULL;
		return pProd;
	}

	const CT* Detach() const
	{
		CT* pProd = m_ptr;
		m_ptr = NULL;
		return pProd;
	}

	BOOL operator!() const
	{
		return (m_ptr == NULL);
	}

	HRESULT CopyTo(CT** ppT) const
	{
		if (ppT == NULL)
			return E_POINTER;
		*ppT = m_ptr;
		if (m_ptr != NULL)
			m_ptr->AddRef();
		return S_OK;
	}

	BOOL operator<(const CSmartPtr<CT>& rSP2) const;
	BOOL operator<=(const CSmartPtr<CT>& rSP2)const;
	BOOL operator>(const CSmartPtr<CT>& rSP2)const;
	BOOL operator>=(const CSmartPtr<CT>& rSP2)const;
};
// External Functions:

template<class CT>
BOOL CSmartPtr<CT>::operator<(const CSmartPtr<CT>& rSP2)const
{
	return (m_ptr < rSP2.m_ptr);
}

template<class CT>
BOOL CSmartPtr<CT>::operator<=(const CSmartPtr<CT>& rSP2)const
{
	return (m_ptr <= rSP2.m_ptr);
}

template<class CT>
BOOL CSmartPtr<CT>::operator>(const CSmartPtr<CT>& rSP2)const
{
	return (m_ptr > rSP2.m_ptr);
}

template<class CT>
BOOL CSmartPtr<CT>::operator>=(const CSmartPtr<CT>& rSP2)const
{
	return (m_ptr >= rSP2.m_ptr);
}

#define CastPtr(S__P, New__Type)  ((New__Type *)((S__P).GetPtr()))
#define CastRef(S__P, New__Type)  (*((New__Type *)((S__P).GetPtr())))

///////////////////////////////////////////// CONST SMART PTR //////////////////////////////////////////

//  Class CSmartPtr : a smart pointer class.
template<class CT>
class CConstSmartPtr
{
	// Data:
private:
	CT* m_ptr;
	//Implementation:
public:
	// Construction  / Destruction
	CConstSmartPtr() : m_ptr(NULL)
	{
	}
	CConstSmartPtr(const CConstSmartPtr<CT>& rSP) : m_ptr(rSP.m_ptr)
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}
	CConstSmartPtr(const CSmartPtr<CT>& rSP) : m_ptr(rSP.GetPtr())
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}

	template<class TDerived>
	CConstSmartPtr(const CConstSmartPtr<TDerived>& rSP) : m_ptr(rSP.GetPtr())
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}

	template<class TDerived>
	CConstSmartPtr(const CSmartPtr<TDerived>& rSP) : m_ptr(rSP.GetPtr())
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}

	// move ctor - avoid unnecessary refcount management
	CConstSmartPtr(CConstSmartPtr<CT>&& rSP) : m_ptr(rSP.m_ptr)
	{
		rSP.Detach();
	}

	// move ctor - avoid unnecessary refcount management
	template<class TDerived>
	CConstSmartPtr(CConstSmartPtr<TDerived>&& rSP) : m_ptr(rSP.GetPtr())
	{
		rSP.Detach();
	}

	CConstSmartPtr(CT* iPtr) : m_ptr(iPtr)
	{
		if (m_ptr != NULL)
			m_ptr->AddRef();
	}
	~CConstSmartPtr()
	{
		if (m_ptr != NULL)
			m_ptr->Release();
	}
	// assignment:
	const CConstSmartPtr<CT>& operator=(const CConstSmartPtr<CT>& rSP)
	{
		if (rSP.m_ptr)
			rSP.m_ptr->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = rSP.m_ptr;
		return *this;
	}

	// assignment derived:
	template <class TDerived>
	const CConstSmartPtr<CT>& operator=(const CConstSmartPtr<TDerived>& rSP)
	{
		if (rSP.GetPtr())
			rSP.GetPtr()->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = rSP.GetPtr();
		return *this;
	}

	// assignment derived:
	template <class TDerived>
	const CConstSmartPtr<CT>& operator=(const CSmartPtr<TDerived>& rSP)
	{
		if (rSP.GetPtr())
			rSP.GetPtr()->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = rSP.GetPtr();
		return *this;
	}

	// move operator= : avoid unnecessary ref count management
	const CConstSmartPtr<CT>& operator=(CConstSmartPtr<CT>&& rSP)
	{
		if (m_ptr != nullptr)
			m_ptr->Release();

		m_ptr = rSP.m_ptr;
		rSP.m_ptr = nullptr;

		return *this;
	}

	// move operator= : avoid unnecessary ref count management
	template <class TDerived>
	const CConstSmartPtr<CT>& operator=(CConstSmartPtr<TDerived>&& rSP)
	{
		if (m_ptr != nullptr)
			m_ptr->Release();

		m_ptr = rSP.GetPtr();
		rSP.Detach();

		return *this;
	}

	const CConstSmartPtr<CT>& operator=(CT* iPtr)
	{
		if (iPtr)
			iPtr->AddRef();

		if (m_ptr != NULL)
			m_ptr->Release();

		m_ptr = iPtr;
		return *this;
	}

	// Cast operator:
	operator const CT* () const
	{
		return m_ptr;
	}

	// Operations:

	void Release()
	{
		if (m_ptr != NULL)
			m_ptr->Release();
		m_ptr = NULL;
	}

	const void** GetPtrPtr() const
	{
		return (void**)&m_ptr;
	}

	const CT* GetPtr() const
	{
		return m_ptr;
	}

	__forceinline const CNoAddRefReleaseCast<CT>* operator->() const
	{
		return (const CNoAddRefReleaseCast<CT> *)m_ptr;
	}

	const CT& operator*() const
	{
		return *m_ptr;
	}

	void Attach(CT* pT)
	{
		if (m_ptr != NULL)
			m_ptr->Release();
		m_ptr = pT;
	}

	const CT* Detach()
	{
		CT* pProd = m_ptr;
		m_ptr = NULL;
		return pProd;
	}

	BOOL operator!() const
	{
		return (m_ptr == NULL);
	}

	HRESULT CopyTo(const CT** ppT) const
	{
		if (ppT == NULL)
			return E_POINTER;
		*ppT = m_ptr;
		if (m_ptr != NULL)
			m_ptr->AddRef();
		return S_OK;
	}

	BOOL operator<(const CConstSmartPtr<CT>& rSP2) const;
	BOOL operator<=(const CConstSmartPtr<CT>& rSP2)const;
	BOOL operator>(const CConstSmartPtr<CT>& rSP2)const;
	BOOL operator>=(const CConstSmartPtr<CT>& rSP2)const;
};
// External Functions:

template<class CT>
BOOL CConstSmartPtr<CT>::operator<(const CConstSmartPtr<CT>& rSP2)const
{
	return (m_ptr < rSP2.m_ptr);
}

template<class CT>
BOOL CConstSmartPtr<CT>::operator<=(const CConstSmartPtr<CT>& rSP2)const
{
	return (m_ptr <= rSP2.m_ptr);
}

template<class CT>
BOOL CConstSmartPtr<CT>::operator>(const CConstSmartPtr<CT>& rSP2)const
{
	return (m_ptr > rSP2.m_ptr);
}

template<class CT>
BOOL CConstSmartPtr<CT>::operator>=(const CConstSmartPtr<CT>& rSP2)const
{
	return (m_ptr >= rSP2.m_ptr);
}


