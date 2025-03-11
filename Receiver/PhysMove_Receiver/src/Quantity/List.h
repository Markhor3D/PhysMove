#pragma once
#include <Arduino.h>
#include "Types.h"
template <typename T>
class List
{
private:
	int _capacity = 0;
	uint16 _count = 0;
	T* InnerList = 0;
public:
	List()
	{
	}
	~List()
	{
		if (_capacity > 0)
		{
			if (InnerList != 0)
			{
				delete InnerList;
				InnerList = 0;
			}
			_capacity = 0;
			// avoid multiple destruction
		}
	}
	T Find(bool (*condition)(T obj))
	{
		for (int i = 0; i < Count(); i++)
		{
			if (condition(InnerList[i]))
				return InnerList[i];
		}
		return 0;
	}	
	uint16 Count()
	{
		return _count;
	}
	uint16 Capacity()
	{
		return _capacity;
		//return size();
	};
	T* ToArray()
	{
		return InnerList;
	}
	float ToFloat(int offset)
	{
		uint8_t bytes[4];
		memcpy(bytes, InnerList + offset, 4);
		return *(reinterpret_cast<float*>(bytes));
	}
	T& operator [] (unsigned int index)
	{
		return InnerList[index];
	}
	T Add(T item) 
	{
		if (Count() == _capacity)
		{
			// allocate new space
			T* newSpace = new T[_capacity + 8];
			// copy data to new space if any
			for (int i = 0; i < _count; i++)
				newSpace[i] = InnerList[i];

			if (_capacity > 0) // delete previous space if not empty
				delete InnerList;
			// swap the space
			InnerList = newSpace;
			// remember how far we have come.
			_capacity += 8;
		}
		InnerList[Count()] = item;
		_count++;
		return item;
	}
	void Clear() 
	{
		// don't waste the allocated space
		_count = 0;

		if (_capacity > 0)
		{
			if (InnerList != 0)
			{
				delete InnerList;
				InnerList = 0;
			}
			_capacity = 0;
			// avoid multiple destruction
		}
	}
	int Distinct()
	{
		int removed = 0;
		for (int i = Count() - 1; i >= 0; i--)
		{
			for (int j = 0; j < i; j++)
			{
				if (InnerList[j] == InnerList[i])
				{
					RemoveAt(i);
					removed++;
					break;
				}
			}
		}
		return removed;
	}
	void Insert(T item, int ind)
	{
		if (ind < 0 || ind > Count())
			return;
		Add(0);
		for (int i = Count() - 1; i > ind; i--)
			InnerList[i] = InnerList[i - 1];
		InnerList[ind] = item;
	}
	void Remove(T item)
	{
		RemoveAt(IndexOf(item));
	}
	void RemoveAt(int ind)
	{
		if (ind < 0 || ind >= Count())
			return;

		for (int i = ind; i < Count() - 1; i++)
			InnerList[i] = InnerList[i + 1];
		_count--;
	}
	int IndexOf(T item) 
	{
		for (int i = 0; i < Count(); i++)
			if (InnerList[i] == item)
				return i;
		return -1;
	}
	bool Contains(T item)
	{
		return IndexOf(item) >= 0;
	}
};