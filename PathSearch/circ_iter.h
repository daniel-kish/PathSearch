#pragma once

#include <iostream>
#include <vector>

template <class T>
class circular_iterator
{
private:
	std::vector<T>& l;
	using iter = typename std::vector<T>::iterator;
	iter i;
public:
	circular_iterator(std::vector<T>& lref) : l{lref}, i{l.begin()}
	{    }
	circular_iterator& operator++()
	{
		if (i >= l.end() - 1)
		{
			i = l.begin();
			return *this;
		}
		++i;
		return *this;
	}
	circular_iterator operator+(int a)
	{
		circular_iterator copy = *this;
		while (a--)
			++copy;
		return copy;
	}
	T& operator*()
	{
		return *i;
	}
	iter& get()
	{
		return i;
	}
};