#ifndef DSU_H
#define DSU_H

using namespace std;

#include <map>

template<typename T>
class dsu
{
    public:
        dsu();
        virtual ~dsu();

        void union_sets(T a,T b);
        T operator[](T p);//parent


    private:
        void make_set(T p);
        map<T,int> rank_;
        map<T,T> parent;

};
template<typename T>
dsu<T>::dsu()
{

}

template<typename T>
dsu<T>::~dsu()
{

}

template<typename T>
inline void dsu<T>::make_set(T p)
{
    if (rank_.count(p))
        return;
    rank_[p]=0;
    parent[p]=p;
}

template<typename T>
inline T dsu<T>::operator[](T p)
{
    make_set(p);
    if (parent[p]==p)
        return p;
    return parent[p]=(*this)[parent[p]];
}

template<typename T>
inline void dsu<T>::union_sets(T a,T b){
	a = (*this)[a];
	b = (*this)[b];
	if (a != b) {
		if (rank_[a] < rank_[b])
			swap (a, b);
		parent[b] = a;
		if (rank_[a] == rank_[b])
			++rank_[a];
	}
}

#endif // DSU_H
