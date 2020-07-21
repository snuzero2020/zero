#pragma	once
#include "matrix/cmatrix"
#include <assert.h>

typedef std::valarray<double>		dVector;
typedef techsoft::matrix<double>	dMatrix;

typedef techsoft::mslice			mSlice;
typedef std::slice					vSlice;

inline void Null (dVector &v)
{
	for (unsigned int i = 0; i<v.size(); ++i) v[i] = 0.;
}

inline double Dot (const dVector &v1, const dVector &v2)
{
	if(v1.size() != v2.size()) 
		assert (0 && "ERROR: Dot(): Inconsistent vector size in Inner Product !");

	double	v = 0.;
	for(unsigned int i=0; i<v1.size(); ++i)
		v += v1[i] * v2[i];

	return v;
}

// dVector의 외적(cross product)을 계산한다.
inline dVector Cross (const dVector &v1, const dVector &v2)
{
	if(v1.size() != 3 || v2.size() != 3) 
		assert (0 && "ERROR: Cross(): dVector dimension should be 3 in Cross Product !");

	dVector v(3);
	v[0] = v1[1]*v2[2] - v1[2]*v2[1];
	v[1] = v1[2]*v2[0] - v1[0]*v2[2];
	v[2] = v1[0]*v2[1] - v1[1]*v2[0];

	return v;
}

inline dVector Multiply (const dVector &v1, const dVector &v2)
{
	if(v1.size() != v2.size())
		assert (0 && "ERROR: Multiply(): Inconsistent vector size in Inner Product !");

	dVector v(v1.size());
	for(unsigned int i=0; i<v1.size(); ++i)
		v[i] = v1[i] * v2[i];

	return v;
}

inline dVector operator * (const dVector &v1, const dVector &v2)
{
	if(v1.size() != v2.size())
		assert (0 && "ERROR: Multiply(): Inconsistent vector size in Inner Product !");

	dVector v(v1.size());
	for(unsigned int i=0; i<v1.size(); ++i)
		v[i] = v1[i] * v2[i];

	return v;
}

inline dVector operator * (const dVector &v1, const double &v2)
{
	dVector v(v1.size());
	for(unsigned int i=0; i<v1.size(); ++i)
		v[i] = v1[i] * v2;

	return v;
}

inline dVector operator / (const dVector &v1, const double &v2)
{
	dVector v(v1.size());
	for(unsigned int i=0; i<v1.size(); ++i)
		v[i] = v1[i] / v2;

	return v;
}

// Vector의 크기를 2-norm으로 계산한다
inline double Norm2 (const dVector &v)
{
	double s = 0.;
	for (int i=0; i<(int)v.size(); ++i) {
		s += v[i]*v[i];
	}
	return sqrt(s);
}


inline dVector GetOrientationZ (dMatrix &T)
{
	dVector v(3);

	v[0] = T(0,2);
	v[1] = T(1,2);
	v[2] = T(2,2);

	return v;
}

inline dVector GetPosition (dMatrix &T)
{
	dVector v(3);

	v[0] = T(0,3);
	v[1] = T(1,3);
	v[2] = T(2,3);

	return v;
}

inline void AttachVector (dVector &V, double a)
{
	dVector N (V.size() + 1);

	N[vSlice(0,V.size(),1)] = V;
	N[V.size()] = a;

	V = N;
}

inline void AttachVector (dVector &V, dVector &a)
{
	dVector N (V.size() + a.size());

	N[vSlice(0,V.size(),1)] = V;
	N[vSlice(V.size(),a.size(),1)] = a;

	V = N;
}

inline void AttachMatrix (dMatrix &M, dMatrix &a)
{
	assert (M.colno () == a.colno ());

	dMatrix N(M.rowno () + a.rowno (), M.colno ());

	N[mSlice(0,0,          M.rowno(),M.colno())] = M;
	N[mSlice(M.rowno(), 0, a.rowno(),a.colno())] = a;

	M = N;
}

inline dMatrix MakeMatrix (const dVector &v) 
{
	dMatrix M(v.size (), 1);
	
	for(unsigned int i=0; i<v.size(); ++i) {
		M[i][0] = v[i];
	}
	return M;
}
