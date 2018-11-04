#ifndef __MATRIX_H__
#define __MATRIX_H__

#pragma warning(disable : 4996)

//#include <windows.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <cmath>

#define LPI	3.1415926535897931

template<class Tp>
class  Matrix
{
public:
		//CONSTRUCTION
		Matrix();
		Matrix( int row, int col);
		Matrix(const Matrix& x);
        ~Matrix();
		
		virtual void	Empty(void);

		//OPERATORS
		void operator=(const Matrix<Tp>& x );
		 inline Tp&				operator()(const int row, const int col);
		 inline Tp&				operator()(const long index);
		 inline const Tp&			operator()(const int row, const int col) const;
		 inline const Tp&			operator()(const long index) const;
		Matrix	operator+(const Matrix& y );
		Matrix	operator>(const Matrix& y ); //PPD
		Matrix	operator-(const Matrix& y );
		Matrix	operator*(const Matrix& y );

		Matrix			operator+(const Tp y);
		Matrix			operator>(const Tp y); //PPD
		Matrix			operator-(const Tp y);
		Matrix			operator*(const Tp y);
		Matrix			operator/(const Tp y);
		Matrix&			operator^(const Matrix y);

		int				SetDimension(const int row, const int col);
		void			GetDimension(int& row, int& col) const;
		 inline long rows() const {return m_nrow;};
		 inline long cols() const {return m_ncol;};
		 inline long width() const {return m_ncol;};
		 inline long height() const {return m_nrow;};
		 inline long size() const {return m_nrow*m_ncol;};

		Matrix				T() const;
		 inline Tp	Data(const int row, const int col) const;
		 inline Tp	Data(const int index) const;
		 inline void	InitValue(Tp value);
		 inline bool	Valid(const int row, const int col) const;
		 inline Tp*	GetData() { return m_pdata; }
		 inline Tp**	GetDataIndex() { return m_p_data_index; }

		virtual Tp		GetMin(void) const;
		virtual Tp		GetMax(void) const;
		virtual void	GetMinMax(Tp &value_min, Tp &value_max);
		virtual Tp		Sum(void) const;
		virtual Tp		Mean(void) const;
		virtual Matrix	ConvHor(Matrix& kern);	// horizontal convolution
		virtual void	SetGaussian(int dim=1, double sigma=2, int direvitive=0, const double nstd=3);	// Gaussian kernel, PPD

		virtual int		MatrixFree();
protected:
		virtual int		MatrixAlloc(const long lSize);

		Tp		*m_pdata ;
		Tp		**m_p_data_index;
		int		m_nrow,m_ncol;

private:
		virtual int MatrixAlloc(const int row, const int col);
};

template<class Tp, class Tp2>
void CopyMatrix(Matrix<Tp> &out,Matrix<Tp2> &M){
	int i,n=M.size();
	out.SetDimension(M.rows(),M.cols());
	for(i=0;i<n;i++)
		out(i)=M(i);
}

template<class Tp>
void CopyMatrixu(Matrix<unsigned char> &Mu,Matrix<Tp> &M){
	int i,n=M.size();
	int f;
	Mu.SetDimension(M.rows(),M.cols());
	for(i=0;i<n;i++){
		f=(int) (M(i));
		if (f<0) f=0;
		if (f>255) f=255;
		Mu(i)=(unsigned char)(f);
	}
}

template<class Tp>
Matrix<Tp>::Matrix()
{
	//null matrix
	m_nrow = 0;
	m_ncol = 0;
	m_pdata = NULL ;
	m_p_data_index = NULL;
}

template<class Tp>
Matrix<Tp>::Matrix(int row, int col)
{
	m_nrow = row;
	m_ncol = col;
	m_pdata = NULL ;
	m_p_data_index = NULL;

	//initialize matrix
	if (row > 0 && col >0)
	{
		if (MatrixAlloc(m_nrow, m_ncol))
		{
			printf( "Memory allocation Error in Matrix<Tp>::Matrix()! Existing...");
			exit(-1);
		}
	}
}

template<class Tp>
Matrix<Tp>::Matrix(const Matrix& x)
{//copy constructor

	m_nrow = x.m_nrow;
	m_ncol = x.m_ncol;
	MatrixAlloc(m_nrow, m_ncol);  //allocate memory

	for (int i = 0; i < rows(); i++)
		for (int j = 0; j < cols(); j++)
			(*this)(i, j) = x.Data(i, j);
}


template<class Tp>
Matrix<Tp>::~Matrix()
{  //delete the data
	MatrixFree(); //free the memory
}

template<class Tp>
int Matrix<Tp>::MatrixFree()
{  //null matrix

	if (m_pdata != NULL)
		delete[] m_pdata;           // STEP 2: DELETE THE ROWS
	m_pdata = NULL;

	if (m_p_data_index != NULL)
		delete []m_p_data_index;
	m_p_data_index = NULL;

	m_nrow = m_ncol = 0;

	return 0;
}

template<class Tp>
 inline bool	Matrix<Tp>::Valid(const int row, const int col) const
{
	if (row>=0 && row<m_nrow && col>=0 && col<m_ncol)
		return true;
	else
		return false;
}

template<class Tp>
void Matrix<Tp>::Empty(void)
{
	MatrixFree();
}


template<class Tp>
int Matrix<Tp>::MatrixAlloc(const long lSize)
{
	  try
      { 								//test for exceptions
		  m_pdata = new Tp[lSize];	//allocate memory for rows
		  m_p_data_index = new Tp*[m_nrow];
		  for (long i=0; i<m_nrow; i++)
			  m_p_data_index[i] = &(m_pdata[m_ncol*i]);
	  }
	  catch (double*)
	  {  // ENTER THIS BLOCK ONLY IF xalloc IS THROWN.
		return -1;
	  }
	  return 0;
}

template<class Tp>
int Matrix<Tp>::MatrixAlloc(const int row, const int col)
{
	if (row>=0 && col>=0)
		return MatrixAlloc(((long)row) * col);
	else
		return -1;
}

template<class Tp>
int Matrix<Tp>::SetDimension(const int row, const int col)
{  //set the dimension and allocates the memory of the matrix

	if (row!=m_nrow || col!=m_ncol)
	{
		//delete the existing data;
		MatrixFree();

		m_nrow = row;
		m_ncol = col;

		if (row>0 && col>0)
			MatrixAlloc(row, col);
	}

	return 0; //could be used to return the success
				  // or failure of allocation
}

template<class Tp>
void Matrix<Tp>::GetDimension(int& row, int& col) const
{  
	//report the dimension of the matrix
	row = m_nrow;
	col = m_ncol;
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator> (const Matrix<Tp>& y) //PPD
{  //see if x is greater than y, elt by elt
	Matrix<Tp> temp(m_nrow, m_ncol); //initialize the temporary matrix
	for(int i = 0; i < rows(); i++)
		for(int j = 0; j < cols(); j++)
			temp(i, j) = Data(i, j) > y.Data(i, j); 
	  return temp; //return the new matrix
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator> (const Tp y) // PPD
{	//see if matrix is greater than constant, elt by elt
	Matrix<Tp> temp(rows(), cols()); //initialize the temporary matrix
	for(int i = 0; i < m_nrow; i++)
		for(int j = 0; j < m_ncol; j++)
			temp(i, j) = (*this)(i,j)>y;
	  return temp; //return the matrix
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator+ (const Matrix<Tp>& y)
{  //add two matrices

	Matrix<Tp> temp(m_nrow, m_ncol); //initialize the temporary matrix

	for(int i = 0; i < rows(); i++)
		for(int j = 0; j < cols(); j++)
			temp(i, j) = Data(i, j) + y.Data(i, j); //add element by element

	  return temp; //return the new matrix
}


template<class Tp>
Matrix<Tp> Matrix<Tp>::operator+ (const Tp y)
{
	//add a constant to a matrices
	Matrix<Tp> temp(rows(), cols()); //initialize the temporary matrix

	for(int i = 0; i < m_nrow; i++)
		for(int j = 0; j < m_ncol; j++)
			temp(i, j) = (*this)(i,j)+y;  //add element by element

	  return temp; //return the matrix
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator - (const Matrix<Tp>& y)
{  //subtract two matrix
	Matrix<Tp> temp;
	
	temp.SetDimension(rows(),cols()); //intialize the temporary matrix

	for (int i = 0; i < rows(); i++)
		for (int j = 0; j<cols(); j++)
			temp(i, j) = Data(i, j)- y.Data(i, j); //sub element by element

	return temp; //return the new matrix
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator - (const Tp y)
{  //subtract a constant from a matrix
	Matrix<Tp> temp(rows(), cols()); //initialize the temporary matrix

	for (int i = 0; i < m_nrow; i++)
		for (int j = 0; j < m_ncol; j++ )
			temp(i,j) = (*this)(i, j)-y; //subtract element by element

	  return temp; //return the matrix
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator*(const Matrix<Tp>& y)
{  
	//multiply two matrices
	Tp			sum;
	Matrix<Tp>	temp;
	
	temp.SetDimension(m_nrow, y.m_ncol); //initialize the temporary matrix

	for (int i = 0; i < m_nrow; i++)
		for (int j = 0; j < y.m_ncol; j++)
		{
			sum = 0;
			for(int k = 0; k < m_ncol; k++)
				sum = sum+Data(i, k)*y.Data(k, j); //multiply

			temp(i, j) = sum;
		}

	  return temp; //return the result of multiplication
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator * (const Tp y)
{  //multiply by a scalar constant
	Matrix<Tp> temp(rows(), cols()); //initialize the temporary matrix

	for (int i = 0; i < m_nrow; i++)
		for (int j = 0; j < m_ncol; j++)
			temp(i,j) = (*this)(i, j)*y; //scale element by element

	  return temp; //return the matrix
}

template<class Tp>
Matrix<Tp>&	Matrix<Tp>::operator^(const Matrix<Tp> y)
{
	for (long i=0; i<size(); i++)
		(*this)(i) = (*this)(i)*y.Data(i);
	return *this;
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::operator / (const Tp y)
{  //multiply by a scalar constant
	Matrix<Tp> temp;
	
	temp.SetDimension(rows(), cols()); //initialize the temporary matrix

	for (int i = 0; i < m_nrow; i++)
		for (int j = 0; j < m_ncol; j++)
			temp(i,j) = (*this)(i, j)/y; //scale element by element

	  return temp; //return the matrix
}

template<class Tp>
 inline Tp& Matrix<Tp>::operator ()(const int row, const int col)
{
	assert( Valid(row, col) ); // out of bounds ?
	return m_p_data_index[row][col];
}

template<class Tp>
 inline Tp& Matrix<Tp>::operator ()(const long index)
{
	assert( index>=0 && index<m_ncol*m_nrow ); // out of bounds ?
	return m_pdata[index];
}

template<class Tp>
 inline const Tp& Matrix<Tp>::operator ()(const int row, const int col) const
{
	assert( Valid(row, col) ); // out of bounds ?
	return m_p_data_index[row][col];
}

template<class Tp>
 inline const Tp& Matrix<Tp>::operator ()(const long index) const
{
	assert( index>=0 && index<m_ncol*m_nrow ); // out of bounds ?
	return m_pdata[index];
}

template<class Tp>
void Matrix<Tp>::operator=(const Matrix<Tp>& x)
{//performs assignment statement

	//beware of self assignments : x = x
	if ( this != &x )
	{
		int i,j;

		if (rows()!=x.rows() || cols()!=x.cols())
		{
			//delete the existing data;
			MatrixFree();

			m_nrow = x.m_nrow;
			m_ncol = x.m_ncol;
			MatrixAlloc(m_nrow, m_ncol); //allocate memory
		}

		for(i=0; i<m_nrow; i++)
			for(j=0; j<m_ncol; j++)
				(*this)(i, j) = (Tp)(x.Data(i, j));
	}
	return ;
}

template<class Tp>
Matrix<Tp> Matrix<Tp>::T() const
{//transposes of a matrix
	Matrix<Tp> temp;
	
	temp.SetDimension(m_ncol, m_nrow); //initialize a temp matrix

	for (int i = 0; i < m_ncol; i++)
		for (int j = 0; j < m_nrow; j++)
			temp(i, j) = Data(j, i); //transpose elements

	 return temp;
}

template<class Tp>
Tp Matrix<Tp>::Data(const int row, const int col) const
{
	return m_p_data_index[row][col];
}

template<class Tp>
Tp Matrix<Tp>::Data(const int index) const
{
	return m_pdata[index];
}


template<class Tp>
 inline	void Matrix<Tp>::InitValue(Tp value)
{
	int i,j;

	if (m_pdata != NULL)
	{
		for (i = 0; i < rows(); i++)
			for (j = 0; j < cols(); j++)
				(*this)(i, j) = value;
	}
}

template<class Tp>
Tp Matrix<Tp>::GetMin(void) const
{
	Tp value=0;

	if (size()>0)
	{
		long i;
		value = Data(0);
		for (i=1; i<size(); i++)
			if (Data(i)<value)
				value = Data(i);
	}

	return value;
}

template<class Tp>
Tp Matrix<Tp>::GetMax(void) const
{
	Tp value=0;

	if (size()>0)
	{
		long i;
		value = Data(0);
		for (i=1; i<size(); i++)
			if (Data(i)>value)
				value = Data(i);
	}

	return value;
}

template<class Tp>
void Matrix<Tp>::GetMinMax(Tp &value_min, Tp &value_max)
{
	if (size()>0)
	{
		long	i;
		Tp		value;

		value_min = Data(0);
		value_max = Data(0);
		for (i=1; i<size(); i++)
		{
			value = Data(i);

			if (value < value_min)
				value_min = value;
			if (value > value_max)
				value_max = value;
		}
	}
}

// the summation of all the elements
template<class Tp>
Tp Matrix<Tp>::Sum(void) const
{
	Tp	value;
	value = 0;
	for (long i=0; i<size(); i++)
		value = value+Data(i);

	return value;
}

template<class Tp>
Tp Matrix<Tp>::Mean(void) const
{
	Tp	value;
	long temp;

	value = 0;
	temp = size();
	if (temp>0)
		value = (Tp)(Sum()/temp);
	return value;
}

 inline double	gauss(double x, double sigma)
	{return exp(-x*x/(2*sigma*sigma)) / (sigma*sqrt(2*LPI));};
 inline double	dgauss(double x, double sigma)
	{return -x*gauss(x,sigma)/(sigma*sigma);};
 inline double	d2gauss(double x, double sigma)
	{return (x*x/pow(sigma,4)-1/pow(sigma,2))*gauss(x,sigma);};

//////////////////////////////////////////////////////
// kernel generation
/*   dim : dimension 1: one
					 2: two
	 sigma: sigmal value
	 direvite: 0:
			   1: first
			   2: second
*/
template<class Tp>
void Matrix<Tp>::SetGaussian(int dim, double sigma, int direvitive, const double nstd)
{
	Matrix<Tp>	op;
	int			size,i,j,u;
	double		x,y;

	size = ((int) (sigma*nstd))*2 +1;
	u = (size-1)/2;

	assert(size > 0);

	// gaussian kernel
	if (dim == 1)
	{
		op.SetDimension(1,size);
		op.InitValue(0);
		for (i=0; i<size; i++){
			x = double(i-u);
			switch (direvitive){
			case 0:	// Gaussian function
					op(0,i) = (Tp)gauss(x,sigma);
					break;
			case 1:	// First dirivtive of Gaussian
					op(0,i) = (Tp)dgauss(x,sigma);
					break;
			case 2: // Second dirivtive of Gaussian
					op(0,i) = (Tp)d2gauss(x,sigma);
					break;
			}
		}
	}
	else{
		double	std2=sigma*sigma;
		double	h1;

		op.SetDimension(size,size);
		op.InitValue(0);
		switch(direvitive){
		case 0:
			for (j=0; j<size; j++)
				for (i=0; i<size; i++){
					x = double(j-u);
					y = double(i-u);

					op(j,i) = (Tp)gauss(sqrt(x*x+y*y),sigma);
				}
			op = op/op.Sum();
			break;
		case 2:
			for (j=0; j<size; j++)
				for (i=0; i<size; i++){
					x = double(j-u);
					y = double(i-u);

					h1 = exp(-(x*x+y*y)/(2*std2));
					op(j,i) = (Tp)(h1*(x*x+y*y-2*std2)/(2*LPI*(pow(sigma,6))));
				}
			break;
		}
	}
	*this = op;
}

// horizontal convolution
template<class Tp>
Matrix<Tp> Matrix<Tp>::ConvHor(Matrix& kern){
	Matrix<Tp>	I;

	I.SetDimension(rows(), cols());
	long i,j,k,l,m,r=rows(),c=cols();

	I.InitValue(0);
	l = kern.cols();
	for (j=0; j<r; j++)
		for (i=0; i<c; i++){
			for (k=0; k<l; k++){
				m = i-(k-(l-1)/2);
				if (m>=0 && m<c)
					I(j,i) += (*this)(j,m)*kern(k);
			}
		}
	return I;
}

#define TINY 1.0e-20;

template<class Tp>
int LUdcmp(Matrix<Tp>& X, Matrix<Tp>& indx, Tp& d){
	int i,imax=0, j, k;
	Tp big, dum, sum, temp;
	int xnrow=X.rows(),xncol=X.cols();

	Matrix<Tp> vv(1,xnrow); //vv stores the implicit scaling of each nrow

	d = (Tp)1.0;			  //no nrow interchange yet
	for (i=0;i<xnrow;i++){    //loop to get implicit scaling information
		big = 0.0;
		for (j=0;j<xncol;j++)
			if ((temp=fabs(X(i,j))) > big) big=temp;

		if (big == 0.0){
			 std::cerr<<"Singular matrix in routine LUDCMP"<<std::endl;
			 return -1;
		}

		vv(0,i)=(Tp)1.0/big;  //save the scaling
	}

	for (j=0;j<xnrow;j++){
	    for (i=0;i<j;i++){
			sum = X(i,j);
			for (k=0;k<i;k++) sum -= X(i,k)*X(k,j);
			  X(i,j)=sum;
		}
		big=0.0;
		for (i=j;i<xnrow;i++){
			sum = X(i,j);
			for (k=0;k<j;k++)
				sum -= X(i,k)*X(k,j);

			X(i,j)=sum;
			if ( (dum=vv(0,i)*fabs(sum)) >= big){
				big=dum;
				imax=i;
			}
		}

		if (j != imax){
			for (k=0;k<xnrow;k++){
				dum = X(imax,k);
				X(imax,k)= X(j,k);
				X(j,k)=dum;
			}
			d = -(d);            			// change the parity of d
			vv(0,imax)=vv(0,j);				// Also interchange the scale factor
		}

		indx(0,j) = (Tp)imax;
		if (X(j,j) == (Tp)0.0) X(j,j)=(Tp)TINY;
		if (j != (xnrow-1)){
			dum = (Tp)1.0/(X(j,j));
			for (i=j+1;i<xnrow;i++) X(i,j) *= dum;
		}
	}
	return 0;
}
#undef TINY

template<class Tp>
inline int LUbksb(Matrix<Tp>& X, Matrix<Tp>& indx, Matrix<Tp>& B){
	int i, ii=-1, ip, j;
	Tp sum;
	int xnrow=X.rows();
  //int xncol=X.cols(); //local variable is initialized but not referenced

	for (i=0;i<xnrow;i++){
		ip  = (int) indx(0,i);
		sum = B(ip, 0);
		B(ip, 0) = B(i,0);
		if (ii>=0)
			for (j=ii;j<=(i-1);j++) sum -= X(i,j)*B(j,0);
		else if (sum) ii=i;
		B(i,0)=sum;
	}

	for (i=(xnrow-1);i>=0;i--){
		sum = B(i,0);
		for (j=i+1;j<xnrow;j++) sum -= X(i,j)*B(j,0);
		B(i,0)=sum/X(i,i);
	}

	return 0;
}

template<class Tp>
void Inv(Matrix<Tp> &result, const Matrix<Tp> &in){
	//computes the inverse of the matrix

	Tp	d; //interchange flag
	int		i,j,rows=in.rows(),cols=in.cols();

	Matrix<Tp> temp(in); 		//make a copy of the original matrix
	Matrix<Tp> col(rows,1) ;     //temporary variable
	Matrix<Tp> index(1,rows) ;
	
	result.SetDimension(rows,cols);    //the final result

	if( LUdcmp(temp, index, d) ){  //decompose the matrix
//		std::cerr<<"\n\n\t Error Singluar matrix...."<<std::endl;
		result.InitValue(0.0);
	}

	for(j=0; j<rows; j++){
		col.InitValue(0);
		col(j,0) = 1;
		LUbksb(temp,index,col);
		for(i=0; i<rows; i++)
			result(i,j) = col(i,0);
	}
}
#endif // __MATRIX_H__

/*[ Code Review -----------------------------------------------------------------------------------------
   Review method         		: 4-Eye-Check 
   Review date             		: 25-July-2008
   Review members      			: Yefeng Zheng & David Liu SCR-DS
   Review result            	: accepted
   Review coverage       		: all 
   Hazard code modified since
   previous inspection? 		: n.a. 
   Label reviewed	     		: SCR_CARDIACCT_HEARTCHAMBER_BASELINE_20080701
]-------------------------------------------------------------------------------------------------------*/
