// Quadratic.h: interface for the Quadratic class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_QUADRATIC_H__78616E1D_5BB3_4553_B9DB_547E7C309744__INCLUDED_)
#define AFX_QUADRATIC_H__78616E1D_5BB3_4553_B9DB_547E7C309744__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Quadratic  
{
public:
	Quadratic();
	Quadratic( double a, double b, double c );
	Quadratic( double x, double y, double velY, double accY );
	~Quadratic();

	void		Setup( double a, double b, double c );
	void		Setup( double x, double y, double velY, double accY );

	double		CalcMin() const;
	double		CalcY( double x ) const;
	bool		Solve( double y, double& x0, double& x1 ) const;
	bool		SmallestNonNegativeRoot( double& t ) const;

	Quadratic	operator+( const Quadratic& q ) const;
	Quadratic	operator-( const Quadratic& q ) const;

private:
	double		m_a;
	double		m_b;
	double		m_c;
};

#endif // !defined(AFX_QUADRATIC_H__78616E1D_5BB3_4553_B9DB_547E7C309744__INCLUDED_)
