#ifndef CANONICALSTRUCTURE_H
#define CANONICALSTRUCTURE_H

class CanonicalStructure
{
	public:
		double ax , x0, Rx0;
    vec x, dx;
    vec Fx, fx;
    vec Rdx, Rx;
    double t0, tf, T;
    double taf , dtaf ;

		CanonicalStructure(int len);
		~CanonicalStructure();
    void generateFx(std::string kind, vector timed);
    double getFx(double t);
    void generateRfx(string kind, vec timed);
	protected:
};

#endif
