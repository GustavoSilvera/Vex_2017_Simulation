#if !defined(RANDOMSTUFF_H)
#define RANDOMSTUFF_H

//random #defines and other things that are used throughout all the files
#define ppi 7.1839
#define PI 3.1415926283846387236983
#define WindowWidth  1400
#define WindowHeight  1200

inline int getSign(double value) {//returns whether a number is negative or positive.
	if (value < 0) { return -1; }
	else { return 1; }
}

#endif