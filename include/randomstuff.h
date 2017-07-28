#if !defined(RANDOMSTUFF_H)
#define RANDOMSTUFF_H

//random #defines and other things that are used throughout all the files
#define ppi 7.1839
#define PI 3.1415926283846387236983
#define WindowWidth  1400
#define WindowHeight  1200

inline int getSign(double value) {//returns whether a number is negative or positive.
	if (value < 0) { return -1; }
	else if (value > 0) { return 1; }
	else { return 0; }
}
inline float sqr(double value) {
	return value*value;
}
inline float SortSmallest3(float v1, float v2, float v3) {
	float smallest = v1;//initially assumes v1 is the smallest
	if (v2 < smallest) smallest = v2;//resets to v2 if its smaller than v1
	if (v3 < smallest) smallest = v3;//resets to v3 if its smaller than v1
	return smallest;
	
}
inline float SortSmallest(float v1, float v2, float v3, float v4) {//finds smallest of the list provided
	float smallest = v1;//initially assumes v1 is the smallest
	if (v2 < smallest) smallest = v2;//resets to v2 if its smaller than v1
	if (v3 < smallest) smallest = v3;//resets to v3 if its smaller than v1
	if (v4 < smallest) smallest = v4;//resets to v4 if its smaller than v1
	return smallest;
	
}
inline float Sort2ndSmallest(float v1, float v2, float v3, float v4) {//finds second smallest of the list provided
	float smallest = SortSmallest(v1, v2, v3, v4);
	float Smallest2nd;
	if (v1 == smallest)	Smallest2nd = SortSmallest3(v2, v3, v4);//exclude v1 from search to the smallest
	else if (v2 == smallest) Smallest2nd = SortSmallest3(v1, v3, v4);//exclude v2 from search to the smallest
	else if (v3 == smallest) Smallest2nd = SortSmallest3(v2, v1, v4);//exclude v3 from search to the smallest
	else if (v4 == smallest) Smallest2nd = SortSmallest3(v2, v3, v1);//exclude v4 from search to the smallest
	return Smallest2nd;
}
inline int sortSmallVER(float v1, float v2, float v3, float v4) {
	float smallestD2V = SortSmallest(v1, v2, v3, v4);
	if (smallestD2V == v1) return 0;
	else if (smallestD2V == v2) return 1;
	else if (smallestD2V == v3) return 2;
	else if (smallestD2V == v4) return 3;
}
#endif