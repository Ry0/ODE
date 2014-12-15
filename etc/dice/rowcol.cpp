#include <stdio.h>
#define ROW 5
#define COL 5

void DropPoint(double ary_x[], double ary_y[], int r, int c, double r_distance, double c_distance){

	for(int i=0; i<r*c; i++){
		for(int j=0; j<c; j++){
			if(i%c==j){
				ary_x[i] = c_distance*j;
			}
		}
	}
	for(int i=0; i<r*c; i++){
		for(int j=0; j<r; j++){
			if(i/r==j){
				ary_y[i] = r_distance*j;
			}
		}
	}
}


int main(void){

	double x[ROW*COL];
	double y[ROW*COL];

	DropPoint(x, y, ROW, COL, 0.5, 0.5);
	// for(int i=0; i<ROW*COL; i++){
	// 	for(int j=0; j<COL; j++){
	// 		if(i%COL==j){
	// 			x[i] = 0.5*j;
	// 		}
	// 	}
	// }
	// for(int i=0; i<ROW*COL; i++){
	// 	for(int j=0; j<ROW; j++){
	// 		if(i/ROW==j){
	// 			y[i] = 0.5*j;
	// 		}
	// 	}
	// }


	for(int i=0; i<ROW*COL; i++){
		//printf("x[%d]=%lf\n",i,x[i]);
		printf("y[%d]=%lf\n",i,y[i]);
	}

	return 0;
}