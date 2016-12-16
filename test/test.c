#include<stdio.h>
#include<math.h>

#define PI 3.14159265359

int main()
{
	FILE *fp;
	int data[1081] = {0};



	int front_index;
	int i,j,k,flag;
	int clcon[1081] = {0};
	int convex[1081] = {0};
	int decision[1081] = {0};
	int step[500] = {0};
	long cldis1[1081] = {0};
	
	double X = 8.0,Y = 5.0,sp = 0;
	double x,y,x0,x_t[1081] = {0},y_t[1081] = {0};
	double deg,rad,stddeg,cmpdeg,slope,radius;
	double cldis0;
	double decision_1,decision_2;
	double x_s,y_s,x_a,y_a,part_1,part_2,a,b;
	
	fp = fopen("test.txt","r");
	for(i = 0;i < 1081;i++)
		fscanf(fp,"%d",&data[i]);

	fclose(fp);

	// OûÌf[^ÌÝð\¦
	for(i = 1;i <= 1080;i++){
		printf("%d    :   %ld [mm]\n", i, data[i]);
		if(i >= 180 && i <= 900){
			//tunnel coodinate
			rad = (i - 180) * 0.25 * (PI / 180.0);	
			x_t[i] = (data[i] / 1000.0) * cos(-rad);
			y_t[i] = (data[i] / 1000.0) * sin(-rad);	
		}
	}
	//display
	for(i = 180;i <= 900;i++)
		printf("%d  x : %f   y : %f\n",i,x_t[i],y_t[i]);
	//Linear approximation
	for(i = 180;i < 540;i++){
		j = 10;
		k = 0;
		for(;;){
			if(-0.2 < x_t[i] - x_t[i + j] && x_t[i] - x_t[i + j] < 0.2){
				step[k] = i + j;
				j += 10;
				k++;
			}
			else
				j += 10;
			if(j > 500)
				break;
		}
		printf("k : %d\n",k);
		if(k >= 10){
			//sum
			x_s = 0;
			for(j = 0;j < k;j++)
					x_s += x_t[ step[j] ];
			y_s = 0;
			for(j = 0;j < k;j++)
					y_s += y_t[ step[j] ];
			//display
			printf("x_s : %f   y_s : %f\n",x_s,y_s);
			//average
			x_a = x_s / (float)k;
			y_a = y_s / (float)k;
			//display
			printf("x_a : %f   y_a : %f\n",x_a,y_a);
			//Linear equations
			part_1 = 0;
			part_2 = 0;
			for(j = 0;j < k;j++){
				part_1 += x_t[ step[j] ] * y_t[ step[j] ];
				part_2 += pow( x_t[ step[j] ] , 2.0 );
			}
			//display
			printf("part_1 : %f   part_2 : %f\n",part_1,part_2);
			a = ( part_1 - ((float)k * x_a * y_a) ) / ( part_2 - ( (float)k * pow(x_a , 2.0) ) );
			b = y_a - (a * x_a);
			printf("%d  a : %lf     b : %lf\n",i,a,b);
			flag = 1;
			break;
		}
	}
	printf("----------\n");

	for(i = 900;i > 540;i--){
		if(flag == 1)
			break;
		j = 10;
		k = 0;
		for(;;){
			if(-0.2 < x_t[i] - x_t[i - j] && x_t[i] - x_t[i - j] < 0.2){
				step[k] = i - j;
				j += 10;
				k++;
			}
			else
				j += 10;
			if(j > 500)
				break;
		}
		printf("k : %d\n",k);
		if(k >= 10){
			//sum
			x_s = 0;
			for(j = 0;j < k;j++)
					x_s += x_t[ step[j] ];
			y_s = 0;
			for(j = 0;j < k;j++)
					y_s += y_t[ step[j] ];
			//display
			printf("x_s : %f   y_s : %f\n",x_s,y_s);
			//average
			x_a = x_s / (float)k;
			y_a = y_s / (float)k;
			//display
			printf("x_a : %f   y_a : %f\n",x_a,y_a);
			//Linear equations
			part_1 = 0;
			part_2 = 0;
			for(j = 0;j < k;j++){
				part_1 += x_t[ step[j] ] * y_t[ step[j] ];
				part_2 += pow( x_t[ step[j] ] , 2.0 );
			}
			//display
			printf("part_1 : %f   part_2 : %f\n",part_1,part_2);
			a = ( part_1 - ((float)k * x_a * y_a) ) / ( part_2 - ( (float)k * pow(x_a , 2.0) ) );
			b = y_a - (a * x_a);
			printf("%d  a : %lf     b : %lf\n",i,a,b);
			break;
		}
	}

	//bottom line
	for(i = 400;i <= 700;i++){
		j = 10;
		k = 0;
		for(;;){
			if(-0.1 < y_t[i] - y_t[i + j] && y_t[i] - y_t[i + j] < 0.1){
				step[k] = i + j;
				j += 10;
				k++;
			}
			else
				j += 10;
			if(i + j > 900)
				break;
		}
		printf("k : %d\n",k);
		if(k >= 10){
			//sum
			x_s = 0;
			for(j = 0;j < k;j++)
					x_s += x_t[ step[j] ];
			y_s = 0;
			for(j = 0;j < k;j++)
					y_s += y_t[ step[j] ];
			//display
			printf("x_s : %f   y_s : %f\n",x_s,y_s);
			//average
			x_a = x_s / (float)k;
			y_a = y_s / (float)k;
			//display
			printf("x_a : %f   y_a : %f\n",x_a,y_a);
			//Linear equations
			part_1 = 0;
			part_2 = 0;
			for(j = 0;j < k;j++){
				part_1 += x_t[ step[j] ] * y_t[ step[j] ];
				part_2 += pow( x_t[ step[j] ] , 2.0 );
			}
			//display
			printf("part_1 : %f   part_2 : %f\n",part_1,part_2);
			a = ( part_1 - ((float)k * x_a * y_a) ) / ( part_2 - ( (float)k * pow(x_a , 2.0) ) );
			b = y_a - (a * x_a);
			printf("%d  a : %lf     b : %lf\n",i,a,b);
			break;
		}
	}


	
	return 0;
}
