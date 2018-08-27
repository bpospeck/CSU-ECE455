#include <stdio.h>
#include "vector_math.h"

#define	MAXPNT 500
#define	MAXPOLY 400
#define	MAXVERT 20
#define	MAXCHARS 100
#define	SCENES 491

FILE *fopen(), *fp, *fp_velocity;

char filename[MAXCHARS];

int nmbrpoints, nmbrpoly;
h_vector vertex[MAXPNT];
int nmbrvert[MAXPOLY];
int faces[MAXPOLY][MAXVERT];
	
h_transform pos;

initpaint()
{
	nmbrpoints=56;
	nmbrpoly=14;

	vertex[1].x=269;	vertex[1].y=230;	vertex[1].z=-109;
	vertex[2].x=269;	vertex[2].y=230;	vertex[2].z=-151;
	vertex[3].x=269;	vertex[3].y=230;	vertex[3].z=-151;
	vertex[4].x=269;	vertex[4].y=230;	vertex[4].z=-109;

	vertex[5].x=269;	vertex[5].y=360;	vertex[5].z=-109;
	vertex[6].x=269;	vertex[6].y=380;	vertex[6].z=-151;
	vertex[7].x=269;	vertex[7].y=380;	vertex[7].z=-151;
	vertex[8].x=269;	vertex[8].y=360;	vertex[8].z=-109;

	vertex[9].x=269;	vertex[9].y=360;	vertex[9].z=69;
	vertex[10].x=269;	vertex[10].y=380;	vertex[10].z=91;
	vertex[11].x=269;	vertex[11].y=380;	vertex[11].z=91;
	vertex[12].x=269;	vertex[12].y=360;	vertex[12].z=69;

	vertex[13].x=269;	vertex[13].y=230;	vertex[13].z=251;
	vertex[14].x=269;	vertex[14].y=170;	vertex[14].z=251;
	vertex[15].x=269;	vertex[15].y=170;	vertex[15].z=251;
	vertex[16].x=269;	vertex[16].y=230;	vertex[16].z=251;

	vertex[17].x=269;	vertex[17].y=100;	vertex[17].z=-151;
	vertex[18].x=269;	vertex[18].y=170;	vertex[18].z=-151;
	vertex[19].x=269;	vertex[19].y=170;	vertex[19].z=-151;
	vertex[20].x=269;	vertex[20].y=100;	vertex[20].z=-151;


	vertex[21].x=281;	vertex[21].y=359;	vertex[21].z=69;
	vertex[22].x=281;	vertex[22].y=381;	vertex[22].z=91;
	vertex[23].x=281;	vertex[23].y=381;	vertex[23].z=91;
	vertex[24].x=281;	vertex[24].y=359;	vertex[24].z=69;

	vertex[25].x=269;	vertex[25].y=359;	vertex[25].z=69;
	vertex[26].x=281;	vertex[26].y=359;	vertex[26].z=69;
	vertex[27].x=281;	vertex[27].y=359;	vertex[27].z=69;
	vertex[28].x=269;	vertex[28].y=359;	vertex[28].z=69;

	vertex[29].x=281;	vertex[29].y=360;	vertex[29].z=-109;
	vertex[30].x=281;	vertex[30].y=380;	vertex[30].z=-151;
	vertex[31].x=281;	vertex[31].y=380;	vertex[31].z=-151;
	vertex[32].x=281;	vertex[32].y=360;	vertex[32].z=-109;

	vertex[33].x=269;	vertex[33].y=360;	vertex[33].z=-109;
	vertex[34].x=281;	vertex[34].y=360;	vertex[34].z=-109;
	vertex[35].x=281;	vertex[35].y=360;	vertex[35].z=-109;
	vertex[36].x=269;	vertex[36].y=360;	vertex[36].z=-109;

	vertex[37].x=281;	vertex[37].y=231;	vertex[37].z=-151;
	vertex[38].x=281;	vertex[38].y=170;	vertex[38].z=-151;
	vertex[39].x=281;	vertex[39].y=170;	vertex[39].z=-151;
	vertex[40].x=281;	vertex[40].y=231;	vertex[40].z=-151;

	vertex[41].x=269;	vertex[41].y=231;	vertex[41].z=-109;
	vertex[42].x=281;	vertex[42].y=231;	vertex[42].z=-109;
	vertex[43].x=281;	vertex[43].y=231;	vertex[43].z=-109;
	vertex[44].x=269;	vertex[44].y=231;	vertex[44].z=-109;

	vertex[45].x=281;	vertex[45].y=100;	vertex[45].z=251;
	vertex[46].x=281;	vertex[46].y=170;	vertex[46].z=251;
	vertex[47].x=281;	vertex[47].y=170;	vertex[47].z=251;
	vertex[48].x=281;	vertex[48].y=100;	vertex[48].z=251;

	vertex[49].x=281;	vertex[49].y=230;	vertex[49].z=209;
	vertex[50].x=281;	vertex[50].y=230;	vertex[50].z=251;
	vertex[51].x=281;	vertex[51].y=230;	vertex[51].z=251;
	vertex[52].x=281;	vertex[52].y=230;	vertex[52].z=209;
	
	vertex[53].x=269;	vertex[53].y=230;	vertex[53].z=209;
	vertex[54].x=281;	vertex[54].y=230;	vertex[54].z=209;
	vertex[55].x=281;	vertex[55].y=230;	vertex[55].z=209;
	vertex[56].x=269;	vertex[56].y=230;	vertex[56].z=209;

	nmbrvert[1]=4;
	nmbrvert[2]=4;
	nmbrvert[3]=4;
	nmbrvert[4]=4;
	nmbrvert[5]=4;

	nmbrvert[6]=4;
	nmbrvert[7]=4;
	nmbrvert[8]=4;
	nmbrvert[9]=4;
	nmbrvert[10]=4;
	nmbrvert[11]=4;
	nmbrvert[12]=4;
	nmbrvert[13]=4;
	nmbrvert[14]=4;

	faces[1][1]=4; faces[1][2]=3; faces[1][3]=2; faces[1][4]=1;
	faces[2][1]=8; faces[2][2]=7; faces[2][3]=6; faces[2][4]=5;
	faces[3][1]=12; faces[3][2]=11; faces[3][3]=10; faces[3][4]=9;
	faces[4][1]=16;	faces[4][2]=15;	faces[4][3]=14;	faces[4][4]=13;
	faces[5][1]=20;	faces[5][2]=19;	faces[5][3]=18;	faces[5][4]=17;

	faces[6][1]=24;	faces[6][2]=23;	faces[6][3]=22;	faces[6][4]=21;
	faces[7][1]=28;	faces[7][2]=27;	faces[7][3]=26;	faces[7][4]=25;
	faces[8][1]=32;	faces[8][2]=31;	faces[8][3]=30;	faces[8][4]=29;
	faces[9][1]=33;	faces[9][2]=34;	faces[9][3]=35;	faces[9][4]=36;
	faces[10][1]=40;faces[10][2]=39;faces[10][3]=38;faces[10][4]=37;
	faces[11][1]=44;faces[11][2]=43;faces[11][3]=42;faces[11][4]=41;
	faces[12][1]=48;faces[12][2]=47;faces[12][3]=46;faces[12][4]=45;
	faces[13][1]=52;faces[13][2]=51;faces[13][3]=50;faces[13][4]=49;
	faces[14][1]=56;faces[14][2]=55;faces[14][3]=54;faces[14][4]=53;
}



paintgen(i)
int i;
{
float last47, last48;

	fscanf(fp_velocity,"%f%f%f%f",&pos.n.x,&pos.o.x,&pos.a.x,&pos.p.x);
	fscanf(fp_velocity,"%f%f%f%f",&pos.n.y,&pos.o.y,&pos.a.y,&pos.p.y);
	fscanf(fp_velocity,"%f%f%f%f",&pos.n.z,&pos.o.z,&pos.a.z,&pos.p.z);
	
		if (i>=10 && i<30)
		 {
			vertex[3].y=pos.p.y+20*pos.a.y;
			vertex[4].y=pos.p.y-20*pos.a.y;
		     }

		if (i>=30 && i<60)
		 {
			vertex[3].y=vertex[6].y;
			vertex[4].y=vertex[5].y;

			vertex[7].z=pos.p.z+10*pos.a.z;
			vertex[8].z=pos.p.z-10*pos.a.z;
		     }

		if (i>=60 && i<89)
		 {
			vertex[7].z=91;
			vertex[8].z=69;

			vertex[11].y=380-150*(i-60)/29;
			vertex[11].z=90+160*(i-60)/29;
			vertex[12].y=360-130*(i-60)/29;
			vertex[12].z=70+139*(i-60)/29;
		     }

		if (i>=89 && i<91)
		 {
			vertex[11].y=230;
			vertex[11].z=251;
			vertex[12].y=230;
			vertex[12].z=209;
		     }

		if (i>=91 && i<146)
		 {
			vertex[15].z=pos.p.z;
			vertex[16].z=pos.p.z;
		     }

		if (i>=146 && i<158)
		 {
			vertex[15].z=-151;
			vertex[16].z=-151;
		     }

		if (i>=159 && i<209)
		 {
			vertex[19].z=pos.p.z;
			vertex[20].z=pos.p.z;
		     }

		if (i>=209 && i<250)
		 {
			vertex[19].z=251;
			vertex[20].z=251;
		     }
	
		if (i>=260 && i<285)
		 {
			vertex[23].z=91-242*(i-260)/25;
			vertex[24].z=69-178*(i-260)/25; 
			vertex[27].z=69-178*(i-260)/25; 
			vertex[28].z=69-178*(i-260)/25; 
		     }

		if (i>=285 && i<305)
		 {
			vertex[23].z=-151;
			vertex[24].z=-109;
			vertex[27].z=-109;
			vertex[28].z=-109;

			vertex[31].y=380-210*(i-285)/20;
			vertex[32].y=360-129*(i-285)/20;
			vertex[35].y=360-130*(i-285)/20;
			vertex[36].y=360-130*(i-285)/20;
		     }

		if (i>=305 && i<380)
		 {
			vertex[31].y=170;
			vertex[32].y=231;
			vertex[35].y=230;
			vertex[36].y=230;

			vertex[39].z=pos.p.z-100*pos.a.y;
			vertex[40].z=pos.p.z-75*pos.a.y;
			if (vertex[39].z > 251) vertex[39].z=251;
			if (vertex[40].z > 251) vertex[40].z=251;
			vertex[43].z=pos.p.z;
			vertex[44].z=pos.p.z;
		     }

		if (i>=380 && i<425)
		 {
			vertex[39].z=251;
			vertex[40].z=251;
			vertex[43].z=209;
			vertex[44].z=209;

			vertex[47].z=pos.p.z+100*pos.n.z;
			vertex[48].z=pos.p.z+150*pos.n.z;
			if (vertex[47].z > 251) vertex[47].z=251;
			if (vertex[48].z > 251) vertex[48].z=251;
		     }

		last47 = vertex[47].z;
		last48 = vertex[48].z;
		if (i>=425 && i<435)
		 {
			vertex[47].z=(i-425)*(-151-last47)/10+last47;
			vertex[48].z=(i-425)*(-151-last48)/10+last48;
		     }
		if (i>=435 && i<445)
		 {
			vertex[47].z=(i-425)*(-151-last47)/20+last47;
			vertex[48].z=(i-425)*(-151-last48)/20+last48;
		     }

		if (i>=445 && i<470)
		 {
			vertex[51].y=230+150*(i-445)/25;
			vertex[51].z=250-160*(i-445)/25;
			vertex[52].y=230+130*(i-445)/25;
			vertex[52].z=209-140*(i-445)/25;
			vertex[55].y=230+130*(i-445)/25;
			vertex[55].z=209-140*(i-445)/25;
			vertex[56].y=230+130*(i-445)/25;
			vertex[56].z=209-140*(i-445)/25;
		     }

		if (i>=470)
		 {
			vertex[51].y=380;
			vertex[51].z=91;
			vertex[52].y=360;
			vertex[52].z=69;
			vertex[55].y=360;
			vertex[55].z=69;
			vertex[56].y=360;
			vertex[56].z=69;
		     }

	write_file();
}

write_file()
{
	int i,j;

        fprintf(fp,"%d %d\n", nmbrpoints,nmbrpoly);
	for (i = 1; i <= nmbrpoints; i++)
	fprintf(fp,"%f %f %f\n", vertex[i].x,vertex[i].y,vertex[i].z);
        for (i = 1; i <= nmbrpoly; i++)
        {
                fprintf(fp,"%d ", nmbrvert[i]);
                for (j = 1; j <= nmbrvert[i]; j++) fprintf(fp,"%d ", faces[i][j]);
                fprintf(fp,"\n");
        }
}

main()
{
	int scene_number;

	if ((fp_velocity = fopen("velocity", "r")) == NULL)
	{
		printf( "Error opening file <velocity> for input.\n\r");
		exit(1);
	}

	initpaint();

	for (scene_number=0; scene_number < SCENES; scene_number++) 
	{
                sprintf(filename,"paint/paint%d.pp",scene_number);
		if ((fp = fopen(filename, "w")) == NULL)
		{
			printf( "Error opening file <%s> for input.\n\r",filename);
			exit(1);
		}

		paintgen(scene_number);

		fclose(fp);
	}
}
