#include <math.h>
#include <stdio.h>

#include <glut.h>

#include "vector_math.h"

#define BIG 1000000     /* used for minimum vertex initialization */

#define PI 3.141592654

#define MAX_ROBOTS 10
#define MAX_OBJECTS 20
#define MAX_CHARS 100

/* display parameters */
#define AR 1            /* aspect ratio of screen */
#define NEAR 0.1      /* distance to closest z clipping plane */
#define FAR 10000.0        /* distance to furthest z clipping plane */
#define HOR 500         /* screen resolution in x */
#define VER 500         /* screen resolution in y */

/* GLOBALS */

/* file pointers and file names */
FILE *fopen(), *fp, *fp_ang[MAX_ROBOTS], *fp_traj[MAX_OBJECTS], *fp_view, *fp_script;
char filename[MAX_CHARS], robotname[MAX_ROBOTS][MAX_CHARS], objectname[MAX_OBJECTS][MAX_CHARS];

/* viewing data */
h_vector ep;    /* Eye Point: the position of the viewer */
h_vector coi;   /* Center Of Interest: the point that the viewer is looking at */
float va;       /* View Angle: angular amount (radians) of world seen on screen */

/* robot kinematics data */
int number_robots;		/* number of robots in the simulation */ 
int *nmbrlinks;			/* number of links in a robot */ 
h_transform *base;		/* world to robot base transform */
float **a,**d,**alpha,**theta;	/* DH parameters */
char **joint_type;    

/* graphic data for each robot */
int **nmbrpoints;	/* number of points in a given link */
int **nmbrpoly;		/* number of polygons in a given link */
int ***nmbrvert;	/* number of vertices in a given polygon and link */
int ****faces;		/* vertex list for all polygons in the robot */
h_vector ***screen;	/* eventual screen coordinates of all points in the robot */
h_vector ***point;	/* coordinates of all points in the robot referenced */
					/* to individual link coordinate frames */

/* graphic data for each object */
int number_objects;	/* number of object in the simulation */ 
int *object_type;	/* object type: 0=static, 1=dynamic, 2=morphing */
int *o_nmbrpoints;	/* number of points in an object*/
int *o_nmbrpoly;	/* number of polygons in an object */
int **o_nmbrvert;	/* number of vertices in a given polygon */
int ***o_faces;		/* vertex list for all polygons in the object */
h_vector **o_screen;/* eventual screen coordinates of all points in the object */
h_vector **o_point;	/* coordinates of all points in the object */
h_transform *o_base;/* world to object base transform */

/* simulation data */
int scene;			/* current scene being displayed */
int direction;		/* forward or reverse animation */
int nmbrscenes;		/* total number of scenes (frames) in the animation */
int maxpoly;		/* maximum number of poloygons in a scene */
int *totnmbrpolys;	/* total number of visible polygons in a scene */
int **nvert;		/* number of vertices in a visible polygon for a given scene */

h_vector ***polygon;


define_view() /* Reads in the Point of View Parameters */
{
	fscanf(fp_view,"%f%f%f", &ep.x,&ep.y,&ep.z);
        fscanf(fp_view,"%f%f%f", &coi.x,&coi.y,&coi.z);
        fscanf(fp_view,"%f", &va);
}


h_transform perspective_transform()  
/* Calculates the homogeneous perspective transformation */
{
        h_transform persp;
 
        float a,b,c;      /* components of vector from coi to ep */
        float d;          /* distance from coi to ep */
        float cot2;       /* cotangent of half the view angle */
        float cosb,sinb;  /* used to rotate a,b,c onto y-z plane */
        float cosc,sinc;  /* used to rotate a,b,c onto z axis */
        float alpha,beta; /* coefficients for z component perspective */
 
        /* define vector from coi to ep */
        a=ep.x-coi.x;
        b=ep.y-coi.y;
        c=ep.z-coi.z;
        d=sqrt(a*a+b*b+c*c);
 
        /* define rotation parameters */
        cosb=c/sqrt(a*a+c*c);
        sinb=a/sqrt(a*a+c*c);
        cosc=sqrt(a*a+c*c)/d;
        sinc=b/d;
 
        /* define z axis perspective transform */
        alpha=FAR/(FAR-NEAR);
        beta=-FAR*NEAR/(FAR-NEAR);
 
        /* cone of vision */
        cot2=cos(va/2)/sin(va/2);
 
        /* point of view transform which is the concantenation of */ 
		/* a translation (from coi to origin), two rotations, a   */
        /* change to left-handed space with origin at ep and the  */
        /* perspective transform */

        persp.n.x=cosb*cot2;
        persp.n.y=-sinb*sinc*AR*cot2;
        persp.n.z=-sinb*cosc*alpha;
        persp.n.w=-sinb*cosc;
        
        persp.o.x=0;
        persp.o.y=cosc*AR*cot2;
        persp.o.z=-sinc*alpha;
        persp.o.w=-sinc;
        
        persp.a.x=-sinb*cot2;
        persp.a.y=-cosb*sinc*AR*cot2;
        persp.a.z=-cosb*cosc*alpha;
        persp.a.w=-cosb*cosc;
        
        persp.p.x=-coi.x*cosb*cot2+coi.z*sinb*cot2;
        persp.p.y=coi.x*sinb*sinc*AR*cot2-coi.y*cosc*AR*cot2
                   +coi.z*cosb*sinc*AR*cot2;
        persp.p.z=coi.x*sinb*cosc*alpha+coi.y*sinc*alpha
                   +coi.z*cosb*cosc*alpha+alpha*d+beta;
        persp.p.w=coi.x*sinb*cosc+coi.y*sinc+coi.z*cosb*cosc+d;
 
        return(persp);
}
 
void display(void) /* displays one scene (frame) of the animation */
{
	int i, j;

    /* clear screen */
    glClear(GL_COLOR_BUFFER_BIT);

	/* draw all visible polygons in this scene */
    for (i = 0; i < totnmbrpolys[scene]; i++)
	{
		glBegin(GL_LINE_LOOP);
		for (j = 0; j < nvert[scene][i]; j++)	
		glVertex3d(polygon[scene][i][j].x, polygon[scene][i][j].y, polygon[scene][i][j].z);
		glEnd();
	}


	glutSwapBuffers();
	glutPostRedisplay();

	/* update current scene number */
	scene = (scene + direction) % nmbrscenes;
	if (scene < 0) scene = nmbrscenes-1;

	/* wait */
	for (i=0; i < 1000000; i++)
		j=sqrt(i);

}

calculate_objects(m)
int m; /* scene number */
{
	int i,j,n,dummy1,dummy2,minvert;
	h_transform traj;
	float znorm; /* z component of surface normal for visibility check */
	h_vector v1,v2;

    	for (n = 0; n < number_objects; n++)
	{
	   if (object_type[n] == 2) /* morphing object */
	   {
		sprintf(filename,"%s/%s/%s%d.pp",objectname[n],objectname[n],objectname[n],m);
		if ((fp = fopen(filename, "r")) == NULL)
		{
			printf( "Error opening file <%s> for input.\n\r",filename);
			exit(1);
		}

		/* read in morphed object */
        	/* Number of Points, Number of Polygons */
		fscanf(fp,"%d%d", &dummy1,&dummy2);
		if (dummy1 != o_nmbrpoints[n]) printf("ERROR: Different number of points in %s\n",filename);
		if (dummy2 != o_nmbrpoly[n]) printf("ERROR: Different number of polygons in %s\n",filename);

        	/* Read new x, y, z position of all objects points */
        	for (i = 0; i < o_nmbrpoints[n]; i++)
		{
                   fscanf(fp,"%f%f%f", &o_point[n][i].x,&o_point[n][i].y,&o_point[n][i].z);
                   o_point[n][i].w=1;
		}

		/* NOTE: It is assumed that the number of polygons stays the same and */
		/*       that the number of vertices is less than or equal to that in */
		/*       the original object since only this amount of memory was allocated */

        	/* Read in Polygons */
        	minvert = BIG;
        	for (i = 0; i < o_nmbrpoly[n]; i++)
        	{
                	fscanf(fp,"%d", &o_nmbrvert[n][i]);

                	for (j = 0; j < o_nmbrvert[n][i]; j++)
                	{
                        	fscanf(fp,"%d", &o_faces[n][i][j]);
                        	if (o_faces[n][i][j] < minvert) minvert = o_faces[n][i][j];
                	}
        	}

		/* check vertex numbering */ 
        	if (minvert == 1) /* PASCAL indexing so shift by 1 */
        	{
                	for (i = 0; i < o_nmbrpoly[n]; i++)
                	for (j = 0; j < o_nmbrvert[n][i]; j++) o_faces[n][i][j]--;
        	}
 

		fclose(fp);
	   }

	    /* apply the base transformation */
  	    for (i = 0; i < o_nmbrpoints[n]; i++) 
	    o_screen[n][i] = multTxV(o_base[n],o_point[n][i]);
 	
	   if (object_type[n] == 1) /* dynamic object */
	   {
		fscanf(fp_traj[n],"%f%f%f%f",&traj.n.x,  &traj.o.x,  &traj.a.x,  &traj.p.x);
		fscanf(fp_traj[n],"%f%f%f%f",&traj.n.y,  &traj.o.y,  &traj.a.y,  &traj.p.y);
		fscanf(fp_traj[n],"%f%f%f%f",&traj.n.z,  &traj.o.z,  &traj.a.z,  &traj.p.z);
					      traj.n.w=0; traj.o.w=0; traj.a.w=0; traj.p.w=1;

		for (i = 0; i < o_nmbrpoints[n]; i++) 
		o_screen[n][i] = multTxV(traj,o_screen[n][i]);
	   }

	   /* transform all object points to normalized screen space */

	   /* point of view transformation */
  	   for (i = 0; i < o_nmbrpoints[n]; i++) 
	   o_screen[n][i] = multTxV(perspective_transform(),o_screen[n][i]);

    	   /* Normalize vectors to perform the perspective scaling */
    	   for (i = 0; i < o_nmbrpoints[n]; i++)
   	   {
		o_screen[n][i].x=o_screen[n][i].x/o_screen[n][i].w;
		o_screen[n][i].y=o_screen[n][i].y/o_screen[n][i].w;
		o_screen[n][i].z=o_screen[n][i].z/o_screen[n][i].w;
     	    }

	   /* final screen transformations */

       for (i = 0; i < o_nmbrpoly[n]; i++)
	   {
		/* do visibility check on the polygon */
		v1.x=o_screen[n][o_faces[n][i][1]].x-o_screen[n][o_faces[n][i][0]].x;
        v1.y=o_screen[n][o_faces[n][i][1]].y-o_screen[n][o_faces[n][i][0]].y;
        v2.x=o_screen[n][o_faces[n][i][2]].x-o_screen[n][o_faces[n][i][1]].x;
        v2.y=o_screen[n][o_faces[n][i][2]].y-o_screen[n][o_faces[n][i][1]].y;

		/* z component of polygon surface normal */
        znorm=v1.x*v2.y-v2.x*v1.y;

        if (znorm>0) /* if visible include in scene */
		{
            nvert[m][totnmbrpolys[m]] = o_nmbrvert[n][i];

			/* allocate memory */
		    polygon[m][totnmbrpolys[m]] = 
		   	(h_vector *)malloc((unsigned) 
			(o_nmbrvert[n][i])*sizeof(h_vector));

           for (j = 0; j < o_nmbrvert[n][i]; j++)
		   {
		      /* device transform */
	          polygon[m][totnmbrpolys[m]][j].x = o_screen[n][o_faces[n][i][j]].x*HOR/2+HOR/2;
		      polygon[m][totnmbrpolys[m]][j].y = o_screen[n][o_faces[n][i][j]].y*VER/2+VER/2;
 		      polygon[m][totnmbrpolys[m]][j].z = o_screen[n][o_faces[n][i][j]].z;
		   }
		   totnmbrpolys[m]++; /* total number of visible polys in this scene */
		}
	   }
     }
}

h_transform link_transform(a,d,alpha,theta)
/* calculate homgeneous transformation matrix given the DH parameters */
float a;	/* length */
float d;	/* offset */
float alpha;	/* twist */
float theta;	/* angle */
{
	h_transform matrix;
	float cosalpha,sinalpha;
	float costheta,sintheta;

	cosalpha=cos(alpha);
	sinalpha=sin(alpha);

	costheta=cos(theta);
	sintheta=sin(theta);

	matrix.n.x=costheta;
	matrix.n.y=sintheta;
	matrix.n.z=0;
	matrix.n.w=0;

	matrix.o.x=-cosalpha*sintheta;
	matrix.o.y=cosalpha*costheta;
	matrix.o.z=sinalpha;
	matrix.o.w=0;
	
	matrix.a.x=sinalpha*sintheta;
	matrix.a.y=-sinalpha*costheta;
	matrix.a.z=cosalpha;
	matrix.a.w=0;

	matrix.p.x=a*costheta;
	matrix.p.y=a*sintheta;
	matrix.p.z=d;
	matrix.p.w=1;

	return(matrix);
}


redfnarm(robot_number) /* calculate position of all robot links (points) for given configuration */
int robot_number;
{
	h_transform matrix;
	int i,j;

	/* begin with robot base to world transform */
	matrix = assign_transform(base[robot_number]);

	/* transform the points for link 0 to the world coordinate frame */
	for (j = 0; j < nmbrpoints[robot_number][0]; j++)
	screen[robot_number][0][j] = multTxV(matrix,point[robot_number][0][j]);

	/* concatenate the transformations along the robot arm */
        for (i = 1; i <= nmbrlinks[robot_number]; i++)
	{
		/* include the current link transform */
		matrix = multTxT(matrix,link_transform(a[robot_number][i],d[robot_number][i],alpha[robot_number][i],theta[robot_number][i]));

		/* transform the points for link i to the world coordinate frame */
		for (j = 0; j < nmbrpoints[robot_number][i]; j++)
		screen[robot_number][i][j] = multTxV(matrix,point[robot_number][i][j]);
	}
}


precalculate() /* precalculate all screen polygons for all scenes of the animation */
{
	int i,j,k,m,n;
	float znorm; /* z component of surface normal for visibility check */
	h_vector v1,v2;

	/* allocate memory */
        totnmbrpolys = (int *)malloc((unsigned) (nmbrscenes)*sizeof(int));
        nvert = (int **)malloc((unsigned) (nmbrscenes)*sizeof(int *));
        polygon = (h_vector ***)malloc((unsigned) 
		  (nmbrscenes)*sizeof(h_vector **));

   for (m = 0; m < nmbrscenes; m++)
   {
	define_view();

 	/* allocate memory for maximum number of polygons in scene */
	nvert[m] = (int *)malloc((unsigned) (maxpoly)*sizeof(int));
	polygon[m] = (h_vector **)malloc((unsigned) 
		     (maxpoly)*sizeof(h_vector *));

	/* total number of polys per scene is different due to visibility */
	totnmbrpolys[m] = 0; 

	calculate_objects(m);

     for (n = 0; n < number_robots; n++)
     {
    /* read in the robots joint variables for this scene */
    for (i = 1; i <= nmbrlinks[n]; i++)
    if (joint_type[n][i] == 'r') fscanf(fp_ang[n],"%f",&theta[n][i]);
                            else fscanf(fp_ang[n],"%f",&d[n][i]);
  
	/* determine the position of all links in world space */
	redfnarm(n);

	/* transform all robot points to normalized screen space */
	for (k = 0; k <= nmbrlinks[n]; k++) 
	{
	   /* point of view transformation */
  	   for (i = 0; i < nmbrpoints[n][k]; i++) 
	   screen[n][k][i] = multTxV(perspective_transform(),screen[n][k][i]);

    	   /* Normalize vectors to perform the perspective scaling */
    	   for (i = 0; i < nmbrpoints[n][k]; i++)
   	   {
		screen[n][k][i].x=screen[n][k][i].x/screen[n][k][i].w;
		screen[n][k][i].y=screen[n][k][i].y/screen[n][k][i].w;
 		screen[n][k][i].z=screen[n][k][i].z/screen[n][k][i].w;
    	    }
	}

	/* final screen transformations */
 
	for (k = 0; k <= nmbrlinks[n]; k++) 
	{
           for (i = 0; i < nmbrpoly[n][k]; i++)
	   {
		/* do visibility check on the polygon */
		v1.x=screen[n][k][faces[n][k][i][1]].x-screen[n][k][faces[n][k][i][0]].x;
        v1.y=screen[n][k][faces[n][k][i][1]].y-screen[n][k][faces[n][k][i][0]].y;
        v2.x=screen[n][k][faces[n][k][i][2]].x-screen[n][k][faces[n][k][i][1]].x;
        v2.y=screen[n][k][faces[n][k][i][2]].y-screen[n][k][faces[n][k][i][1]].y;

		/* z component of polygon surface normal */
        znorm=v1.x*v2.y-v2.x*v1.y;

        if (znorm>0) /* if visible include in scene */
		{
            nvert[m][totnmbrpolys[m]] = nmbrvert[n][k][i];
 
			/* allocate memory */
		    polygon[m][totnmbrpolys[m]] = 
		   	(h_vector *)malloc((unsigned) 
			(nmbrvert[n][k][i])*sizeof(h_vector));

           for (j = 0; j < nmbrvert[n][k][i]; j++)
		   {
		      /* device transform */
	          polygon[m][totnmbrpolys[m]][j].x = 
			  screen[n][k][faces[n][k][i][j]].x*HOR/2+HOR/2;
		      polygon[m][totnmbrpolys[m]][j].y = 
			  screen[n][k][faces[n][k][i][j]].y*VER/2+VER/2;
		      polygon[m][totnmbrpolys[m]][j].z = 
			  screen[n][k][faces[n][k][i][j]].z;
           }
		   totnmbrpolys[m]++; /* total number of visible polys in this scene */
		}
	   }
	}
     }
   }
}


define_robot(robot_number)  /* read in the Denavit and Hartenberg parameters */
int robot_number;	    /* and point-polygon description of the robot    */
{
	int i,j,k;
	int minvert; /* used to check minimum vertex index to see if 0 */

	sprintf(filename,"%s/%s.dh",robotname[robot_number],robotname[robot_number]);
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}

	fscanf(fp,"%d", &nmbrlinks[robot_number]);

	/* allocate memory */
        a[robot_number] = vector(1,nmbrlinks[robot_number]); /* arrays from 1 to nmbrlinks[robot_number] */
        d[robot_number] = vector(1,nmbrlinks[robot_number]);
        alpha[robot_number] = vector(1,nmbrlinks[robot_number]);
        theta[robot_number] = vector(1,nmbrlinks[robot_number]);
        joint_type[robot_number] = cvector(1,nmbrlinks[robot_number]); 
	/* arrays from 0 to numberlinks */
        nmbrpoints[robot_number] = (int *)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(int));
        nmbrpoly[robot_number] = (int *)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(int));
        nmbrvert[robot_number] = (int **)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(int *));
        faces[robot_number] = (int ***)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(int **));
        point[robot_number] = (h_vector **)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(h_vector *));
        screen[robot_number] = (h_vector **)malloc((unsigned) (nmbrlinks[robot_number]+1)*sizeof(h_vector *));


	for (i = 1; i <= nmbrlinks[robot_number]; i++)
	fscanf(fp,"%f%f%f%f%1s",&a[robot_number][i], &d[robot_number][i], &alpha[robot_number][i], &theta[robot_number][i], &joint_type[robot_number][i]);

	fclose(fp);

  	printf("\nRobot D_H Parameters\n");
  	printf("%d Degrees of Freedom\n", nmbrlinks[robot_number]);
  	printf("Link#\tLength\t\tOffset\t\tTwist\t\tAngle\t\tType\n");
	for (i = 1; i <= nmbrlinks[robot_number]; i++)
	printf("%d\t%f\t%f\t%f\t%f\t%c\n",i,a[robot_number][i], d[robot_number][i], alpha[robot_number][i], theta[robot_number][i], joint_type[robot_number][i]);

	sprintf(filename,"%s/%s.base",robotname[robot_number],robotname[robot_number]);
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}
	fscanf(fp,"%f%f%f%f",&base[robot_number].n.x,  &base[robot_number].o.x,  &base[robot_number].a.x,  &base[robot_number].p.x);
    fscanf(fp,"%f%f%f%f",&base[robot_number].n.y,  &base[robot_number].o.y,  &base[robot_number].a.y,  &base[robot_number].p.y);
    fscanf(fp,"%f%f%f%f",&base[robot_number].n.z,  &base[robot_number].o.z,  &base[robot_number].a.z,  &base[robot_number].p.z);
        			      base[robot_number].n.w=0; base[robot_number].o.w=0; base[robot_number].a.w=0; base[robot_number].p.w=1;

	fclose(fp);

        printf("\nBase transformation\n");
        printf("%f\t%f\t%f\t%f\n",base[robot_number].n.x, base[robot_number].o.x, base[robot_number].a.x, base[robot_number].p.x);
        printf("%f\t%f\t%f\t%f\n",base[robot_number].n.y, base[robot_number].o.y, base[robot_number].a.y, base[robot_number].p.y);
        printf("%f\t%f\t%f\t%f\n",base[robot_number].n.z, base[robot_number].o.z, base[robot_number].a.z, base[robot_number].p.z);

	sprintf(filename,"%s/%s.pp",robotname[robot_number],robotname[robot_number]);
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}


   for (k = 0; k <= nmbrlinks[robot_number]; k++) /* for each link */
   {
        /* Number of Points, Number of Polygons */
        fscanf(fp,"%d%d", &nmbrpoints[robot_number][k],&nmbrpoly[robot_number][k]);

	maxpoly += nmbrpoly[robot_number][k];

	/* allocate memory */
	nmbrvert[robot_number][k]  = (int *)malloc((unsigned) (nmbrpoly[robot_number][k])*sizeof(int));
	faces[robot_number][k]  = (int **)malloc((unsigned) (nmbrpoly[robot_number][k])*sizeof(int *));
	point[robot_number][k]  = (h_vector *)malloc((unsigned) (nmbrpoints[robot_number][k])*sizeof(h_vector));
	screen[robot_number][k] = (h_vector *)malloc((unsigned) (nmbrpoints[robot_number][k])*sizeof(h_vector));

	/* output and check data */
        printf("\n\nLink %d Data:\n",k);
        printf("%d points\n", nmbrpoints[robot_number][k]);
        printf("%d polygons\n", nmbrpoly[robot_number][k]);

        /* Read x, y, z position of all objects points */
        for (i = 0; i < nmbrpoints[robot_number][k]; i++)
        {
                fscanf(fp,"%f%f%f", &point[robot_number][k][i].x,&point[robot_number][k][i].y,&point[robot_number][k][i].z);
                point[robot_number][k][i].w=1;
         }
 
        /* Read in Polygons */
        /* First integer on the line is the number of vertices in the polygon */
       	/* The following integers are indices into the point list that define */
       	/* the vertices of the polygon */
        minvert = BIG;
        for (i = 0; i < nmbrpoly[robot_number][k]; i++)
        {
                fscanf(fp,"%d", &nmbrvert[robot_number][k][i]);

		/* allocate memory */
		faces[robot_number][k][i] = (int *)malloc((unsigned) (nmbrvert[robot_number][k][i])*sizeof(int));

                for (j = 0; j < nmbrvert[robot_number][k][i]; j++)
                {
                        fscanf(fp,"%d", &faces[robot_number][k][i][j]);
                        if (faces[robot_number][k][i][j] < minvert) minvert = faces[robot_number][k][i][j];
                }
        }

	/* check vertex numbering */ 
        if (minvert != 0)
        printf("\nWARNING: VERTEX INDICES START WITH %d\n", minvert);
        /* If the index starts with 1 (as with a Pascal data file) */
        /* shift to 0 in order to be compatible with C indexing    */
        if (minvert == 1)
        {
                printf("SHIFTING INDICES TO START WITH 0\n");
                for (i = 0; i < nmbrpoly[robot_number][k]; i++)
                for (j = 0; j < nmbrvert[robot_number][k][i]; j++) faces[robot_number][k][i][j]--;
        }
 
 
   } 
	fclose(fp);
}


define_object(object_number)  /* read in the point-polygon description of an object */
int object_number;
{
	int i,j,dummy;
	int minvert; /* used to check minimum vertex index to see if 0 */

	if (object_type[object_number] == 1) /* dynamic object */
	{
		sprintf(filename,"%s/%s.traj",objectname[object_number],objectname[object_number]);
		if ((fp_traj[object_number] = fopen(filename, "r")) == NULL)
		{
			printf( "Error opening file <%s> for input.\n\r",filename);
			exit(1);
		}
   		fscanf(fp_traj[object_number],"%d",&dummy);
		if (nmbrscenes != dummy) printf("INCOMPATIBLE NUMBER OF SCENES in %s\n",filename);
	}

	sprintf(filename,"%s/%s.base",objectname[object_number],objectname[object_number]);
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}
	fscanf(fp,"%f%f%f%f",&o_base[object_number].n.x,  &o_base[object_number].o.x,
			             &o_base[object_number].a.x,  &o_base[object_number].p.x);
    fscanf(fp,"%f%f%f%f",&o_base[object_number].n.y,  &o_base[object_number].o.y,
			             &o_base[object_number].a.y,  &o_base[object_number].p.y);
    fscanf(fp,"%f%f%f%f",&o_base[object_number].n.z,  &o_base[object_number].o.z,
			             &o_base[object_number].a.z,  &o_base[object_number].p.z);
        		          o_base[object_number].n.w=0; o_base[object_number].o.w=0;
			              o_base[object_number].a.w=0; o_base[object_number].p.w=1;
	fclose(fp);


	sprintf(filename,"%s/%s.pp",objectname[object_number],objectname[object_number]);
	if ((fp = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}

        /* Number of Points, Number of Polygons */
        fscanf(fp,"%d%d", &o_nmbrpoints[object_number],&o_nmbrpoly[object_number]);

	/* update maximum number of polygons in a scene */
	maxpoly += o_nmbrpoly[object_number];

	/* allocate memory */
	o_nmbrvert[object_number]  = (int *)malloc((unsigned) (o_nmbrpoly[object_number])*sizeof(int));
	o_faces[object_number]  = (int **)malloc((unsigned) (o_nmbrpoly[object_number])*sizeof(int *));
	o_point[object_number]  = (h_vector *)malloc((unsigned) (o_nmbrpoints[object_number])*sizeof(h_vector));
	o_screen[object_number] = (h_vector *)malloc((unsigned) (o_nmbrpoints[object_number])*sizeof(h_vector));


        /* Read x, y, z position of all objects points */
        for (i = 0; i < o_nmbrpoints[object_number]; i++)
        {
                fscanf(fp,"%f%f%f", &o_point[object_number][i].x,&o_point[object_number][i].y,&o_point[object_number][i].z);
                o_point[object_number][i].w=1;
         }

        /* Read in Polygons */
        /* First integer on the line is the number of vertices in the polygon */
       	/* The following integers are indices into the point list that define */
       	/* the vertices of the polygon */
        minvert = BIG;
        for (i = 0; i < o_nmbrpoly[object_number]; i++)
        {
                fscanf(fp,"%d", &o_nmbrvert[object_number][i]);

		/* allocate memory */
		o_faces[object_number][i] = (int *)malloc((unsigned) (o_nmbrvert[object_number][i])*sizeof(int));

                for (j = 0; j < o_nmbrvert[object_number][i]; j++)
                {
                        fscanf(fp,"%d", &o_faces[object_number][i][j]);
                        if (o_faces[object_number][i][j] < minvert) minvert = o_faces[object_number][i][j];
                }
        }

	/* check vertex numbering */ 
        if (minvert != 0)
        printf("\nWARNING: VERTEX INDICES START WITH %d\n", minvert);
        /* If the index starts with 1 (as with a Pascal data file) */
        /* shift to 0 in order to be compatible with C indexing    */
        if (minvert == 1)
        {
                printf("SHIFTING INDICES TO START WITH 0\n");
                for (i = 0; i < o_nmbrpoly[object_number]; i++)
                for (j = 0; j < o_nmbrvert[object_number][i]; j++) o_faces[object_number][i][j]--;
        }
 
 	fclose(fp);
}



read_script(argc, argv)
int argc;
char *argv[];
{
	int i,dummy;

	if (argc > 1) strcpy(filename,argv[1]);
	else
	{
		printf("Input script filename\n");
		scanf("%s",filename);
	}
	if ((fp_script = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}

    fscanf(fp_script,"%d", &nmbrscenes);

    fscanf(fp_script,"%s", filename);
	if ((fp_view = fopen(filename, "r")) == NULL)
	{
		printf( "Error opening file <%s> for input.\n\r",filename);
		exit(1);
	}

    fscanf(fp_script,"%d", &number_robots);
	if (number_robots > MAX_ROBOTS) printf("TOO MANY ROBOTS\n");

	/* allocate memory for each robot */
        nmbrlinks = (int *)malloc((unsigned) (number_robots)*sizeof(int));;
        a = (float **)malloc((unsigned) (number_robots)*sizeof(float *));;
        d = (float **)malloc((unsigned) (number_robots)*sizeof(float *));;
        alpha = (float **)malloc((unsigned) (number_robots)*sizeof(float *));;
        theta = (float **)malloc((unsigned) (number_robots)*sizeof(float *));;
        joint_type = (char **)malloc((unsigned) (number_robots)*sizeof(char *));;
        base = (h_transform *)malloc((unsigned) (number_robots)*sizeof(h_transform));
	/* arrays from 0 to numberlinks */
        nmbrpoints = (int **)malloc((unsigned) (number_robots)*sizeof(int *));
        nmbrpoly = (int **)malloc((unsigned) (number_robots)*sizeof(int *));
        nmbrvert = (int ***)malloc((unsigned) (number_robots)*sizeof(int **));
        faces = (int ****)malloc((unsigned) (number_robots)*sizeof(int ***));
        point = (h_vector ***)malloc((unsigned) (number_robots)*sizeof(h_vector **));
        screen = (h_vector ***)malloc((unsigned) (number_robots)*sizeof(h_vector **));

	maxpoly = 0; /* initialize total number of possible polygons in a scene */

	for (i=0; i < number_robots; i++)
	{
        fscanf(fp_script,"%s", robotname[i]);

		define_robot(i);

		/* setup angle file name */

		sprintf(filename,"%s/%s.ang",robotname[i],robotname[i]);
		if ((fp_ang[i] = fopen(filename, "r")) == NULL)
		{
			printf( "Error opening file <%s> for input.\n\r",filename);
			exit(1);
		}
   		fscanf(fp_ang[i],"%d",&dummy);
		if (nmbrscenes != dummy) printf("INCOMPATIBLE NUMBER OF SCENES in %s\n",filename);
	}

    fscanf(fp_script,"%d", &number_objects);
	if (number_objects > MAX_OBJECTS) printf("TOO MANY OBJECTS\n");

	/* allocate memory */
        object_type = (int *)malloc((unsigned) (number_objects)*sizeof(int));
        o_nmbrpoints = (int *)malloc((unsigned) (number_objects)*sizeof(int));
        o_nmbrpoly = (int *)malloc((unsigned) (number_objects)*sizeof(int));
        o_nmbrvert = (int **)malloc((unsigned) (number_objects)*sizeof(int *));
        o_faces = (int ***)malloc((unsigned) (number_objects)*sizeof(int **));
        o_point = (h_vector **)malloc((unsigned) (number_objects)*sizeof(h_vector *));
        o_screen = (h_vector **)malloc((unsigned) (number_objects)*sizeof(h_vector *));
        o_base = (h_transform *)malloc((unsigned) (number_objects)*sizeof(h_transform));

	for (i=0; i < number_objects; i++)
	{
        fscanf(fp_script,"%d%s", &object_type[i],objectname[i]);

		define_object(i);
	}

	fclose(fp_script);
 }

init_windows(argc, argv)
int argc;
char *argv[];
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(2*HOR, VER);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("replay_animation");
	
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 2*HOR, 0.0, VER, -1, 1);
	/* NOTE: OpenGL flips both NEAR and FAR clipping planes */ 	
   
}

main(argc, argv)
int argc;
char *argv[];
{
	int i;

	read_script(argc,argv);

	precalculate();

	/* close up open files */
	fclose(fp_view);
	for (i=0; i < number_robots;  i++) fclose(fp_ang[i]);
	for (i=0; i < number_objects; i++)
	if (object_type[i] == 1) /* dynamic object */
	fclose(fp_traj[i]);

	scene = 0;
	direction = 1;

    init_windows(argc, argv);

	glutDisplayFunc(display);
	glutMainLoop();
	return(0);
}

 

