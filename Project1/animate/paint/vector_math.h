
typedef struct
{
        float x;
        float y;
        float z;
        float w; /* homogeneous coordinate */
} h_vector;  /* homogeneous representation of a vector */
 
typedef struct
{
        h_vector n; /* common normal to orientation and approach vectors (x axis) */
        h_vector o; /* orientation vector (y axis) */
        h_vector a; /* approach vector (z axis) */
        h_vector p; /* position vector */
} h_transform; /* a 4 x 4 homogeneous transformation matrix */


/* homogeneous vector functions */

extern float vector_norm(/* v */);		/* return norm (length) of vector */
extern float dot(/* v1, v2 */);			/* dot product of two vectors */
extern h_vector cross( /* v1,v2 */ );		/* cross product of two vectors */
extern h_vector add_vector( /* v1,v2 */ );	/* add two vectore */
extern h_vector scale_vector( /* s,v */ );	/* scale a vector */
extern h_vector unit_vector(/* v */);		/* return a unit vector */

/* homogeneous transform functions */

extern h_vector multTxV(/* T,v */);		/* multiply a transform and a vector */
extern h_transform multTxT(/* T1,T2 */);	/* multiply two transforms (T1*T2) */
extern h_transform assign_transform(/* T */);	/* assign one transform to another */
extern h_transform transform_inverse(/* T */);	/* return inverse of a normalized transform */


/* memory allocation functions */

extern h_vector *hvarray(/* nl,nh */);
/* array of homogeneous vectors with indices [nl,nh] */

extern h_transform *htarray(/* nl,nh */); 
/* array of homogeneous transforms with indices [nl,nh] */

/* (from nrutil.c in numerical recipes) */
extern float *vector(/* nl,nh */);  /* array of floats with indices [nl,nh]  */
extern int *ivector(/* nl,nh */); /* array of integers with indices [nl,nh]  */
extern float **matrix(/* nrl,nrh,ncl,nch */);
/* 2D array of floats with indices [nrl,nrh][ncl,nch] */
